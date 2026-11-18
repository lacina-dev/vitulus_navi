"""The owner's corridors as a soft preference of the global planner (T36).

A corridor is a line the owner drew in the map editor (object type "corridor"
of /map_objects/objects: points in the map frame, props.width_m). Until now
only the editor and the agent's description of places knew it. With
navi_manager.yaml map_corridor_prefer navi_man sends /navi_manager/corridor_cost:
a grid with the geometry of the last /navi_manager/map, `cost` inside the
corridors and 0 everywhere else. The corridor layer of the GLOBAL costmap
(plugins/corridor_layer.cpp, after inflation) and a planner started with a
lower neutral_cost (config/global_costmap_corridor.yaml) make a corridor cell
1.5 * cost cheaper for the planner than it is today (cost 7: 39.5 instead of
50, a way along a corridor may be 1.27 times longer and still wins), and every
other cell cost exactly what it costs today. So a plan that gains nothing from
a corridor is priced as without the switch, nothing becomes an obstacle and
nothing that is one opens. Without a corridor the grid is empty.

Around the dock there is no corridor: the cells within dock_radius of the line
DOCKED - DOCK - UNDOCK are 0 even inside a corridor, so docking and undocking
plan as without the switch.

`connects` of a corridor is not used: the planner takes a corridor when it lies
on the way, whatever it was drawn to connect.

No ROS in here; numpy is imported on first use (navi_man must start without it).
"""
import array
import json
import math

FRAME = 'map'
DEFAULT_WIDTH_M = 0.60      # vitulus_mapping objects.DEFAULT_CORRIDOR_M
MAX_WIDTH_M = 10.0
COST_MAX = 60               # plugins/corridor_layer.cpp MAX_V; the planner of the switch uses up to 8
TOPIC = '/navi_manager/corridor_cost'
LIBRARY = 'libvitulus_navi_corridor_layer.so'
LIBRARY_VERSION = 4         # plugins/corridor_layer.cpp vitulus_navi_corridor_layer_version()
DOCK_POINTS = ('DOCKED', 'DOCK', 'UNDOCK')
TODAY = (50.0, 3.0, 253)    # neutral_cost, cost_factor, lethal_cost the layer is built for (TODAY_* there)


def parse(raw):
    """/map_objects/objects JSON -> (site, corridors, dock): corridors =
    ((name, width_m, ((x, y), ...)), ...) of the active ones, sorted, so the
    result compares as a signature; dock = the points DOCKED, DOCK, UNDOCK the
    map has, in that order."""
    doc = json.loads(raw)
    out, dock = [], {}
    for o in doc.get('objects') or []:
        if o.get('type') == 'waypoint' and o.get('name') in DOCK_POINTS:
            try:
                x, y = o['points'][0]
                if math.isfinite(float(x)) and math.isfinite(float(y)):
                    dock[o['name']] = (float(x), float(y))
            except (TypeError, ValueError, KeyError, IndexError):
                pass
            continue
        if o.get('type') != 'corridor' or not o.get('active', True):
            continue
        try:
            pts = tuple((float(x), float(y)) for x, y in o.get('points') or [])
            width = float((o.get('props') or {}).get('width_m') or DEFAULT_WIDTH_M)
        except (TypeError, ValueError):
            continue
        if len(pts) < 2 or not all(math.isfinite(v) for p in pts for v in p):
            continue
        if not (math.isfinite(width) and 0.0 < width <= MAX_WIDTH_M):
            width = DEFAULT_WIDTH_M
        out.append((str(o.get('name', '')), width, pts))
    return doc.get('site'), tuple(sorted(out)), tuple(dock[n] for n in DOCK_POINTS if n in dock)


def clamp_cost(cost):
    try:
        cost = int(round(float(cost)))
    except (TypeError, ValueError):
        return 0
    return max(0, min(COST_MAX, cost))


def geometry(info):
    p, q = info.origin.position, info.origin.orientation
    return (int(info.width), int(info.height), float(info.resolution), p.x, p.y, q.x, q.y, q.z, q.w)


def mask(corridors, info):
    """bool array [iy, ix]: the cells whose centre is no further than half the
    width from a corridor's line."""
    import numpy as np
    w, h, res = int(info.width), int(info.height), float(info.resolution)
    ox, oy = info.origin.position.x, info.origin.position.y
    q = info.origin.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    inside = np.zeros((h, w), dtype=bool)
    for _name, width, pts in corridors:
        half = width / 2.0
        g = []                                        # the line in grid metres
        for x, y in pts:
            dx, dy = x - ox, y - oy
            if yaw != 0.0:
                dx, dy = c * dx + s * dy, -s * dx + c * dy
            g.append((dx, dy))
        for (ax, ay), (bx, by) in zip(g, g[1:]):
            i0 = max(0, int(math.floor((min(ax, bx) - half) / res)))
            i1 = min(w, int(math.floor((max(ax, bx) + half) / res)) + 1)
            j0 = max(0, int(math.floor((min(ay, by) - half) / res)))
            j1 = min(h, int(math.floor((max(ay, by) + half) / res)) + 1)
            if i0 >= i1 or j0 >= j1:
                continue
            cx = (np.arange(i0, i1) + 0.5) * res      # cell centres
            cy = ((np.arange(j0, j1) + 0.5) * res)[:, None]
            ex, ey = bx - ax, by - ay
            ll = ex * ex + ey * ey
            if ll > 0.0:
                t = np.clip(((cx - ax) * ex + (cy - ay) * ey) / ll, 0.0, 1.0)
            else:
                t = 0.0
            d2 = (cx - (ax + t * ex)) ** 2 + (cy - (ay + t * ey)) ** 2
            inside[j0:j1, i0:i1] |= d2 <= half * half
    return inside


def dock_area(dock, radius):
    """The surroundings of the dock as a line with a width (no corridor there)."""
    if not dock or not radius > 0.0:
        return ()
    pts = tuple(dock) if len(dock) > 1 else (dock[0], dock[0])
    return (('dock', 2.0 * float(radius), pts),)


def cost_grid(corridors, info, cost, dock=(), dock_radius=0.0):
    """int8 array [iy, ix] for /navi_manager/corridor_cost: `cost` in the cells of
    the corridors (not around the dock), 0 elsewhere. None when there is nothing
    to prefer (no corridor, cost 0, no cell of the map in a corridor)."""
    import numpy as np
    cost = clamp_cost(cost)
    if not corridors or cost <= 0 or int(info.width) * int(info.height) == 0:
        return None
    inside = mask(corridors, info)
    area = dock_area(dock, dock_radius)
    if area:
        inside &= ~mask(area, info)
    if not inside.any():
        return None
    return np.where(inside, cost, 0).astype(np.int8)


class Corridors(object):
    """What navi_man has to send: remembers the corridors, the geometry of the
    last map sent and what went out, so the grid is built only on a change."""

    def __init__(self, cost, dock_radius=0.0):
        self.cost = clamp_cost(cost)
        try:
            self.dock_radius = max(0.0, min(20.0, float(dock_radius)))
        except (TypeError, ValueError):
            self.dock_radius = 0.0
        self.dock = ()
        self.site = None
        self.corridors = ()
        self.info = None            # MapMetaData of the last /navi_manager/map
        self.frame = None
        self._sent = None           # key of what is on the topic
        self.cells = 0              # corridor cells in the last grid
        self.error = None

    def on_objects(self, raw):
        """False = bad message, the last corridors stay."""
        try:
            self.site, self.corridors, self.dock = parse(raw)
        except (ValueError, TypeError, KeyError, AttributeError):
            return False
        return True

    def on_map(self, msg):
        self.info, self.frame = msg.info, msg.header.frame_id

    def build(self, grid_type, served=None):
        """The grid to publish, or None when the topic already says it.
        `served` = the site mapping_manager serves (None = not known): another
        site's corridors are in another map."""
        self.error = None
        if self.info is None:
            return None
        corridors = self.corridors
        if served is not None and self.site != served:
            corridors = ()
        if self.frame != FRAME:
            corridors = ()                              # indoor / odom maps: none
        key = (corridors, geometry(self.info), self.cost, self.dock, self.dock_radius) if corridors else ()
        if key == self._sent:
            return None
        arr = None
        if corridors:
            try:
                arr = cost_grid(corridors, self.info, self.cost, self.dock, self.dock_radius)
            except Exception as e:  # noqa: BLE001 - never in the map's way
                self.error = repr(e)
        if arr is None:
            key = ()                                    # empty grid: no preference
            if key == self._sent:
                return None
        out = grid_type()
        out.header.frame_id = FRAME
        self.cells = 0
        if arr is not None:
            out.info = self.info
            # int8[] as an array: 1 byte a cell in navi_man (the latch keeps the
            # message), a list would be 8 and more
            out.data = array.array('b')
            out.data.frombytes(arr.tobytes())
            self.cells = int(arr.astype(bool).sum())
        self._sent = key
        return out


def library_path(prefixes=None, environ=None):
    """Where pluginlib will look for the built corridor layer, or None."""
    import os
    env = os.environ if environ is None else environ
    if prefixes is None:
        prefixes = [p for p in env.get('CMAKE_PREFIX_PATH', '').split(os.pathsep) if p]
    dirs = [os.path.join(p, 'lib') for p in prefixes]
    dirs += [d for d in env.get('LD_LIBRARY_PATH', '').split(os.pathsep) if d]
    for d in dirs:
        path = os.path.join(d, LIBRARY)
        if os.path.isfile(path):
            return path
    return None


_PROBE = (
    "import ctypes, sys\n"
    "lib = ctypes.CDLL(sys.argv[1])\n"
    "f = lib.vitulus_navi_corridor_layer_version\n"
    "f.restype = ctypes.c_int\n"
    "if f() != int(sys.argv[2]):\n"
    "    sys.exit(3)\n"
    "if len(sys.argv) > 3:\n"
    "    e = lib.vitulus_navi_corridor_layer_table_error\n"
    "    e.argtypes, e.restype = [ctypes.c_double, ctypes.c_double, ctypes.c_int], ctypes.c_double\n"
    "    sys.exit(0 if e(float(sys.argv[3]), float(sys.argv[4]), int(sys.argv[5])) <= 1e-6 else 4)\n"
    "sys.exit(0)\n")


def planner_values(config_dir):
    """((neutral, factor, lethal) with the switch, the same of today): what
    move_base_flex gets from global_planner_params.yaml, and with
    global_costmap_corridor.yaml loaded over it."""
    import os
    import yaml
    with open(os.path.join(config_dir, 'global_planner_params.yaml')) as f:
        today = (yaml.safe_load(f) or {}).get('GlobalPlanner') or {}
    with open(os.path.join(config_dir, 'global_costmap_corridor.yaml')) as f:
        now = dict(today)
        now.update((yaml.safe_load(f) or {}).get('GlobalPlanner') or {})

    def values(d):
        return (float(d.get('neutral_cost', TODAY[0])), float(d.get('cost_factor', TODAY[1])),
                int(d.get('lethal_cost', TODAY[2])))
    return values(now), values(today)


def config_problem(config_dir):
    """None when the planner without the switch is the one the layer was written for."""
    try:
        _now, today = planner_values(config_dir)
    except Exception as e:  # noqa: BLE001
        return 'the planner values of the switch cannot be read: %r' % (e,)
    if today != TODAY:
        return ('global_planner_params.yaml has neutral_cost, cost_factor, lethal_cost %r, the corridor layer '
                'is built for %r' % (today, TODAY))
    return None


def library_problem(path=None, timeout=20.0, config_dir=None):
    """None when move_base_flex can load the corridor layer and the layer will
    run, else why not (the switch then stays off, planner parameters included:
    a plugin that does not load takes the planner down, and the planner's lower
    neutral_cost without the layer's exact table would weaken the inflation).
    The library is loaded in a short process of its own, not in navi_man: it has
    to be there, load with all it links to, and be this version of the layer;
    the package has to export it to costmap_2d; and with config_dir, the planner
    values of the switch have to give, outside the corridors, exactly the prices
    of the planner without it (asked from the library itself)."""
    import subprocess
    import sys
    args = []
    if config_dir is not None:
        problem = config_problem(config_dir)
        if problem:
            return problem
        now = planner_values(config_dir)[0]
        args = [repr(now[0]), repr(now[1]), str(now[2])]
    if path is None:
        path = library_path()
    if path is None:
        return '%s is not built' % LIBRARY
    try:
        r = subprocess.run([sys.executable, '-c', _PROBE, path, str(LIBRARY_VERSION)] + args,
                           stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=timeout)
    except Exception as e:  # noqa: BLE001
        return '%s could not be tried: %r' % (path, e)
    if r.returncode == 3:
        return '%s is another version of the layer (rebuild vitulus_navi)' % path
    if r.returncode == 4:
        return ('with neutral_cost %g and cost_factor %g (config/global_costmap_corridor.yaml) the planner '
                'cannot pay exactly today\'s prices' % (now[0], now[1]))
    if r.returncode != 0:
        tail = (r.stdout or b'').decode('utf-8', 'replace').strip().splitlines()[-1:]
        return '%s does not load: %s' % (path, tail[0] if tail else 'exit %d' % r.returncode)
    return export_problem()


def export_problem():
    """None when pluginlib finds the layer's class in the vitulus_navi it sees."""
    try:
        import rospkg
        rp = rospkg.RosPack()
        prefix = rp.get_path('vitulus_navi')
        exports = [x.replace('${prefix}', prefix)
                   for x in rp.get_manifest('vitulus_navi').get_export('costmap_2d', 'plugin')]
    except Exception as e:  # noqa: BLE001
        return 'vitulus_navi not found by rospack: %r' % (e,)
    for xml in exports:
        try:
            with open(xml) as f:
                if 'vitulus_navi::CorridorLayer' in f.read():
                    return None
        except (IOError, OSError):
            pass
    return 'vitulus_navi does not export vitulus_navi::CorridorLayer to costmap_2d'
