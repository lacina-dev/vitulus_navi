"""Dotazy na svět robota nad mapou — robotí nástroj, bez agenta.

Proč to tady je
---------------
`map_compose` umí svět robota NAKRESLIT.  Tenhle modul ho umí SPOČÍTAT: kde
robot je vůči bodům a zónám, jaké překážky má kolem sebe (známé z mapy, nové,
pohyblivé), kudy se dá projet, jestli se dostane k cíli, kde je nejužší místo,
jestli není uvězněný a kudy by se dal cíl objet.  Výsledek je strukturovaný
(JSON) a k němu jedna česká věta.

Podle pravidla „robotí vlastnost patří do robotího balíčku" je to tady, ve
`vitulus_navi`, vedle `map_compose`, a funguje bez agenta: CLI
`rosrun vitulus_navi world_query locate|analyze|reach|points`.  Agent
(`vitulus_claude.agent.world`) ho jen volá.

Domácí pravidla (stejná jako `map_compose`)
-------------------------------------------
* **Jen čtení.**  Odběry topiců, TF, soubory site bundle.  Žádný publisher,
  žádné volání služby ani akce — ani plánovač MBF se nevolá: GlobalPlanner
  `make_plan` sdílí stav s plánem, který MBF právě počítá, a jeho výsledek by
  se vykreslil v UI jako „aktuální plán".  Proto vlastní A* nad TOUTÉŽ global
  costmapou se stejnou sémantikou (99 = inscribed, 100 = lethal, neznámé
  průjezdné jako `allow_unknown: true`).
* **Nikdy nevyhazuje** a nikdy neblokuje déle než timeout čtení.
* **„Nevím" není „nic tam není".**  Chybějící vrstva je v `missing`, prázdná
  v `empty`; z chybějící costmapy nikdy nevznikne „cesta je volná".

Rámy: všechno v `map` (x = východ, y = sever); směry vůči robotu v jeho
rámu (x = vpřed, y = vlevo).  Úhly v datech ve stupních.
"""
import heapq
import json
import math
import os
import time

try:
    import numpy as np
except Exception:                                       # noqa: BLE001
    np = None

try:
    from . import map_compose as mc
except Exception:                                       # noqa: BLE001
    import importlib.util as _util
    _spec = _util.spec_from_file_location(
        'vitulus_navi_local_map_compose',
        os.path.join(os.path.dirname(os.path.abspath(__file__)), 'map_compose.py'))
    mc = _util.module_from_spec(_spec)
    _spec.loader.exec_module(mc)

# ---- sémantika costmapy (nav_msgs/OccupancyGrid z costmap_2d) -------------
LETHAL = 100          # 254 v costmap_2d -> 100 na drátě
INSCRIBED = 99        # 253 -> 99: střed robota tu znamená kolizi
OCC_MAP = 65          # occupancy mapy: od tohohle obsazeno (map_server)
UNKNOWN = -1

PLAN_RES_M = 0.10     # A* běží na hrubší mřížce (max-pool = konzervativní)
START_ESCAPE_M = 0.45  # robot smí vyjet z vlastní inflace (dok, stěna)
GOAL_TOLERANCE_M = 0.30   # = GlobalPlanner default_tolerance
COST_FACTOR = 3.0         # = GlobalPlanner cost_factor
MAX_EXPAND = 400000
ROBOT_HALF_WIDTH_M = 0.26  # footprint 0.48 × 0.51 m
NARROW_M = 0.45           # průjezd s vůlí pod tímhle = úzké místo
TRAP_RADIUS_M = 2.5
MIN_NEW_CELLS = 4         # menší „nová" oblast costmapy = šum
MIN_CLUSTER_POINTS = 5    # menší lidarový shluk užší než 5 cm = šum

SECTORS = (('front', 'vpředu'), ('front_left', 'vpředu vlevo'),
           ('left', 'vlevo'), ('back_left', 'vzadu vlevo'),
           ('back', 'vzadu'), ('back_right', 'vzadu vpravo'),
           ('right', 'vpravo'), ('front_right', 'vpředu vpravo'))

DOCK_NAMES = ('dock', 'docked')


def _now():
    return time.time()


def _round(value, digits=2):
    try:
        return round(float(value), digits)
    except (TypeError, ValueError):
        return None


# ---------------------------------------------------------------------------
# mřížka
# ---------------------------------------------------------------------------
class Grid(object):
    """OccupancyGrid z `map_compose._collect_grid` jako numpy pole (řádek 0 = jih)."""

    def __init__(self, payload):
        self.w = int(payload['w'])
        self.h = int(payload['h'])
        self.res = float(payload['res'])
        self.ox = float(payload['ox'])
        self.oy = float(payload['oy'])
        self.topic = payload.get('topic')
        self.a = np.asarray(payload['data'], dtype=np.int16).reshape(self.h, self.w)

    def cell(self, x, y):
        c = int(math.floor((float(x) - self.ox) / self.res))
        r = int(math.floor((float(y) - self.oy) / self.res))
        if 0 <= r < self.h and 0 <= c < self.w:
            return r, c
        return None

    def xy(self, r, c):
        return (self.ox + (c + 0.5) * self.res, self.oy + (r + 0.5) * self.res)

    def value(self, x, y):
        rc = self.cell(x, y)
        return None if rc is None else int(self.a[rc])

    def contains(self, x, y):
        return self.cell(x, y) is not None


def costmap_from_map(mapgrid, inscribed_m=ROBOT_HALF_WIDTH_M, soft_m=0.85):
    """Náhradní „costmapa" z uložené mapy, když global costmapa mlčí.

    Lethal = obsazená buňka mapy, inscribed = do poloviny šířky robota, měkký
    náklad do `soft_m` (jako inflation_radius).  Nevidí nové překážky — jen to,
    co je v mapě; výsledek to proto nese v `topic` a volající to hlásí.
    """
    if mapgrid is None:
        return None
    try:
        from scipy import ndimage
    except Exception:                                   # noqa: BLE001
        return None
    occ = mapgrid.a >= OCC_MAP
    dist = ndimage.distance_transform_edt(~occ) * mapgrid.res
    a = np.where(occ, LETHAL, np.where(dist <= inscribed_m, INSCRIBED,
                 np.where(dist <= soft_m, (60 * (1 - (dist - inscribed_m) /
                                                 max(1e-6, soft_m - inscribed_m))).astype(int),
                          0)))
    a = np.where(mapgrid.a == UNKNOWN, np.where(occ, LETHAL, UNKNOWN), a)
    g = Grid.__new__(Grid)
    g.w, g.h, g.res, g.ox, g.oy = mapgrid.w, mapgrid.h, mapgrid.res, mapgrid.ox, mapgrid.oy
    g.topic = 'náhrada z mapy %s (global costmapa mlčí)' % mapgrid.topic
    g.a = a.astype(np.int16)
    g.substitute = True
    return g


def _costgrid(collected):
    """Global costmapa, nebo náhrada z mapy (s poznámkou do `missing`)."""
    cg = grid_of(collected['data'].get('costmap'))
    if cg is not None:
        return cg
    mg = grid_of(collected['data'].get('map'))
    sub = costmap_from_map(mg)
    if sub is not None:
        collected.setdefault('notes', []).append(
            'global costmapa mlčí — plánuji nad uloženou mapou (nové překážky nevidím)')
    return sub


def grid_of(payload):
    if np is None or not isinstance(payload, dict) or payload.get('empty'):
        return None
    try:
        return Grid(payload)
    except Exception:                                   # noqa: BLE001
        return None


# ---------------------------------------------------------------------------
# geometrie vůči robotu
# ---------------------------------------------------------------------------
def to_robot(robot, x, y):
    """(vpřed, vlevo) bodu v rámu robota."""
    dx, dy = float(x) - robot['x'], float(y) - robot['y']
    c, s = math.cos(robot['yaw']), math.sin(robot['yaw'])
    return dx * c + dy * s, -dx * s + dy * c


def sector_of(fwd, left):
    """(kód, česky) jednoho z 8 sektorů pro bod v rámu robota."""
    angle = math.degrees(math.atan2(left, fwd))          # 0 vpřed, + vlevo
    index = int(((angle + 22.5) % 360.0) // 45.0)
    return SECTORS[index]


def relative(robot, x, y):
    fwd, left = to_robot(robot, x, y)
    code, words = sector_of(fwd, left)
    return {'dist_m': _round(math.hypot(fwd, left)),
            'fwd_m': _round(fwd), 'left_m': _round(left),
            'rel_deg': _round(math.degrees(math.atan2(left, fwd)), 0),
            'sector': code, 'where': words}


def azimuth_deg(dx, dy):
    """Azimut od severu po směru hodin (map: +y sever, +x východ)."""
    return (math.degrees(math.atan2(dx, dy)) + 360.0) % 360.0


def point_in_polygon(x, y, ring):
    inside = False
    n = len(ring)
    j = n - 1
    for i in range(n):
        xi, yi = ring[i]
        xj, yj = ring[j]
        if ((yi > y) != (yj > y)) and \
                (x < (xj - xi) * (y - yi) / ((yj - yi) or 1e-12) + xi):
            inside = not inside
        j = i
    return inside


def dist_to_segment(px, py, ax, ay, bx, by):
    vx, vy = bx - ax, by - ay
    seg = vx * vx + vy * vy
    t = 0.0 if seg <= 0 else max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / seg))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


def dist_to_ring(x, y, ring):
    best = None
    for i in range(len(ring)):
        a, b = ring[i], ring[(i + 1) % len(ring)]
        d = dist_to_segment(x, y, a[0], a[1], b[0], b[1])
        best = d if best is None or d < best else best
    return best


def map_to_utm(x, y, datum):
    e0, n0, yaw = datum
    c, s = math.cos(yaw), math.sin(yaw)
    return e0 + x * c - y * s, n0 + x * s + y * c


# ---------------------------------------------------------------------------
# sběr dat (paralelně, přes sběrače map_compose — tentýž kód, tytéž zdroje)
# ---------------------------------------------------------------------------
ALL_SOURCES = ('robot', 'map', 'costmap', 'local_costmap', 'scan',
               'waypoints', 'zones', 'lidar_objects', 'plan', 'geofence')


def collect(sources=ALL_SOURCES, timeout_s=2.0, site=None):
    """{'data': {zdroj: payload}, 'missing': {zdroj: důvod}, 'empty': {...}, 'site'}."""
    sources = tuple(sources or ALL_SOURCES)
    site = site or mc.active_site()
    datum = mc._datum(site)
    timeout = max(0.2, float(timeout_s))
    if set(sources) & {'robot', 'scan', 'lidar_objects'}:
        mc._prime_tf(timeout)
    table = {
        'robot': lambda: mc._collect_robot(timeout),
        'map': lambda: mc._collect_grid(mc.MAP_TOPIC, timeout),
        'costmap': lambda: mc._collect_grid(mc.COSTMAP_TOPIC, timeout),
        'local_costmap': lambda: mc._collect_grid(mc.LOCAL_COSTMAP_TOPIC, timeout),
        'scan': lambda: mc._collect_scan(timeout),
        'waypoints': lambda: mc._collect_waypoints(timeout, site, datum),
        'zones': lambda: mc._collect_zones(timeout),
        'lidar_objects': lambda: _collect_lidar_objects_raw(timeout),
        'plan': lambda: mc._collect_plan(timeout),
        'geofence': lambda: mc._collect_geofence(site, datum),
    }
    jobs = dict((k, table[k]) for k in sources if k in table)
    got = mc._gather(jobs, timeout)
    out = {'data': {}, 'missing': {}, 'empty': {}, 'site': site,
           'datum': datum, 'raw': {}}
    for name in jobs:
        payload, error = got.get(name) or (None, 'neodpovědělo v limitu')
        out['raw'][name] = (payload, error)
        if isinstance(payload, dict) and payload.get('empty'):
            out['empty'][name] = payload['empty']
        elif payload is None:
            out['missing'][name] = error or 'bez důvodu'
        else:
            out['data'][name] = payload
    return out


def _collect_lidar_objects_raw(timeout):
    """Surový JSON z `/safety/lidar_objects` (base_link) — pohyblivé objekty."""
    msg = mc._peek(mc.LIDAR_OBJECTS_TOPIC, timeout)
    raw = getattr(msg, 'data', None) if msg is not None else None
    if not raw:
        return None, 'topic %s mlčí (detekce objektů neběží)' % mc.LIDAR_OBJECTS_TOPIC
    try:
        payload = json.loads(raw)
    except Exception as exc:                            # noqa: BLE001
        return None, 'neplatný JSON: %s' % exc
    if not isinstance(payload, dict):
        return None, 'neznámý tvar JSONu'
    return payload, None


# ---------------------------------------------------------------------------
# body, trasy, zóny
# ---------------------------------------------------------------------------
def points_table(waypoints):
    """{jméno: (x, y)} z vrstvy waypoints."""
    out = {}
    for item in (waypoints or {}).get('points') or []:
        try:
            name, x, y = item
            out[str(name)] = (float(x), float(y))
        except (TypeError, ValueError):
            continue
    return out


def resolve_goal(goal, points):
    """Cíl jako jméno bodu (bez ohledu na velikost písmen), 'x,y', (x, y) nebo dict.

    Vrací (jméno|None, (x, y)) nebo (None, None).
    """
    if goal is None or goal == '':
        return None, None
    if isinstance(goal, dict):
        try:
            return goal.get('name'), (float(goal['x']), float(goal['y']))
        except (KeyError, TypeError, ValueError):
            goal = goal.get('name')
    if isinstance(goal, (list, tuple)) and len(goal) >= 2:
        try:
            return None, (float(goal[0]), float(goal[1]))
        except (TypeError, ValueError):
            return None, None
    text = str(goal).strip()
    for name, xy in points.items():
        if name == text:
            return name, xy
    low = text.lower()
    for name, xy in points.items():
        if name.lower() == low:
            return name, xy
    if ',' in text:
        try:
            a, b = text.split(',', 1)
            return None, (float(a), float(b))
        except ValueError:
            pass
    return None, None


def describe_points(robot, waypoints, zones=None):
    points = points_table(waypoints)
    rows = []
    for name, (x, y) in points.items():
        row = {'name': name, 'x': _round(x), 'y': _round(y)}
        if robot:
            row.update(relative(robot, x, y))
        if zones:
            row['zones'] = [z for z, ring in zones if point_in_polygon(x, y, ring)]
        rows.append(row)
    rows.sort(key=lambda r: (r.get('dist_m') is None, r.get('dist_m') or 0, r['name']))
    paths = []
    for name, line in (waypoints or {}).get('paths') or []:
        length = sum(math.hypot(line[i + 1][0] - line[i][0], line[i + 1][1] - line[i][1])
                     for i in range(len(line) - 1))
        row = {'name': name, 'points': len(line), 'length_m': _round(length)}
        if robot and line:
            near = min(line, key=lambda p: math.hypot(p[0] - robot['x'], p[1] - robot['y']))
            row['nearest'] = relative(robot, near[0], near[1])
            row['start'] = relative(robot, line[0][0], line[0][1])
        paths.append(row)
    zone_rows = []
    for name, ring in zones or []:
        area = 0.5 * abs(sum(ring[i][0] * ring[(i + 1) % len(ring)][1]
                             - ring[(i + 1) % len(ring)][0] * ring[i][1]
                             for i in range(len(ring))))
        cx = sum(p[0] for p in ring) / len(ring)
        cy = sum(p[1] for p in ring) / len(ring)
        row = {'name': name, 'area_m2': _round(area, 1), 'centroid': [_round(cx), _round(cy)],
               'vertices': len(ring)}
        if robot:
            row['inside'] = point_in_polygon(robot['x'], robot['y'], ring)
            row['edge_dist_m'] = _round(dist_to_ring(robot['x'], robot['y'], ring))
            row['centroid_rel'] = relative(robot, cx, cy)
        zone_rows.append(row)
    return {'points': rows, 'paths': paths, 'zones': zone_rows}


def _zones_of(collected):
    zones = (collected['data'].get('zones') or {}).get('zones') or []
    return [(str(n), [(float(a), float(b)) for a, b in ring]) for n, ring in zones]


# ---------------------------------------------------------------------------
# překážky a volný prostor
# ---------------------------------------------------------------------------
def _crop(grid, cx, cy, radius_m):
    c0 = max(0, int((cx - radius_m - grid.ox) / grid.res))
    r0 = max(0, int((cy - radius_m - grid.oy) / grid.res))
    c1 = min(grid.w, int((cx + radius_m - grid.ox) / grid.res) + 1)
    r1 = min(grid.h, int((cy + radius_m - grid.oy) / grid.res) + 1)
    if c1 <= c0 or r1 <= r0:
        return None
    return r0, r1, c0, c1


def obstacles(costgrid, mapgrid, robot, radius_m=8.0, moving=None, limit=12):
    """Souvislé lethal oblasti costmapy kolem robota, nejbližší první.

    `kind`: `static_known` = je v uložené mapě (zeď, plot), `static_new` = v mapě
    není (nová překážka — přesně to, co robota zastaví), `moving` = sedí na
    pohyblivý objekt z `/safety/lidar_objects`.
    """
    if costgrid is None or robot is None:
        return None
    try:
        from scipy import ndimage
    except Exception:                                   # noqa: BLE001
        return None
    box = _crop(costgrid, robot['x'], robot['y'], radius_m)
    if box is None:
        return []
    r0, r1, c0, c1 = box
    sub = costgrid.a[r0:r1, c0:c1] >= LETHAL
    if not sub.any():
        return []
    known_mask = None
    if mapgrid is not None and abs(mapgrid.res - costgrid.res) < 1e-6:
        dr = int(round((costgrid.oy - mapgrid.oy) / mapgrid.res))
        dc = int(round((costgrid.ox - mapgrid.ox) / mapgrid.res))
        mr0, mc0 = r0 + dr, c0 + dc
        if 0 <= mr0 and 0 <= mc0 and mr0 + (r1 - r0) <= mapgrid.h and mc0 + (c1 - c0) <= mapgrid.w:
            occ = mapgrid.a[mr0:mr0 + (r1 - r0), mc0:mc0 + (c1 - c0)] >= OCC_MAP
            known_mask = ndimage.binary_dilation(occ, iterations=3)
    # Nová překážka často DOLÉHÁ na známou zeď; jedna souvislá oblast by ji
    # spolkla (většina buněk je zeď → „známé").  Proto se známé a nové buňky
    # značí zvlášť: nová překážka je vždycky vlastní objekt.
    if known_mask is not None:
        labels = np.zeros(sub.shape, dtype=np.int32)
        lk, nk = ndimage.label(sub & known_mask, structure=np.ones((3, 3), dtype=int))
        ln, nn = ndimage.label(sub & ~known_mask, structure=np.ones((3, 3), dtype=int))
        labels[lk > 0] = lk[lk > 0]
        labels[ln > 0] = ln[ln > 0] + nk
    else:
        labels, _count = ndimage.label(sub, structure=np.ones((3, 3), dtype=int))
    rows = []
    for index, sl in enumerate(ndimage.find_objects(labels), start=1):
        if sl is None:
            continue
        cells = np.argwhere(labels[sl] == index)
        if cells.size == 0:
            continue
        rr = cells[:, 0] + sl[0].start + r0
        cc = cells[:, 1] + sl[1].start + c0
        xs = costgrid.ox + (cc + 0.5) * costgrid.res
        ys = costgrid.oy + (rr + 0.5) * costgrid.res
        d = np.hypot(xs - robot['x'], ys - robot['y'])
        near = int(np.argmin(d))
        # rozměr v rámu robota
        c, s = math.cos(robot['yaw']), math.sin(robot['yaw'])
        fw = (xs - robot['x']) * c + (ys - robot['y']) * s
        lf = -(xs - robot['x']) * s + (ys - robot['y']) * c
        known = None
        if known_mask is not None:
            known = float(known_mask[rr - r0, cc - c0].mean())
        kind = 'unknown' if known is None else ('static_known' if known >= 0.6 else 'static_new')
        if kind == 'static_new' and cells.shape[0] < MIN_NEW_CELLS:
            kind = 'noise'          # jedna dvě buňky = šum lidaru, ne překážka
        rows.append({
            'cells': int(cells.shape[0]),
            'centroid': [_round(xs.mean()), _round(ys.mean())],
            'nearest': [_round(xs[near]), _round(ys[near])],
            'dist_m': _round(d[near]),
            'clear_m': _round(max(0.0, d[near] - ROBOT_HALF_WIDTH_M)),
            'size_fwd_m': _round(fw.max() - fw.min() + costgrid.res),
            'size_left_m': _round(lf.max() - lf.min() + costgrid.res),
            'extent_m': _round(max(xs.max() - xs.min(), ys.max() - ys.min()) + costgrid.res),
            'known_frac': None if known is None else _round(known),
            'kind': kind,
            'rel': relative(robot, xs[near], ys[near]),
        })
    for obj in moving or []:
        best = None
        for row in rows:
            dd = math.hypot(row['centroid'][0] - obj['x'], row['centroid'][1] - obj['y'])
            if dd < 0.6 and (best is None or dd < best[0]):
                best = (dd, row)
        if best:
            best[1]['kind'] = 'moving'
            best[1]['moving'] = obj
    rows.sort(key=lambda r: r['dist_m'])
    return rows[:limit]


def free_rays(costgrid, robot, max_m=5.0, threshold=INSCRIBED):
    """Volná vzdálenost ve směru 8 sektorů (od středu robota do inflace)."""
    if costgrid is None or robot is None:
        return None
    out = {}
    step = costgrid.res
    for index, (code, words) in enumerate(SECTORS):
        ang = robot['yaw'] + math.radians(45.0 * index)
        dist = 0.0
        hit = None
        escaped = False
        while dist < max_m:
            dist += step
            v = costgrid.value(robot['x'] + dist * math.cos(ang),
                               robot['y'] + dist * math.sin(ang))
            if v is None:
                hit = 'mimo mapu'
                break
            if v < threshold:
                escaped = True
            elif escaped or dist > START_ESCAPE_M:
                hit = 'lethal' if v >= LETHAL else 'inflace'
                break
        out[code] = {'free_m': _round(min(dist, max_m)), 'where': words,
                     'stop': hit or 'volno do %.0f m' % max_m}
    return out


# ---------------------------------------------------------------------------
# plánování (A*) — jen výpočet, nic se neposílá
# ---------------------------------------------------------------------------
def _coarse(grid, res=PLAN_RES_M):
    """Max-pool costmapy na `res` (konzervativní: buňka nese nejhorší hodnotu)."""
    cached = getattr(grid, '_coarse', None)
    if cached is not None and cached[2] == res:
        return cached[0], cached[1]
    out = _coarse_compute(grid, res)
    grid._coarse = (out[0], out[1], res)
    return out


def _coarse_compute(grid, res):
    k = max(1, int(round(res / grid.res)))
    h2, w2 = grid.h // k, grid.w // k
    a = grid.a[:h2 * k, :w2 * k].reshape(h2, k, w2, k)
    known = a.max(axis=(1, 3))
    allunk = (a == UNKNOWN).all(axis=(1, 3))
    coarse = np.where(allunk, UNKNOWN, known).astype(np.int16)
    return coarse, k * grid.res


def _clearance(grid):
    """EDT: vzdálenost (m) každé buňky k nejbližší lethal buňce."""
    try:
        from scipy import ndimage
    except Exception:                                   # noqa: BLE001
        return None
    free = grid.a < LETHAL
    if free.all():
        return np.full(grid.a.shape, 99.0)
    return ndimage.distance_transform_edt(free) * grid.res


PLAN_WINDOW_M = 6.0   # A* hledá jen v obdélníku start/cíl + tahle rezerva


def plan(costgrid, start, goal, clearance=None, max_expand=MAX_EXPAND,
         window_m=PLAN_WINDOW_M):
    """A* z `start` do `goal` (map xy).  Jen výpočet.

    {reachable, length_m, points[[x,y]...], min_clearance_m, bottleneck,
     narrow[], reason, expanded, goal_adjusted_m}
    """
    if costgrid is None:
        return {'reachable': None, 'reason': 'global costmapa není k dispozici'}
    coarse, res = _coarse(costgrid)
    H, W = coarse.shape

    def cell(x, y):
        c = int((x - costgrid.ox) / res)
        r = int((y - costgrid.oy) / res)
        return (r, c) if 0 <= r < H and 0 <= c < W else None

    def xy(r, c):
        return costgrid.ox + (c + 0.5) * res, costgrid.oy + (r + 0.5) * res

    s = cell(*start)
    g = cell(*goal)
    if s is None:
        return {'reachable': False, 'reason': 'robot je mimo costmapu'}
    if g is None:
        return {'reachable': False, 'reason': 'cíl je mimo costmapu'}
    esc = int(math.ceil(START_ESCAPE_M / res))
    pad = int(math.ceil(float(window_m) / res)) if window_m else max(H, W)
    rmin, rmax = max(0, min(s[0], g[0]) - pad), min(H - 1, max(s[0], g[0]) + pad)
    cmin, cmax = max(0, min(s[1], g[1]) - pad), min(W - 1, max(s[1], g[1]) + pad)

    def passable(r, c):
        if not (rmin <= r <= rmax and cmin <= c <= cmax):
            return False
        v = coarse[r, c]
        if v < INSCRIBED:
            return True
        # vlastní inflaci kolem startu robot smí opustit (stojí v doku u stěny)
        return v < LETHAL and abs(r - s[0]) <= esc and abs(c - s[1]) <= esc

    goal_adjusted = 0.0
    if not passable(*g):
        tol = int(math.ceil(GOAL_TOLERANCE_M / res))
        best = None
        for dr in range(-tol, tol + 1):
            for dc in range(-tol, tol + 1):
                r, c = g[0] + dr, g[1] + dc
                if 0 <= r < H and 0 <= c < W and coarse[r, c] < INSCRIBED:
                    d = math.hypot(dr, dc)
                    if d * res <= GOAL_TOLERANCE_M and (best is None or d < best[0]):
                        best = (d, (r, c))
        if best is None:
            v = int(coarse[g])
            return {'reachable': False,
                    'reason': ('cíl leží v překážce' if v >= LETHAL else
                               'cíl je v inflaci překážky (robot by se tam nevešel)'),
                    'goal_cost': v}
        goal_adjusted = best[0] * res
        g = best[1]
    sqrt2 = math.sqrt(2.0)
    nbrs = ((1, 0, 1.0), (-1, 0, 1.0), (0, 1, 1.0), (0, -1, 1.0),
            (1, 1, sqrt2), (1, -1, sqrt2), (-1, 1, sqrt2), (-1, -1, sqrt2))
    openh = [(0.0, 0.0, s)]
    came = {s: None}
    cost = {s: 0.0}
    expanded = 0
    found = False
    while openh:
        _f, gc, cur = heapq.heappop(openh)
        if cur == g:
            found = True
            break
        if gc > cost.get(cur, 1e18):
            continue
        expanded += 1
        if expanded > max_expand:
            break
        r, c = cur
        for dr, dc, step in nbrs:
            nr, nc = r + dr, c + dc
            if not (0 <= nr < H and 0 <= nc < W) or not passable(nr, nc):
                continue
            v = coarse[nr, nc]
            pen = 0.0 if v <= 0 else COST_FACTOR * min(v, 98) / 98.0
            ng = gc + step * (1.0 + pen)
            if ng < cost.get((nr, nc), 1e18):
                cost[(nr, nc)] = ng
                came[(nr, nc)] = cur
                h = math.hypot(nr - g[0], nc - g[1])
                heapq.heappush(openh, (ng + h, ng, (nr, nc)))
    if not found:
        why = ('plán přerušen po %d expanzích' % expanded if expanded > max_expand
               else 'cíl je od robota oddělený překážkami (žádná cesta v costmapě)')
        return {'reachable': False, 'reason': why, 'expanded': expanded}
    cells = []
    cur = g
    while cur is not None:
        cells.append(cur)
        cur = came[cur]
    cells.reverse()
    pts = [xy(r, c) for r, c in cells]
    pts[0] = (float(start[0]), float(start[1]))
    length = sum(math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
                 for i in range(len(pts) - 1))
    out = {'reachable': True, 'length_m': _round(length),
           'straight_m': _round(math.hypot(goal[0] - start[0], goal[1] - start[1])),
           'expanded': expanded, 'goal_adjusted_m': _round(goal_adjusted)}
    if clearance is not None:
        worst = None
        narrow = []
        in_narrow = False
        skip = START_ESCAPE_M
        walked = 0.0
        for i, (x, y) in enumerate(pts):
            if i:
                walked += math.hypot(x - pts[i - 1][0], y - pts[i - 1][1])
            rc = costgrid.cell(x, y)
            if rc is None:
                continue
            clr = float(clearance[rc])
            if walked >= skip and (worst is None or clr < worst[0]):
                worst = (clr, x, y, walked)
            gap = clr - ROBOT_HALF_WIDTH_M
            if walked >= skip and gap < NARROW_M:
                if not in_narrow:
                    narrow.append({'at_m': _round(walked), 'x': _round(x), 'y': _round(y),
                                   'gap_m': _round(gap)})
                    in_narrow = True
                else:
                    narrow[-1]['gap_m'] = _round(min(narrow[-1]['gap_m'], gap))
            else:
                in_narrow = False
        if worst:
            out['min_clearance_m'] = _round(worst[0])
            out['bottleneck'] = {'x': _round(worst[1]), 'y': _round(worst[2]),
                                 'at_m': _round(worst[3]),
                                 'gap_m': _round(worst[0] - ROBOT_HALF_WIDTH_M)}
        out['narrow'] = narrow[:6]
    out['points'] = [[_round(x), _round(y)] for x, y in _decimate(pts)]
    return out


def _decimate(pts, keep_every_m=0.3):
    if len(pts) <= 2:
        return pts
    out = [pts[0]]
    acc = 0.0
    for i in range(1, len(pts) - 1):
        acc += math.hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1])
        if acc >= keep_every_m:
            out.append(pts[i])
            acc = 0.0
    out.append(pts[-1])
    return out


def blocked_at(costgrid, start, goal):
    """První místo na přímce robot→cíl, kde je inflace/překážka (mimo start)."""
    if costgrid is None:
        return None
    dx, dy = goal[0] - start[0], goal[1] - start[1]
    dist = math.hypot(dx, dy)
    if dist < 1e-6:
        return None
    step = costgrid.res
    t = 0.0
    escaped = False
    while t < dist:
        t += step
        x, y = start[0] + dx * t / dist, start[1] + dy * t / dist
        v = costgrid.value(x, y)
        if v is None:
            return {'x': _round(x), 'y': _round(y), 'at_m': _round(t), 'what': 'mimo mapu'}
        if v < INSCRIBED:
            escaped = True
        elif escaped or t > START_ESCAPE_M:
            return {'x': _round(x), 'y': _round(y), 'at_m': _round(t),
                    'what': 'překážka' if v >= LETHAL else 'inflace překážky'}
    return None


def trapped(costgrid, robot, radius_m=TRAP_RADIUS_M):
    """Je robot uvězněný?  Flood fill průjezdných buněk v kruhu `radius_m`.

    {trapped, reach_area_m2, exits: [sektory, kudy se dostane na okraj kruhu]}
    """
    if costgrid is None or robot is None:
        return None
    coarse, res = _coarse(costgrid)
    H, W = coarse.shape
    s = (int((robot['y'] - costgrid.oy) / res), int((robot['x'] - costgrid.ox) / res))
    if not (0 <= s[0] < H and 0 <= s[1] < W):
        return {'trapped': None, 'reason': 'robot je mimo costmapu'}
    esc = int(math.ceil(START_ESCAPE_M / res))
    rad = int(radius_m / res)
    seen = {s}
    stack = [s]
    exits = set()
    while stack:
        r, c = stack.pop()
        for dr, dc in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nr, nc = r + dr, c + dc
            if (nr, nc) in seen or not (0 <= nr < H and 0 <= nc < W):
                continue
            dd = math.hypot(nr - s[0], nc - s[1])
            if dd > rad:
                x = costgrid.ox + (c + 0.5) * res
                y = costgrid.oy + (r + 0.5) * res
                exits.add(sector_of(*to_robot(robot, x, y))[0])
                continue
            v = coarse[nr, nc]
            ok = v < INSCRIBED or (v < LETHAL and abs(nr - s[0]) <= esc and abs(nc - s[1]) <= esc)
            if not ok:
                continue
            seen.add((nr, nc))
            stack.append((nr, nc))
    order = [code for code, _ in SECTORS]
    return {'trapped': not exits, 'radius_m': radius_m,
            'reach_area_m2': _round(len(seen) * res * res, 1),
            'exits': [e for e in order if e in exits]}


def via_candidates(costgrid, start, goal, points, exclude=(), clearance=None,
                   blocker=None, limit=4):
    """Mezilehlé body pro objížďku: pojmenované waypointy + body bokem překážky.

    Každý kandidát se ověří plánem start→bod a bod→cíl.  `kind`: `waypoint`
    (jde přes `nav.goto_waypoint`), `detour` (souřadnice bez jména — návrh).
    """
    if costgrid is None:
        return []
    exclude = set(n.lower() for n in exclude or ())
    cands = []
    for name, (x, y) in points.items():
        low = name.lower()
        if low in exclude or low in DOCK_NAMES:
            continue
        if math.hypot(x - goal[0], y - goal[1]) < 0.7 or math.hypot(x - start[0], y - start[1]) < 1.0:
            continue
        cands.append((name, (x, y), 'waypoint'))
    if blocker:
        bx, by = blocker['x'], blocker['y']
        dx, dy = goal[0] - start[0], goal[1] - start[1]
        n = math.hypot(dx, dy) or 1.0
        px, py = -dy / n, dx / n
        for side, sign in (('left', 1.0), ('right', -1.0)):
            for off in (1.0, 1.6, 2.4):
                x, y = bx + sign * off * px, by + sign * off * py
                v = costgrid.value(x, y)
                if v is not None and v < INSCRIBED:
                    cands.append((None, (x, y), 'detour_' + side))
                    break
    rows = []
    for name, xy, kind in cands:
        a = plan(costgrid, start, xy, clearance=clearance, max_expand=MAX_EXPAND // 4)
        if not a.get('reachable'):
            continue
        b = plan(costgrid, xy, goal, clearance=clearance, max_expand=MAX_EXPAND // 4)
        if not b.get('reachable'):
            continue
        rows.append({'name': name, 'xy': [_round(xy[0]), _round(xy[1])],
                     'kind': 'waypoint' if kind == 'waypoint' else 'detour',
                     'side': kind.split('_')[1] if kind.startswith('detour') else None,
                     'total_m': _round(a['length_m'] + b['length_m']),
                     'min_clearance_m': _round(min(a.get('min_clearance_m') or 99,
                                                   b.get('min_clearance_m') or 99)),
                     'reach_ok': True,
                     'points': a.get('points', [])[:-1] + b.get('points', [])})
    if rows:
        best = min(r['total_m'] for r in rows)
        rows = [r for r in rows if r['total_m'] <= max(2.0 * best, best + 6.0)]
    rows.sort(key=lambda r: (r['kind'] != 'waypoint', r['total_m']))
    return rows[:limit]


# ---------------------------------------------------------------------------
# lidar: shluky v rámu robota
# ---------------------------------------------------------------------------
def scan_clusters(scan, robot, mapgrid=None, max_range_m=6.0, gap_m=0.18,
                  min_points=3, moving=None):
    """Shluky lidarových bodů v okolí robota (z bodů skenu v rámu map).

    Každý shluk: sektor, vzdálenost, šířka, zda leží na zdi uložené mapy
    (`known`), zda sedí na pohyblivý objekt.  Malé hloučky (< `min_points`)
    se zahodí jako šum.
    """
    if not scan or robot is None:
        return None
    pts = []
    for x, y in scan.get('points') or []:
        fwd, left = to_robot(robot, x, y)
        d = math.hypot(fwd, left)
        if d <= max_range_m:
            pts.append((math.atan2(left, fwd), d, x, y, fwd, left))
    pts.sort()
    clusters = []
    cur = []
    for p in pts:
        if cur:
            q = cur[-1]
            if math.hypot(p[4] - q[4], p[5] - q[5]) > gap_m:
                clusters.append(cur)
                cur = []
        cur.append(p)
    if cur:
        if clusters and math.hypot(cur[-1][4] - clusters[0][0][4],
                                   cur[-1][5] - clusters[0][0][5]) <= gap_m:
            clusters[0] = cur + clusters[0]
        else:
            clusters.append(cur)
    out = []
    for cl in clusters:
        if len(cl) < min_points:
            continue
        near = min(cl, key=lambda p: p[1])
        width = math.hypot(cl[0][4] - cl[-1][4], cl[0][5] - cl[-1][5])
        if len(cl) < MIN_CLUSTER_POINTS and width < 0.05:
            continue
        known = None
        if mapgrid is not None:
            hits = 0
            total = 0
            for p in cl:
                rc = mapgrid.cell(p[2], p[3])
                if rc is None:
                    continue
                total += 1
                r, c = rc
                win = mapgrid.a[max(0, r - 3):r + 4, max(0, c - 3):c + 4]
                if (win >= OCC_MAP).any():
                    hits += 1
            known = (hits / float(total)) if total else None
        code, words = sector_of(near[4], near[5])
        row = {'sector': code, 'where': words, 'dist_m': _round(near[1]),
               'rel_deg': _round(math.degrees(near[0]), 0),
               'width_m': _round(width), 'points': len(cl),
               'nearest_map': [_round(near[2]), _round(near[3])],
               'known_frac': None if known is None else _round(known),
               'kind': ('unknown' if known is None else
                        'static_known' if known >= 0.6 else 'static_new')}
        for obj in moving or []:
            if math.hypot(near[2] - obj['x'], near[3] - obj['y']) < 0.6:
                row['kind'] = 'moving'
                row['moving'] = obj
        out.append(row)
    out.sort(key=lambda r: r['dist_m'])
    return out


def moving_objects(raw, robot):
    """Objekty z `/safety/lidar_objects` (base_link) přepočtené do map."""
    out = []
    if not isinstance(raw, dict) or robot is None:
        return out
    c, s = math.cos(robot['yaw']), math.sin(robot['yaw'])
    for item in raw.get('objects') or []:
        try:
            lx, ly = float(item['x']), float(item['y'])
        except (KeyError, TypeError, ValueError):
            continue
        out.append({'id': item.get('id'), 'cls': item.get('cls'),
                    'x': robot['x'] + lx * c - ly * s, 'y': robot['y'] + lx * s + ly * c,
                    'range_m': _round(item.get('range_m') or math.hypot(lx, ly)),
                    'bearing_deg': _round(item.get('bearing_deg'), 0),
                    'speed': _round(item.get('speed')), 'size': _round(item.get('size')),
                    'sector': sector_of(lx, ly)[0], 'where': sector_of(lx, ly)[1]})
    return out


# ---------------------------------------------------------------------------
# veřejné dotazy
# ---------------------------------------------------------------------------
def _robot_of(collected):
    robot = collected['data'].get('robot')
    if not robot:
        return None
    return {'x': float(robot['x']), 'y': float(robot['y']), 'yaw': float(robot['yaw']),
            'age_s': robot.get('age_s'), 'source': robot.get('source'),
            'rtk': robot.get('rtk')}


def locate(goal=None, timeout_s=2.0, site=None, collected=None):
    """Kde robot je: map + UTM, natočení, nejbližší body, zóny, stav buňky."""
    t0 = _now()
    col = collected or collect(('robot', 'map', 'costmap', 'waypoints', 'zones'),
                               timeout_s, site)
    robot = _robot_of(col)
    if robot is None:
        return {'ok': False, 'error': 'poloha robota není známá: %s'
                % col['missing'].get('robot', '?'), 'missing': col['missing']}
    azimuth, yaw_deg, point = mc.heading(robot['yaw'])
    out = {'ok': True, 'site': col['site'],
           'pose': {'x': _round(robot['x']), 'y': _round(robot['y']),
                    'yaw_deg': _round(yaw_deg, 1), 'azimuth_deg': _round(azimuth, 0),
                    'facing': point, 'source': robot['source'],
                    'tf_age_s': _round(robot['age_s']), 'rtk': robot['rtk']}}
    if col['datum']:
        e, n = map_to_utm(robot['x'], robot['y'], col['datum'])
        out['utm'] = {'east': _round(e), 'north': _round(n)}
    zones = _zones_of(col)
    desc = describe_points(robot, col['data'].get('waypoints'), zones)
    out['points'] = desc['points'][:8]
    out['zones'] = desc['zones']
    out['in_zones'] = [z['name'] for z in desc['zones'] if z.get('inside')]
    mapgrid = grid_of(col['data'].get('map'))
    costgrid = grid_of(col['data'].get('costmap'))
    out['map_cell'] = None if mapgrid is None else mapgrid.value(robot['x'], robot['y'])
    out['cost_here'] = None if costgrid is None else costgrid.value(robot['x'], robot['y'])
    out['in_map'] = None if mapgrid is None else mapgrid.contains(robot['x'], robot['y'])
    if goal is not None:
        name, xy = resolve_goal(goal, points_table(col['data'].get('waypoints')))
        if xy:
            out['goal'] = dict(relative(robot, *xy), name=name, x=_round(xy[0]), y=_round(xy[1]))
        else:
            out['goal'] = {'error': 'cíl %r neznám' % (goal,)}
    nearest = out['points'][0] if out['points'] else None
    bits = ['robot x=%.2f y=%.2f, míří na %s' % (robot['x'], robot['y'], point)]
    if nearest:
        bits.append('nejblíž bod %s %.1f m %s' % (nearest['name'], nearest['dist_m'], nearest['where']))
    if out['in_zones']:
        bits.append('v zóně %s' % ', '.join(out['in_zones']))
    out['missing'] = col['missing']
    out['summary'] = ', '.join(bits)
    out['wall_s'] = _round(_now() - t0)
    return out


def analyze(goal=None, radius_m=8.0, timeout_s=2.0, site=None, collected=None,
            via=True):
    """Překážky, volné směry, dosažitelnost cíle, úzká místa, uvěznění, objížďky."""
    t0 = _now()
    col = collected or collect(('robot', 'map', 'costmap', 'scan', 'waypoints',
                                'zones', 'lidar_objects'), timeout_s, site)
    robot = _robot_of(col)
    out = {'ok': robot is not None, 'site': col['site'], 'missing': col['missing'],
           'empty': col['empty']}
    if robot is None:
        out['error'] = 'poloha robota není známá: %s' % col['missing'].get('robot', '?')
        return out
    costgrid = _costgrid(col)
    mapgrid = grid_of(col['data'].get('map'))
    moving = moving_objects(col['data'].get('lidar_objects'), robot)
    out['robot'] = {'x': _round(robot['x']), 'y': _round(robot['y']),
                    'yaw_deg': _round(math.degrees(robot['yaw']), 1)}
    out['obstacles'] = obstacles(costgrid, mapgrid, robot, radius_m, moving)
    out['free'] = free_rays(costgrid, robot)
    out['scan_clusters'] = scan_clusters(col['data'].get('scan'), robot, mapgrid,
                                         moving=moving)
    out['moving'] = moving
    out['trapped'] = trapped(costgrid, robot)
    clearance = _clearance(costgrid) if costgrid is not None else None
    if clearance is not None:
        rc = costgrid.cell(robot['x'], robot['y'])
        out['clearance_here_m'] = None if rc is None else _round(clearance[rc])
    points = points_table(col['data'].get('waypoints'))
    if goal is not None:
        name, xy = resolve_goal(goal, points)
        if xy is None:
            out['reach'] = {'reachable': None, 'reason': 'cíl %r neznám' % (goal,),
                            'known_points': sorted(points)}
        else:
            reach = plan(costgrid, (robot['x'], robot['y']), xy, clearance)
            reach['goal'] = dict(relative(robot, *xy), name=name,
                                 x=_round(xy[0]), y=_round(xy[1]))
            block = blocked_at(costgrid, (robot['x'], robot['y']), xy)
            reach['straight_blocked'] = block
            if block and out.get('obstacles'):
                reach['blocker'] = min(
                    out['obstacles'],
                    key=lambda o: math.hypot(o['nearest'][0] - block['x'],
                                             o['nearest'][1] - block['y']))
            if via and costgrid is not None and (
                    not reach.get('reachable') or via == 'always'):
                reach['via'] = via_candidates(
                    costgrid, (robot['x'], robot['y']), xy, points,
                    exclude=[name] if name else (), clearance=clearance, blocker=block)
            out['reach'] = reach
    out['notes'] = col.get('notes') or []
    out['costmap_source'] = None if costgrid is None else costgrid.topic
    out['summary'] = summarize(out)
    if out['notes']:
        out['summary'] += ' (%s)' % '; '.join(out['notes'])
    out['wall_s'] = _round(_now() - t0)
    return out


def reach(goal, start=None, timeout_s=2.0, site=None, collected=None, via=True):
    """Dosažitelnost jednoho cíle (volitelně z jiného startu)."""
    col = collected or collect(('robot', 'map', 'costmap', 'waypoints'), timeout_s, site)
    robot = _robot_of(col)
    costgrid = _costgrid(col)
    points = points_table(col['data'].get('waypoints'))
    name, xy = resolve_goal(goal, points)
    if xy is None:
        return {'ok': False, 'error': 'cíl %r neznám' % (goal,), 'known_points': sorted(points)}
    if start is not None:
        _sn, sxy = resolve_goal(start, points)
    else:
        sxy = (robot['x'], robot['y']) if robot else None
    if sxy is None:
        return {'ok': False, 'error': 'start neznám (poloha robota chybí?)',
                'missing': col['missing']}
    clearance = _clearance(costgrid) if costgrid is not None else None
    out = plan(costgrid, sxy, xy, clearance)
    out['ok'] = out.get('reachable') is not None
    out['goal'] = {'name': name, 'x': _round(xy[0]), 'y': _round(xy[1])}
    block = blocked_at(costgrid, sxy, xy)
    out['straight_blocked'] = block
    if costgrid is not None and (not out.get('reachable') or via == 'always'):
        out['via'] = via_candidates(costgrid, sxy, xy, points,
                                    exclude=[name] if name else (),
                                    clearance=clearance, blocker=block)
    out['missing'] = col['missing']
    out['notes'] = col.get('notes') or []
    return out


def points(timeout_s=2.0, site=None, collected=None):
    col = collected or collect(('robot', 'waypoints', 'zones'), timeout_s, site)
    robot = _robot_of(col)
    out = describe_points(robot, col['data'].get('waypoints'), _zones_of(col))
    out.update({'ok': True, 'site': col['site'], 'missing': col['missing'],
                'source': (col['data'].get('waypoints') or {}).get('source')})
    return out


def summarize(result):
    """Jedna česká věta z výsledku `analyze()`."""
    bits = []
    obs = [o for o in (result.get('obstacles') or []) if o['dist_m'] <= 3.0]
    new = [o for o in obs if o['kind'] in ('static_new', 'moving')]
    if new:
        o = new[0]
        bits.append('%s překážka %.1f m %s (%.1f × %.1f m)' % (
            'pohyblivá' if o['kind'] == 'moving' else 'nová (není v mapě)',
            o['dist_m'], o['rel']['where'], o['size_fwd_m'], o['size_left_m']))
    elif obs:
        bits.append('nejbližší známá překážka %.1f m %s' % (obs[0]['dist_m'], obs[0]['rel']['where']))
    elif result.get('obstacles') is not None:
        bits.append('do 3 m žádná překážka v costmapě')
    else:
        bits.append('costmapa nedostupná — překážky nevím')
    tr = result.get('trapped') or {}
    if tr.get('trapped'):
        bits.append('ROBOT JE UVĚZNĚNÝ (z kruhu %.1f m nevede cesta)' % tr['radius_m'])
    reach_ = result.get('reach')
    if reach_:
        goal = (reach_.get('goal') or {})
        label = goal.get('name') or 'cíl'
        if reach_.get('reachable'):
            s = 'cesta k %s %.1f m' % (label, reach_['length_m'])
            if reach_.get('bottleneck') and reach_['bottleneck']['gap_m'] < NARROW_M:
                s += ', nejužší místo vůle %.2f m' % reach_['bottleneck']['gap_m']
            if reach_.get('straight_blocked'):
                s += ' (přímo to nejde, objíždí)'
            bits.append(s)
        elif reach_.get('reachable') is False:
            bits.append('%s NEDOSAŽITELNÝ: %s' % (label, reach_.get('reason')))
        vias = reach_.get('via') or []
        if vias and not reach_.get('reachable'):
            v = vias[0]
            bits.append('objížďka přes %s (%.1f m)' % (v['name'] or ('bod %s' % v['side']), v['total_m']))
    return '; '.join(bits)
