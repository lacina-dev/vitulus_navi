#!/usr/bin/env python3
"""The owner's corridors as a soft preference of the global planner (T36), offline.

    python3 test/test_map_corridor.py      (sourced workspace)

navi_manager.yaml map_corridor_prefer (default false): navi_man sends
/navi_manager/corridor_cost (map_corridor_cost inside the corridors of
/map_objects/objects, 0 elsewhere) and starts move_base_flex with
config/global_costmap_corridor.yaml: the corridor layer last in the global
costmap and a planner with a lower neutral_cost, so that a corridor cell is
cheaper for the planner and every other cell costs it exactly what it costs
today. Here: the rasterising, when navi_man sends the grid (only on a change),
the switch off = nothing at all, the launch switch (off = parameters exactly as
before), the check that the layer can be loaded, and the layer's arithmetic
through the built library (skipped where it is not built). The layer in the
costmap and the planner are tried on a private master: tools/corridor_plan_trial.py.
"""
import importlib.machinery
import importlib.util
import json
import math
import os
import random
import sys
import types
import unittest

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:1'     # never the robot's master

HERE = os.path.dirname(os.path.abspath(__file__))
PKG = os.path.dirname(HERE)
SRC = os.path.join(PKG, 'src')
# this tree's vitulus_navi first ($(find vitulus_navi) in the launch file)
os.environ['ROS_PACKAGE_PATH'] = os.path.dirname(PKG) + ':' + os.environ.get('ROS_PACKAGE_PATH', '/opt/ros/noetic/share')

import numpy as np  # noqa: E402
import roslaunch.config  # noqa: E402
import yaml  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from std_msgs.msg import String  # noqa: E402

try:
    import docker  # noqa: F401
except ImportError:
    sys.modules['docker'] = types.ModuleType('docker')

sys.path.insert(0, SRC)
from navi_man import map_corridor, map_dedup, path_rules  # noqa: E402
sys.path.remove(SRC)

LAUNCH = os.path.join(PKG, 'launch', 'vitulus_navi_mbf.launch')
W, H, RES, OX, OY = 80, 60, 0.05, -1.0, -0.5        # 4 x 3 m


def load():
    path = os.path.join(PKG, 'nodes', 'navi_man')
    loader = importlib.machinery.SourceFileLoader('navi_man_corridor_test', path)
    spec = importlib.util.spec_from_loader('navi_man_corridor_test', loader)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def grid(fill=0, w=W, h=H, frame='map', ox=OX, oy=OY, res=RES, yaw=0.0):
    m = OccupancyGrid()
    m.header.frame_id = frame
    m.info.width, m.info.height, m.info.resolution = w, h, res
    m.info.origin.position.x, m.info.origin.position.y = ox, oy
    m.info.origin.orientation.z, m.info.origin.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
    m.data = tuple([fill] * (w * h))
    return m


def corridor(points, width=0.6, name='c', active=True, **more):
    o = {'id': 'o_' + name, 'family': 'place', 'type': 'corridor', 'name': name, 'geometry': 'line',
         'points': [list(p) for p in points], 'active': active,
         'props': {'width_m': width, 'connects': []}}
    o.update(more)
    return o


def objects_msg(objs, site='garden'):
    return json.dumps({'site': site, 'seq': 1, 'objects': objs})


def brute(points, width, info):
    """Cell by cell: centre no further than width / 2 from the line."""
    out = np.zeros((info.height, info.width), dtype=bool)
    q = info.origin.orientation
    yaw = 2.0 * math.atan2(q.z, q.w)
    for j in range(info.height):
        for i in range(info.width):
            gx, gy = (i + 0.5) * info.resolution, (j + 0.5) * info.resolution
            x = info.origin.position.x + math.cos(yaw) * gx - math.sin(yaw) * gy
            y = info.origin.position.y + math.sin(yaw) * gx + math.cos(yaw) * gy
            best = min(path_rules._dist_segment((x, y), a, b) for a, b in zip(points, points[1:]))
            out[j, i] = best <= width / 2.0 + 1e-9
    return out


class Raster(unittest.TestCase):

    def test_parse(self):
        zone = {'family': 'zone', 'type': 'zone', 'name': 'Z', 'points': [[0, 0], [1, 0], [1, 1]]}
        site, cs, dock = map_corridor.parse(objects_msg([
            zone,
            corridor([(0, 0), (1, 0)], 0.8, 'b'),
            corridor([(0, 0), (1, 1)], 0.6, 'off', active=False),
            corridor([(0, 0)], 0.6, 'one point'),
            corridor([(0, 0), (2, 0)], None, 'a'),            # no width -> default
            corridor([(0, 0), (2, 0)], -1, 'neg'),            # bad width -> default
            corridor([(0, 0), ('x', 0)], 0.6, 'bad'),
        ]))
        self.assertEqual(site, 'garden')
        self.assertEqual([(n, w) for n, w, _p in cs], [('a', 0.6), ('b', 0.8), ('neg', 0.6)])
        self.assertEqual(cs[1][2], ((0.0, 0.0), (1.0, 0.0)))
        self.assertEqual(map_corridor.parse(json.dumps({'site': None}))[1:], ((), ()))
        self.assertEqual(dock, ())
        self.assertEqual(map_corridor.DEFAULT_WIDTH_M, 0.60)

    def test_straight_corridor_cells(self):
        info = grid().info
        # along x at y = 0.5 (a cell border), 0.6 m wide: 6 rows each side
        m = map_corridor.mask((('c', 0.6, ((0.0, 0.5), (2.0, 0.5))),), info)
        col = m[:, 40]                                   # x = 1.0, the middle
        self.assertEqual(int(col.sum()), 12)
        self.assertEqual(list(np.flatnonzero(col)), list(range(14, 26)))
        self.assertEqual(int(m[20].sum()), 40 + 12)      # 2 m + a round end of 0.3 m each side
        self.assertFalse(m[:, 10].any())                 # x = -0.5: before the start cap

    def test_matches_cell_by_cell(self):
        rnd = random.Random(7)
        for yaw in (0.0, 0.4):
            info = grid(yaw=yaw).info
            for _ in range(6):
                pts = tuple((rnd.uniform(-1.5, 3.5), rnd.uniform(-1.0, 3.0)) for _ in range(rnd.randint(2, 5)))
                width = rnd.choice((0.2, 0.6, 1.1))
                got = map_corridor.mask((('c', width, pts),), info)
                want = brute(pts, width, info)
                diff = got != want
                # only cells exactly on the border may differ (float)
                self.assertLessEqual(int(diff.sum()), 2, (yaw, pts, width))

    def test_cost_grid(self):
        info = grid().info
        cs = (('c', 0.6, ((0.0, 0.5), (2.0, 0.5))),)
        g = map_corridor.cost_grid(cs, info, 4)
        self.assertEqual(g.dtype, np.int8)
        self.assertEqual(g.shape, (H, W))
        self.assertEqual(sorted(set(g.ravel().tolist())), [0, 4])
        self.assertTrue((g == 4).tolist() == map_corridor.mask(cs, info).tolist())     # the corridor, 0 elsewhere
        self.assertIsNone(map_corridor.cost_grid((), info, 4))                       # no corridor
        self.assertIsNone(map_corridor.cost_grid(cs, info, 0))                       # no strength
        self.assertIsNone(map_corridor.cost_grid((('far', 0.6, ((50, 50), (60, 50))),), info, 4))
        self.assertIsNone(map_corridor.cost_grid(cs, grid(w=0, h=0).info, 4))        # "no map" grid
        self.assertEqual(int(map_corridor.cost_grid(cs, info, 500).max()), map_corridor.COST_MAX)
        self.assertEqual(map_corridor.clamp_cost('x'), 0)
        self.assertEqual(map_corridor.clamp_cost(-3), 0)

    def test_detour_the_cost_buys(self):
        """The planner of the switch (38 + 1.5 c') pays outside the corridors what the
        planner of today pays (50 + 3 c), and 1.5 * strength less in a corridor."""
        with open(os.path.join(PKG, 'config', 'global_planner_params.yaml')) as f:
            gp = yaml.safe_load(f)['GlobalPlanner']
        with open(os.path.join(PKG, 'config', 'global_costmap_corridor.yaml')) as f:
            y = yaml.safe_load(f)
        with open(os.path.join(PKG, 'config', 'navi_manager.yaml')) as f:
            strength = yaml.safe_load(f)['map_corridor_cost']
        now, layer = y['GlobalPlanner'], y['global_costmap']['corridor_layer']
        self.assertEqual(list(layer), ['topic'])
        # today's planner is the one the layer is built for
        self.assertEqual((gp['neutral_cost'], gp['cost_factor'], gp['lethal_cost']), map_corridor.TODAY)
        with open(os.path.join(PKG, 'plugins', 'corridor_layer.cpp')) as f:
            cpp = f.read()
        for line in ('TODAY_NEUTRAL = 50.0;', 'TODAY_FACTOR = 3.0;', 'TODAY_LETHAL = 253;'):
            self.assertIn(line, cpp)
        self.assertEqual(sorted(now), ['cost_factor', 'neutral_cost'])        # nothing else of the planner changes
        self.assertIsInstance(now['neutral_cost'], int)                       # an int parameter of the planner
        # exact in whole cells: c' = slope * c + offset
        slope = gp['cost_factor'] / now['cost_factor']
        offset = (gp['neutral_cost'] - now['neutral_cost']) / now['cost_factor']
        self.assertEqual((slope, offset), (2.0, 8.0))
        for c in range(0, 252):
            self.assertEqual(now['neutral_cost'] + now['cost_factor'] * (slope * c + offset),
                             gp['neutral_cost'] + gp['cost_factor'] * c)
        self.assertTrue(1 <= strength <= offset)
        ratio = gp['neutral_cost'] / (gp['neutral_cost'] - now['cost_factor'] * strength)
        self.assertTrue(1.25 < ratio < 1.30, ratio)                           # takes a detour of 25 %, not of 30 %


class Sink(object):
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class NaviMan(unittest.TestCase):
    """The real navi_man methods around a map going out."""

    @classmethod
    def setUpClass(cls):
        cls.M = load()

    def node(self, on, cost=4, built=True):
        M = self.M
        n = M.Node.__new__(M.Node)
        n.params = {}
        if on is not None:
            n.params.update(map_corridor_prefer=on, map_corridor_cost=cost)
        n.nm_map_pub = Sink()
        n._nm_corridor_pub = Sink()
        n.rule_set = path_rules.RuleSet()
        n._nm_map_dedup = map_dedup.MapDedup()
        n._nm_map_on_change = None
        self._lib = M.map_corridor.library_problem
        M.map_corridor.library_problem = (lambda *a, **k: None) if built else (lambda *a, **k: 'does not load: x')
        self.addCleanup(setattr, M.map_corridor, 'library_problem', self._lib)
        return n

    CS = [corridor([(0.0, 0.5), (2.0, 0.5)])]

    def test_off_is_nothing(self):
        for on in (None, False):
            n = self.node(on)
            n.callback_map_edits(String(objects_msg(self.CS)))
            n._nm_map_publish(grid(0))
            n.callback_rules_status(String(json.dumps({'serving': {'site': 'garden'}})))
            self.assertEqual(n._nm_corridor_pub.msgs, [])
            self.assertEqual(len(n.nm_map_pub.msgs), 1)
            self.assertEqual(n._nm_corridor_launch_args(), [])

    def test_not_built_stays_off(self):
        n = self.node(True, built=False)
        n.callback_map_edits(String(objects_msg(self.CS)))
        n._nm_map_publish(grid(0))
        self.assertEqual(n._nm_corridor_pub.msgs, [])
        self.assertEqual(n._nm_corridor_launch_args(), [])

    def test_on_sends_only_on_change(self):
        n = self.node(True)
        self.assertEqual(n._nm_corridor_launch_args(), ['corridor_prefer:=true'])
        out = n._nm_corridor_pub.msgs
        n.callback_map_edits(String(objects_msg(self.CS)))
        self.assertEqual(out, [])                              # no map yet
        m = grid(0)
        n._nm_map_publish(m)
        self.assertEqual(len(out), 1)
        g = out[-1]
        self.assertEqual((g.header.frame_id, g.info.width, g.info.height), ('map', W, H))
        self.assertEqual(g.info.origin.position.x, OX)
        self.assertEqual(sorted(set(g.data)), [0, 4])
        self.assertEqual(g.data.itemsize, 1)                  # a byte a cell in navi_man, not a list
        self.assertEqual(g.data.count(4), int(map_corridor.mask(map_corridor.parse(objects_msg(self.CS))[1], m.info).sum()))
        # the same map again (every 2 s today), the same objects again, a status: nothing
        for _ in range(5):
            n._nm_map_publish(grid(50))
            n.callback_map_edits(String(objects_msg(self.CS + [{'type': 'zone', 'family': 'zone', 'points': []}])))
            n.callback_rules_status(String(json.dumps({'serving': {'site': 'garden'}})))
        self.assertEqual(len(out), 1)
        # another geometry: a new grid that fits it
        n._nm_map_publish(grid(0, w=W + 10))
        self.assertEqual(len(out), 2)
        self.assertEqual(out[-1].info.width, W + 10)
        # the corridor moved
        n.callback_map_edits(String(objects_msg([corridor([(0.0, 1.5), (2.0, 1.5)])])))
        self.assertEqual(len(out), 3)
        # the corridor deleted: an empty grid once = no preference
        n.callback_map_edits(String(objects_msg([])))
        n.callback_map_edits(String(objects_msg([])))
        n._nm_map_publish(grid(0, w=W + 20))
        self.assertEqual(len(out), 4)
        self.assertEqual((out[-1].info.width, len(out[-1].data)), (0, 0))

    def test_no_corridor_around_the_dock(self):
        """Review A680 N2: docking and undocking must plan as without the switch."""
        dock = [{'type': 'waypoint', 'family': 'waypoint', 'name': n, 'points': [list(p)]}
                for n, p in (('UNDOCK', (2.0, 0.9)), ('DOCKED', (0.0, 0.9)), ('DOCK', (1.0, 0.9)), ('TEST', (2.5, 0.0)))]
        cs = [corridor([(-0.5, 0.5), (2.9, 0.5)])]                # runs past the dock line, 0.4 m from it
        n = self.node(True)
        n.params['map_corridor_dock_radius_m'] = 0.5
        n._nm_corridor = None
        out = n._nm_corridor_pub.msgs
        n.callback_map_edits(String(objects_msg(cs + dock)))
        m = grid(0)
        n._nm_map_publish(m)
        g = np.array(out[-1].data, dtype=np.int8).reshape(H, W)
        cor = map_corridor.mask(map_corridor.parse(objects_msg(cs))[1], m.info)
        area = map_corridor.mask((('d', 1.0, ((0.0, 0.9), (1.0, 0.9), (2.0, 0.9))),), m.info)
        self.assertTrue((cor & area).any() and (cor & ~area).any())
        self.assertTrue(((g == 4) == (cor & ~area)).all())        # the corridor is cut out around the dock
        self.assertEqual(sorted(set(g.ravel().tolist())), [0, 4])
        self.assertEqual(map_corridor.parse(objects_msg(dock))[2], ((0.0, 0.9), (1.0, 0.9), (2.0, 0.9)))
        # the dock moved: a new grid; the same again: nothing
        dock[1]['points'] = [[0.0, 1.0]]
        n.callback_map_edits(String(objects_msg(cs + dock)))
        n.callback_map_edits(String(objects_msg(cs + dock)))
        self.assertEqual(len(out), 2)
        # radius 0 = the whole corridor; a corridor wholly at the dock = nothing to prefer
        cc = map_corridor.parse(objects_msg(cs))[1]
        self.assertTrue(((map_corridor.cost_grid(cc, m.info, 4, ((0.0, 0.9),), 0.0) == 4) == cor).all())
        self.assertIsNone(map_corridor.cost_grid(cc, m.info, 4, ((0.0, 0.5), (2.5, 0.5)), 1.5))
        one = map_corridor.cost_grid(cc, m.info, 4, ((1.0, 0.5),), 0.4)          # a single dock point is a disc
        self.assertEqual((int(one[20, 40]), int(one[20, 60])), (0, 4))
        with open(os.path.join(PKG, 'config', 'navi_manager.yaml')) as f:
            self.assertGreaterEqual(yaml.safe_load(f)['map_corridor_dock_radius_m'], 2.0)

    def test_no_corridor_is_one_empty_grid(self):
        n = self.node(True)
        n.callback_map_edits(String(objects_msg([])))
        for w in (W, W + 5, W):
            n._nm_map_publish(grid(0, w=w))
        self.assertEqual([len(m.data) for m in n._nm_corridor_pub.msgs], [0])

    def test_other_site_and_other_frame(self):
        n = self.node(True)
        out = n._nm_corridor_pub.msgs
        n.callback_map_edits(String(objects_msg(self.CS, site='garden')))
        n._nm_map_publish(grid(0))
        self.assertEqual(len(out[-1].data), W * H)
        n.callback_rules_status(String(json.dumps({'serving': {'site': 'cottage'}})))
        self.assertEqual(len(out[-1].data), 0)                 # another site is served
        n.callback_rules_status(String(json.dumps({'serving': {'site': 'garden'}})))
        self.assertEqual(len(out[-1].data), W * H)
        n._nm_map_publish(grid(0, frame='odom'))
        self.assertEqual(len(out[-1].data), 0)

    def test_bad_objects_keep_the_last(self):
        n = self.node(True)
        n.callback_map_edits(String(objects_msg(self.CS)))
        n._nm_map_publish(grid(0))
        n.callback_map_edits(String('{not json'))
        self.assertEqual(len(n._nm_corridor_pub.msgs), 1)

    def test_objects_and_map_before_the_yaml(self):
        n = self.node(True)
        params, n.params = n.params, None                      # navi_manager.yaml not read yet
        n.callback_map_edits(String(objects_msg(self.CS)))
        n._nm_map_publish(grid(0))
        self.assertEqual(n._nm_corridor_pub.msgs, [])
        n.params = params
        n.callback_rules_status(String(json.dumps({'serving': {'site': 'garden'}})))
        self.assertEqual(len(n._nm_corridor_pub.msgs), 1)
        self.assertEqual(len(n._nm_corridor_pub.msgs[-1].data), W * H)


class Library(unittest.TestCase):
    """Review A680 N3: the planner is never asked for a layer it cannot load."""

    def test_missing_and_not_loadable(self):
        import tempfile
        self.assertIsNone(map_corridor.library_path(prefixes=['/nonexistent'], environ={}))
        found = map_corridor.library_path
        try:
            map_corridor.library_path = lambda *a, **k: None
            self.assertIn('not built', map_corridor.library_problem())
        finally:
            map_corridor.library_path = found
        with tempfile.NamedTemporaryFile(suffix='.so') as f:
            f.write(b'not a library')
            f.flush()
            self.assertIn('does not load', map_corridor.library_problem(f.name))
        # a real library that is not the layer (an old build has no version either)
        other = '/opt/ros/noetic/lib/libroscpp.so'
        if os.path.isfile(other):
            self.assertIn('does not load', map_corridor.library_problem(other))

    @unittest.skipUnless(map_corridor.library_path(), 'the layer is not built in this environment')
    def test_built_library(self):
        self.assertIsNone(map_corridor.library_problem())
        version = map_corridor.LIBRARY_VERSION
        try:
            map_corridor.LIBRARY_VERSION = version + 1          # navi_man newer than the build
            self.assertIn('another version', map_corridor.library_problem())
        finally:
            map_corridor.LIBRARY_VERSION = version

    def config(self, planner=None, today=None, drop=False):
        """A copy of the two planner files, changed."""
        import shutil
        import tempfile
        d = tempfile.mkdtemp()
        self.addCleanup(shutil.rmtree, d, True)
        for name, change in (('global_planner_params.yaml', today), ('global_costmap_corridor.yaml', planner)):
            with open(os.path.join(PKG, 'config', name)) as f:
                y = yaml.safe_load(f)
            if name.startswith('global_costmap') and drop:
                del y['GlobalPlanner']
            elif change:
                y['GlobalPlanner'].update(change)
            with open(os.path.join(d, name), 'w') as f:
                yaml.safe_dump(y, f)
        return d

    def test_planner_values_are_checked(self):
        """Review A714: the planner's lower neutral_cost only with a layer whose table is exact."""
        cfg = os.path.join(PKG, 'config')
        self.assertEqual(map_corridor.planner_values(cfg), ((38.0, 1.5, 253), (50.0, 3.0, 253)))
        self.assertIsNone(map_corridor.config_problem(cfg))
        # today's planner changed and the layer was not: off, whatever the library says
        for today in ({'neutral_cost': 60}, {'cost_factor': 2.0}, {'lethal_cost': 250}):
            self.assertIn('is built for', map_corridor.library_problem(config_dir=self.config(today=today)), today)
        self.assertIn('cannot be read', map_corridor.library_problem(config_dir='/nonexistent'))

    @unittest.skipUnless(map_corridor.library_path(), 'the layer is not built in this environment')
    def test_planner_values_are_tried_with_the_library(self):
        self.assertIsNone(map_corridor.library_problem(config_dir=os.path.join(PKG, 'config')))
        for planner in ({'neutral_cost': 37}, {'cost_factor': 1.4}, {'neutral_cost': 40, 'cost_factor': 2.0},
                        {'lethal_cost': 250}):
            self.assertIn("cannot pay exactly today's prices",
                          map_corridor.library_problem(config_dir=self.config(planner=planner)), planner)
        # the planner values missing in the file = today's planner: the layer would change nothing, fine
        self.assertIsNone(map_corridor.library_problem(config_dir=self.config(drop=True)))

    def test_export_is_checked(self):
        self.assertIsNone(map_corridor.export_problem())

    @unittest.skipUnless(map_corridor.library_path(), 'the layer is not built in this environment')
    def test_layer_arithmetic(self):
        """Reviews A680 N1, A691 N4: outside the corridors the planner pays for every cell
        exactly what it pays without the switch (the ceiling as well); a corridor cell is
        cheaper by a fixed amount with the same slope of the inflation; never an obstacle."""
        import ctypes
        lib = ctypes.CDLL(map_corridor.library_path())
        cell, price = lib.vitulus_navi_corridor_layer_cell, lib.vitulus_navi_corridor_layer_price
        cell.argtypes, cell.restype = [ctypes.c_int, ctypes.c_int] + [ctypes.c_double] * 4 + [ctypes.c_int], ctypes.c_int
        err = lib.vitulus_navi_corridor_layer_table_error
        err.argtypes, err.restype = [ctypes.c_double, ctypes.c_double, ctypes.c_int], ctypes.c_double
        price.argtypes, price.restype = [ctypes.c_int, ctypes.c_double, ctypes.c_double, ctypes.c_int], ctypes.c_double
        with open(os.path.join(PKG, 'config', 'global_planner_params.yaml')) as f:
            gp = yaml.safe_load(f)['GlobalPlanner']
        with open(os.path.join(PKG, 'config', 'global_costmap_corridor.yaml')) as f:
            now = yaml.safe_load(f)['GlobalPlanner']
        N0, K0, L = float(gp['neutral_cost']), float(gp['cost_factor']), gp['lethal_cost']
        N, K = float(now['neutral_cost']), float(now['cost_factor'])
        # the planner's own rule (global_planner Expander::getCost)
        self.assertEqual([price(c, N0, K0, L) for c in (0, 10, 67, 68, 251, 252, 255)], [50, 80, 251, 252, 252, 253, 253])
        ref = [price(c, N0, K0, L) for c in range(256)]
        out = [cell(c, 0, N, K, N0, K0, L) for c in range(256)]
        self.assertEqual(out[252:], [252, 253, 254, 255])                         # obstacle, inscribed, unknown stay
        self.assertEqual(out[:4] + out[66:70], [8, 10, 12, 14, 140, 142, 144, 144])
        for c in range(252):
            self.assertTrue(c <= out[c] <= 251, c)                                # only raised, still passable
            self.assertEqual(price(out[c], N, K, L), ref[c], c)                   # the very same price as today
        for v in range(1, 70):
            got = [cell(c, v, N, K, N0, K0, L) for c in range(256)]
            self.assertEqual(got[252:], [252, 253, 254, 255], v)
            for c in range(252):
                self.assertTrue(c <= got[c] <= out[c], (v, c))                    # raised, never over "outside"
                p = price(got[c], N, K, L)
                self.assertLessEqual(p, ref[c], (v, c))
                # cheaper by K * v at most, and never under neutral' + K0 * c (the slope stays)
                self.assertGreaterEqual(p, min(ref[c], max(ref[c] - K * min(v, 60), N + K0 * c)) - 1e-9, (v, c))
            self.assertTrue(all(a <= b for a, b in zip(got[:252], got[1:252])), v)    # the order of cells stays
        seven = [price(cell(c, 7, N, K, N0, K0, L), N, K, L) for c in (0, 10, 60, 67, 68, 100)]
        self.assertEqual(seven, [39.5, 69.5, 219.5, 240.5, 242.0, 252.0])
        # the layer's own check at start (review A714): only a planner that pays exactly today's prices
        self.assertEqual(err(N, K, L), 0.0)
        self.assertEqual(err(N0, K0, L), 0.0)                                     # today's planner: the table is "nothing"
        for bad in ((37.0, 1.5), (38.0, 1.4), (39.0, 1.5), (40.0, 2.0), (45.0, 3.0)):
            self.assertGreater(err(bad[0], bad[1], L), 0.4, bad)
        self.assertGreater(err(N, K, L - 1), 0.4)
        # without the planner's lower neutral cost the layer changes nothing at all
        self.assertEqual([cell(c, v, N0, K0, N0, K0, L) for v in (0, 7) for c in range(256)], list(range(256)) * 2)


def launch_params(args):
    cfg = roslaunch.config.load_config_default([(LAUNCH, ['map_topic:=/navi_manager/map'] + args)], None, verbose=False)
    return {k: p.value for k, p in cfg.params.items()}


class LaunchSwitch(unittest.TestCase):
    NS = '/move_base_flex/global_costmap/'

    def test_off_is_unchanged(self):
        base = launch_params([])
        with open(os.path.join(PKG, 'config', 'global_costmap_params.yaml')) as f:
            self.assertEqual(base[self.NS + 'plugins'], yaml.safe_load(f)['global_costmap']['plugins'])
        self.assertFalse(any('corridor' in k for k in base))
        for v in ('false', 'True ', '1', 'yes', ''):
            p = launch_params(['corridor_prefer:=' + v]) if v else base
            if v.strip().lower() == 'true':
                continue
            self.assertEqual(p, base, v)

    def test_on_adds_the_layer_last_and_nothing_else(self):
        base = launch_params([])
        on = launch_params(['corridor_prefer:=true'])
        self.assertEqual(on[self.NS + 'plugins'], base[self.NS + 'plugins'] +
                         [{'name': 'corridor_layer', 'type': 'vitulus_navi::CorridorLayer'}])
        self.assertEqual(on[self.NS + 'plugins'][-2]['name'], 'inflation_layer')
        self.assertEqual(on[self.NS + 'corridor_layer/topic'], map_corridor.TOPIC)
        gp = '/move_base_flex/GlobalPlanner/'
        changed = {self.NS + 'plugins', gp + 'neutral_cost', gp + 'cost_factor'}
        added = {k for k in on if k not in base}
        self.assertEqual(added, {self.NS + 'corridor_layer/topic'})
        self.assertEqual({k for k in base if on[k] != base[k]}, changed)
        self.assertEqual((base[gp + 'neutral_cost'], base[gp + 'cost_factor'], base[gp + 'lethal_cost']), map_corridor.TODAY)
        self.assertEqual((on[gp + 'neutral_cost'], on[gp + 'cost_factor']), (38, 1.5))
        # the local costmap does not know the corridors
        self.assertFalse(any('corridor' in json.dumps(v) for k, v in on.items() if '/local_costmap/' in k))

    def test_plugin_is_exported(self):
        with open(os.path.join(PKG, 'corridor_layer_plugin.xml')) as f:
            xml = f.read()
        self.assertIn('vitulus_navi::CorridorLayer', xml)
        self.assertIn(map_corridor.LIBRARY[:-3], xml)
        with open(os.path.join(PKG, 'package.xml')) as f:
            self.assertIn('<costmap_2d plugin="${prefix}/corridor_layer_plugin.xml"/>', f.read())
        with open(os.path.join(PKG, 'plugins', 'corridor_layer.cpp')) as f:
            self.assertIn('MAX_V = %d;' % map_corridor.COST_MAX, f.read())

    def test_yaml_default_off(self):
        with open(os.path.join(PKG, 'config', 'navi_manager.yaml')) as f:
            y = yaml.safe_load(f)
        self.assertIs(y['map_corridor_prefer'], False)


if __name__ == '__main__':
    unittest.main(verbosity=1)
