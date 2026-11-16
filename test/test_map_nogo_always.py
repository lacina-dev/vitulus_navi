#!/usr/bin/env python3
"""The nogo areas are in /navi_manager/map whatever the map source (T36 Z1), offline.

    python3 test/test_map_nogo_always.py      (sourced workspace)

mapping_manager stamps the owner's nogo areas into /mapping_manager/site_map
only. navi_man serves that raster while its source is "octomap"; every start of
the navi launch at run time (save / edit of a map) and /navi_manager/show_map
switch to "planner" / "rtabmap" for good, and those grids carry a nogo only by
chance. With map_nogo_always (default true) navi_man stamps the active nogo
areas of /map_objects/rules into every grid it sends that is not the site
raster.

A small world drives the real navi_man callbacks and publish_map:
  mapping   the site raster WITH the nogo stamped (as mapping_manager does)
  planner   its grid WITHOUT the nogo (a layer view / not refreshed yet)
  rules     /map_objects/rules + /mapping_manager/status
  costmap   reads the latch of /navi_manager/map
"""
import importlib.machinery
import importlib.util
import itertools
import json
import os
import random
import sys
import types
import unittest

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:1'     # never the robot's master

import numpy as np  # noqa: E402
import rospy  # noqa: E402
import yaml  # noqa: E402
from nav_msgs.msg import OccupancyGrid  # noqa: E402
from std_msgs.msg import String  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'src')
try:
    import docker  # noqa: F401
except ImportError:
    sys.modules['docker'] = types.ModuleType('docker')

# This tree's packages, then devel again: vitulus_navi.srv is generated there.
sys.path.insert(0, SRC)
from navi_man import map_dedup, map_nogo, path_rules  # noqa: E402
sys.path.remove(SRC)

try:
    from vitulus_mapping import editmask
except ImportError:
    editmask = None


def load():
    path = os.path.join(HERE, '..', 'nodes', 'navi_man')
    loader = importlib.machinery.SourceFileLoader('navi_man_under_test', path)
    spec = importlib.util.spec_from_loader('navi_man_under_test', loader)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


W, H, RES, OX, OY = 60, 40, 0.05, -1.0, -0.5        # 3 x 2 m


def grid(fill, w=W, h=H, frame='map', ox=OX, oy=OY, res=RES):
    m = OccupancyGrid()
    m.header.frame_id = frame
    m.info.width, m.info.height, m.info.resolution = w, h, res
    m.info.origin.position.x, m.info.origin.position.y = ox, oy
    m.info.origin.orientation.w = 1.0
    m.data = tuple([fill] * (w * h))                # int8[] arrives as a tuple
    return m


ZONE_A = [(-0.5, 0.0), (0.3, 0.0), (0.3, 0.6), (-0.5, 0.6)]
ZONE_B = [(0.8, 0.2), (1.6, 0.2), (1.2, 1.1)]
ZONE_OUT = [(50.0, 50.0), (51.0, 50.0), (51.0, 51.0)]   # not on this grid


def rules_msg(zones, site='garden', extra=()):
    rules = [{'id': 'r%d' % i, 'type': 'nogo', 'name': 'zone%d' % i,
              'points': [list(p) for p in z]} for i, z in enumerate(zones)]
    return String(json.dumps({'site': site, 'seq': 1, 'rules': rules + list(extra)}))


def status_msg(site):
    return String(json.dumps({'serving': {'site': site, 'raster': 'combined'} if site else None}))


def zone_cells(zones, msg=None):
    return map_nogo.cells([('z', z) for z in zones], (msg or grid(0)).info)


def stamped(fill, zones):
    """What mapping_manager serves: the raster with the nogo areas as obstacles."""
    m = grid(fill)
    data = list(m.data)
    for i in zone_cells(zones):
        data[i] = 100
    m.data = tuple(data)
    return m


class Sink(object):
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class Running(object):
    """RTABMAP_MAPPING as a container that is up already."""
    cont_name = 'RTABMAP_MAPPING'
    status = 'running'

    def __init__(self):
        self.client = self
        self.containers = self

    def get(self, _name):
        return self


SITE, PLAN, RTAB, BOOT = 10, 50, 70, -1
# (map_nogo_always, map_publish_on_change, map_source_republish, map_edited_on_change)
SAVING = [c for c in itertools.product((False, True), repeat=3)]


class World(object):

    def __init__(self, M, on=True, saving=(False, False, False), fallback='bootstrap'):
        self.M = M
        self.pub_on_change, self.republish, self.edited_on_change = saving
        self.zones = []
        self.site = None
        self.plan = None
        self.plan_sent = None
        n = M.Node.__new__(M.Node)
        n.params = {'map_publish_on_change': self.pub_on_change,
                    'map_source_republish': self.republish}
        if on is not None:
            n.params['map_nogo_always'] = on
        n.nm_map_pub = Sink()
        n.pub_map_source = Sink()
        n.is_rtabmap_pub = Sink()
        n.pub_serve_site = Sink()
        n.rule_set = path_rules.RuleSet()
        n.map_show = 'planner'
        n.is_map_to_publish = True
        n.rtabmap_grid_map_msg = None
        n.planner_map_msg = None
        n.site_map_msg = None
        n._octomap_serve_deadline = None
        n._octomap_last_serve_req = None
        n._octomap_serve_window = 60.0
        n._octomap_fallback_active = False
        n._octomap_fallback_mode = fallback
        n._bootstrap_map_msg = None
        n._bootstrap_build_attempts = 0
        n._build_bootstrap_map = lambda: grid(BOOT)
        n.rtabmap_mapping = Running()                     # show_map "rtabmap" starts nothing
        n._nm_map_dedup = map_dedup.MapDedup()
        n._nm_map_on_change = None
        n._nm_map_source_republish = None
        self.n = n
        n.callback_rules_status(status_msg('garden'))

    # --- the owner edits the nogo areas ------------------------------------
    def set_zones(self, zones, mapping=True, site='garden'):
        """map_objects_server sends the rules; mapping_manager re-serves the
        site raster with them (mapping=False: it has not yet / does not)."""
        self.zones = list(zones)
        self.n.callback_map_rules(rules_msg(zones, site))
        if mapping and self.site is not None:
            self.serve_site()

    def serve_site(self):
        self.site = stamped(SITE, self.zones)
        self.n.callback_site_map(self.site)

    # --- node_planner: every 2 s, or only on change -------------------------
    def planner_edit(self, fill=PLAN):
        self.plan = grid(fill)

    def planner_2s(self):
        if self.plan is None:
            return
        if self.edited_on_change and self.plan_sent is not None \
                and self.plan_sent.data == self.plan.data:
            return
        self.plan_sent = self.plan
        # every message is a new object on the wire
        m = grid(0)
        m.data = self.plan.data
        self.n.callback_web_plan_map(m)

    def tick(self, k=1):
        for _ in range(k):
            self.n.publish_map()

    def idle(self, rounds=2):
        for _ in range(rounds):
            for _ in range(3):
                self.planner_2s()
            self.tick()

    def boot(self, source='octomap', site=True):
        if site:
            self.serve_site()
        self.planner_edit()
        self.planner_2s()
        self.n._nm_map_source_set('planner')              # start_navi_launch
        self.n.callback_web_plan_show_map(String(source))
        self.idle()
        return self

    # --- what a costmap subscribing NOW gets -------------------------------
    def latch(self):
        return self.n.nm_map_pub.msgs[-1]

    def base(self):
        """The fill of the latched grid outside every nogo."""
        return self.latch().data[0]

    def blocked(self, zones=None):
        data = self.latch().data
        return all(data[i] == 100 for i in zone_cells(self.zones if zones is None else zones))

    def free(self, zones):
        data = self.latch().data
        return all(data[i] == self.base() for i in zone_cells(zones))


class Base(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.rostime.set_rostime_initialized(True)
        cls.M = load()

    def each(self):
        for saving in SAVING:
            with self.subTest(pub_on_change=saving[0], source_republish=saving[1],
                              edited_on_change=saving[2]):
                yield saving

    def switched(self, saving):
        """Does a source change inside navi_man reach the latch at all? Not
        with map_edited_on_change and without map_source_republish (T39 C4:
        that pair must not be used; the latch then keeps the site raster)."""
        return saving[1] or not saving[2]


class StamperTest(unittest.TestCase):
    """map_nogo alone."""

    def test_the_same_cells_as_mapping_manager(self):
        if editmask is None:
            self.skipTest('vitulus_mapping not on the path')
        rnd = random.Random(36)
        for _ in range(200):
            w, h = rnd.randint(5, 90), rnd.randint(5, 90)
            res = rnd.choice((0.05, 0.1, 0.025))
            ox, oy = rnd.uniform(-5, 5), rnd.uniform(-5, 5)
            zones = []
            for _z in range(rnd.randint(1, 3)):
                zones.append([(ox + rnd.uniform(-0.5, w * res + 0.5),
                               oy + rnd.uniform(-0.5, h * res + 0.5))
                              for _p in range(rnd.randint(3, 7))])
            m = grid(0, w=w, h=h, ox=ox, oy=oy, res=res)
            theirs = np.zeros((w, h), dtype=np.int8)        # editmask axes [ix, iy]
            n = editmask.rasterize_edits(
                theirs, [{'kind': 'obstacle', 'vertices': z} for z in zones], res, (ox, oy))
            self.assertEqual(n, len(zones))
            out = map_nogo.Nogo().apply(m, [('z', z) for z in zones])
            ours = np.asarray(out.data, dtype=np.int8).reshape(h, w)   # [iy, ix]
            self.assertTrue(np.array_equal(ours, theirs.T))

    def test_nothing_to_add_is_the_same_object(self):
        s = map_nogo.Nogo()
        m = grid(PLAN)
        self.assertIs(s.apply(m, []), m)
        self.assertIs(s.apply(m, [('out', ZONE_OUT)]), m)
        self.assertIs(s.apply(m, [('line', ZONE_A[:2])]), m)
        done = stamped(PLAN, [ZONE_A])
        self.assertIs(s.apply(done, [('a', ZONE_A)]), done)
        self.assertEqual(s.added, 0)
        self.assertIs(s.apply(None, [('a', ZONE_A)]), None)

    def test_the_source_grid_is_never_modified(self):
        s = map_nogo.Nogo()
        m = grid(PLAN)
        before = m.data
        out = s.apply(m, [('a', ZONE_A)])
        self.assertIsNot(out, m)
        self.assertIs(m.data, before)
        self.assertEqual(set(m.data), {PLAN})
        self.assertEqual(s.added, len(zone_cells([ZONE_A])))
        self.assertGreater(s.added, 100)
        self.assertEqual(sorted(i for i, v in enumerate(out.data) if v == 100),
                         zone_cells([ZONE_A]))
        self.assertIs(out.info, m.info)
        self.assertIs(s.apply(m, [('a', ZONE_A)]), out)     # the same grid again: cached
        self.assertEqual(s.added, len(zone_cells([ZONE_A])))

    def test_unknown_and_free_cells_become_obstacles(self):
        for fill in (-1, 0, 99):
            out = map_nogo.Nogo().apply(grid(fill), [('a', ZONE_A)])
            self.assertEqual({out.data[i] for i in zone_cells([ZONE_A])}, {100})

    def test_partly_outside_the_grid(self):
        zone = [(OX - 1.0, OY - 1.0), (OX + 0.5, OY - 1.0), (OX + 0.5, OY + 0.5), (OX - 1.0, OY + 0.5)]
        out = map_nogo.Nogo().apply(grid(0), [('edge', zone)])
        got = np.asarray(out.data).reshape(H, W)
        self.assertTrue((got[:11, :11] == 100).all())
        self.assertEqual(int((got == 100).sum()), 121)

    def test_a_grid_that_cannot_be_stamped_goes_out_as_it_came(self):
        s = map_nogo.Nogo()
        for m in (grid(0, frame='odom'), grid(0, res=0.0), grid(0, w=0, h=0)):
            self.assertIs(s.apply(m, [('a', ZONE_A)]), m)
        self.assertIsNone(s.error)                          # the empty grid is no error
        self.assertIs(s.apply(grid(0, frame='odom'), [('a', ZONE_A)]).__class__, OccupancyGrid)
        self.assertIn('frame', s.error)
        short = grid(0)
        short.data = short.data[:-1]
        self.assertIs(s.apply(short, [('a', ZONE_A)]), short)
        self.assertEqual(s.error, 'bad grid')
        self.assertIs(s.apply(grid(0), [('a', [(0, 0), (1, 'x'), (1, 1)])]).__class__, OccupancyGrid)
        self.assertIsNotNone(s.error)                       # raised inside, did not get out

    def test_a_turned_grid(self):
        m = grid(0)
        m.info.origin.orientation.z = m.info.origin.orientation.w = 0.5 ** 0.5   # +90 deg
        m.info.origin.position.x, m.info.origin.position.y = 0.0, 0.0
        # grid x runs along map +y, grid y along map -x
        zone = [(-0.5, 0.2), (-0.1, 0.2), (-0.1, 1.0), (-0.5, 1.0)]
        out = map_nogo.Nogo().apply(m, [('t', zone)])
        got = np.asarray(out.data).reshape(H, W)
        self.assertTrue((got[3:10, 5:20] == 100).all())
        self.assertFalse((got[12:, :] == 100).any())
        self.assertFalse((got[:, 22:] == 100).any())

    def test_new_rules_or_another_geometry_are_computed_again(self):
        s = map_nogo.Nogo()
        m = grid(0)
        a = s.apply(m, [('a', ZONE_A)])
        b = s.apply(m, [('b', ZONE_B)])
        self.assertEqual(sorted(i for i, v in enumerate(b.data) if v == 100), zone_cells([ZONE_B]))
        self.assertEqual(sorted(i for i, v in enumerate(a.data) if v == 100), zone_cells([ZONE_A]))
        moved = grid(0, ox=OX + 0.5)
        c = s.apply(moved, [('b', ZONE_B)])
        self.assertEqual(sorted(i for i, v in enumerate(c.data) if v == 100),
                         zone_cells([ZONE_B], moved))
        self.assertNotEqual(zone_cells([ZONE_B], moved), zone_cells([ZONE_B]))


class EverySourceTest(Base):
    """Switch on (default): the latch has the nogo whatever the source and
    whatever the saving switches."""

    def test_start_navi_launch_at_run_time(self):
        """Save / edit of a map: the source goes to "planner" and stays."""
        for saving in self.each():
            w = World(self.M, saving=saving)
            w.set_zones([ZONE_A])
            w.boot()
            self.assertEqual(w.base(), SITE)
            self.assertTrue(w.blocked())
            w.n._nm_map_source_set('planner')
            for _ in range(4):
                w.idle(1)
                self.assertTrue(w.blocked())            # at no tick an open nogo
            self.assertEqual(w.base(), PLAN if self.switched(saving) else SITE)
            if self.switched(saving):
                self.assertEqual(w.n.pub_map_source.msgs[-1].data, 'planner')

    def test_show_map_to_planner_and_rtabmap(self):
        for saving in self.each():
            w = World(self.M, saving=saving)
            w.set_zones([ZONE_A, ZONE_B])
            w.boot()
            w.n.callback_web_plan_show_map(String('planner'))
            w.idle()
            self.assertEqual(w.base(), PLAN)
            self.assertTrue(w.blocked())
            w.n.callback_rtabmap_grid_map(grid(RTAB))
            w.n.callback_web_plan_show_map(String('rtabmap'))
            w.idle()
            self.assertEqual(w.base(), RTAB)
            self.assertTrue(w.blocked())
            w.n.callback_web_plan_show_map(String('octomap'))
            w.idle()
            self.assertIs(w.latch(), w.site)

    def test_new_map_rtabmap(self):
        """callback_new_map (legacy maps only): the rtabmap grid, when it comes."""
        for saving in self.each():
            w = World(self.M, saving=saving)
            w.set_zones([ZONE_A])
            w.boot()
            w.n._nm_map_source_set('rtabmap')
            w.idle()
            self.assertTrue(w.blocked())
            w.n.callback_rtabmap_grid_map(grid(RTAB))
            w.tick()
            self.assertEqual(w.base(), RTAB)
            self.assertTrue(w.blocked())

    def test_boot_with_the_planner_source(self):
        for saving in self.each():
            w = World(self.M, saving=saving)
            w.set_zones([ZONE_A])
            w.boot(source='planner')
            self.assertEqual(w.base(), PLAN)
            self.assertTrue(w.blocked())

    def test_octomap_without_a_raster(self):
        """Bootstrap map / the planner stand-in while no site raster is here."""
        for saving in self.each():
            for fallback, fill in (('bootstrap', BOOT), ('planner', PLAN)):
                w = World(self.M, saving=saving, fallback=fallback)
                w.set_zones([ZONE_A])
                w.boot(site=False)
                self.assertEqual(w.base(), fill)
                self.assertTrue(w.blocked())
                w.serve_site()                          # the raster arrives
                w.tick()
                self.assertIs(w.latch(), w.site)
                w.n.callback_site_map(grid(0, w=0, h=0))    # a fresh site is being mapped
                w.idle()
                self.assertEqual(w.base(), fill)
                self.assertTrue(w.blocked())

    def test_planner_layer_view_during_a_mission(self):
        """node_planner shows another layer (SMACH|ZONE, free polygons...)."""
        for saving in self.each():
            w = World(self.M, saving=saving)
            w.set_zones([ZONE_A])
            w.boot(source='planner')
            for fill in (0, 100, -1, PLAN):
                w.planner_edit(fill)
                w.idle(1)
                self.assertEqual(w.base(), fill)
                self.assertTrue(w.blocked())


class ZoneEditTest(Base):
    """The owner edits a nogo while the source is "planner": the planner's grid
    does not follow (debounce, a mission, a layer view), the latch must."""

    def test_add_change_remove(self):
        for saving in self.each():
            w = World(self.M, saving=saving)
            w.boot(source='planner')
            self.assertEqual(set(w.latch().data), {PLAN})
            sent = len(w.n.nm_map_pub.msgs)
            w.set_zones([ZONE_A])
            w.tick()                                    # the very next tick, no map_edited needed
            self.assertEqual(len(w.n.nm_map_pub.msgs), sent + 1)
            self.assertTrue(w.blocked([ZONE_A]))
            self.assertTrue(w.free([ZONE_B]))
            w.idle()
            self.assertTrue(w.blocked([ZONE_A]))
            w.set_zones([ZONE_B])                       # moved
            w.tick()
            self.assertTrue(w.blocked([ZONE_B]))
            self.assertTrue(w.free([ZONE_A]))
            w.set_zones([ZONE_A, ZONE_B])
            w.tick()
            self.assertTrue(w.blocked([ZONE_A, ZONE_B]))
            w.idle(3)
            self.assertTrue(w.blocked([ZONE_A, ZONE_B]))
            w.set_zones([])                             # removed: the planner's own grid again
            w.tick()
            self.assertEqual(set(w.latch().data), {PLAN})
            self.assertIs(w.latch(), w.n.planner_map_msg)

    def test_edit_before_the_mapping_manager_follows(self):
        """Source "octomap": the raster is mapping_manager's business, as before."""
        for saving in self.each():
            w = World(self.M, saving=saving)
            w.boot()
            sent = len(w.n.nm_map_pub.msgs)
            w.set_zones([ZONE_A], mapping=False)
            w.tick()
            self.assertEqual(len(w.n.nm_map_pub.msgs), sent)    # nothing asked for
            w.serve_site()
            w.tick()
            self.assertIs(w.latch(), w.site)
            self.assertTrue(w.blocked())

    def test_rules_of_another_site_do_not_apply(self):
        for saving in self.each():
            w = World(self.M, saving=saving)
            w.boot(source='planner')
            w.set_zones([ZONE_A], site='elsewhere')
            w.idle()
            self.assertEqual(set(w.latch().data), {PLAN})
            w.n.callback_rules_status(status_msg('elsewhere'))  # ... until that site is served
            w.tick()
            self.assertTrue(w.blocked([ZONE_A]))
            w.n.callback_rules_status(status_msg('garden'))
            w.tick()
            self.assertEqual(set(w.latch().data), {PLAN})

    def test_a_bad_rules_message_keeps_the_last_good_ones(self):
        w = World(self.M)
        w.set_zones([ZONE_A])
        w.boot(source='planner')
        w.n.callback_map_rules(String('{not json'))
        w.n.callback_map_rules(String(json.dumps({'site': None, 'rules': []})))
        w.idle()
        self.assertTrue(w.blocked([ZONE_A]))

    def test_only_nogo_areas_are_stamped(self):
        w = World(self.M)
        w.boot(source='planner')
        extra = [{'id': 's', 'type': 'slow', 'name': 's', 'points': [list(p) for p in ZONE_B]},
                 {'id': 'n', 'type': 'noblade', 'name': 'n', 'points': [list(p) for p in ZONE_B]}]
        w.n.callback_map_rules(String(json.dumps(
            {'site': 'garden', 'rules': json.loads(rules_msg([ZONE_A]).data)['rules'] + extra})))
        w.idle()
        self.assertTrue(w.blocked([ZONE_A]))
        self.assertTrue(w.free([ZONE_B]))

    def test_the_status_at_1_hz_asks_for_nothing(self):
        for saving in self.each():
            w = World(self.M, saving=saving)
            w.set_zones([ZONE_A])
            w.boot(source='planner')
            w.tick()
            self.assertFalse(w.n.is_map_to_publish)
            for _ in range(5):
                w.n.callback_rules_status(status_msg('garden'))
                w.n.callback_map_rules(rules_msg([ZONE_A]))     # the same rules again
            self.assertFalse(w.n.is_map_to_publish)

    def test_saving_is_kept(self):
        """map_publish_on_change: a standing robot sends the stamped grid once."""
        for republish in (False, True):
            w = World(self.M, saving=(True, republish, False))
            w.set_zones([ZONE_A])
            w.boot(source='planner')
            sent = len(w.n.nm_map_pub.msgs)
            w.idle(20)
            self.assertEqual(len(w.n.nm_map_pub.msgs), sent)
            self.assertTrue(w.blocked())


class NothingChangesTest(Base):
    """Source "octomap" with a raster: on and off send the very same messages."""

    def script(self, w):
        w.set_zones([ZONE_A])
        w.boot()
        w.set_zones([ZONE_A, ZONE_B])
        w.idle()
        w.planner_edit(0)
        w.idle()
        w.set_zones([])
        w.idle()
        return w

    def test_octomap_serves_the_site_raster_object_itself(self):
        for saving in self.each():
            seq = {}
            for on in (False, True):
                w = self.script(World(self.M, on=on, saving=saving))
                first = next(k for k, m in enumerate(w.n.nm_map_pub.msgs) if m.data[0] == SITE)
                for m in w.n.nm_map_pub.msgs[first:]:
                    self.assertEqual(m.data[0], SITE)
                self.assertIs(w.latch(), w.n.site_map_msg)
                stamper = getattr(w.n, '_nm_nogo_stamper', None)
                if stamper is not None and stamper._src is not None:
                    self.assertNotEqual(stamper._src.data[0], SITE)     # never saw the raster
                seq[on] = [(m.data, m.info.width, m.info.height)
                           for m in w.n.nm_map_pub.msgs[first:]]
                seq[on, 'src'] = [m.data for m in w.n.pub_map_source.msgs]
            self.assertEqual(seq[True], seq[False])
            self.assertEqual(seq[True, 'src'], seq[False, 'src'])

    def test_no_nogo_no_difference_with_any_source(self):
        """No rules (the robot today): the grids go out as the objects they came."""
        for saving in self.each():
            for source in ('planner', 'rtabmap', 'octomap'):
                counts = {}
                for on in (False, True):
                    w = World(self.M, on=on, saving=saving)
                    w.n.callback_rtabmap_grid_map(grid(RTAB))
                    w.boot(source=source)
                    w.n._nm_map_source_set('planner')
                    w.idle()
                    stamper = getattr(w.n, '_nm_nogo_stamper', None)
                    self.assertTrue(stamper is None or stamper._out is None)   # no copy made
                    self.assertEqual({len(set(m.data)) for m in w.n.nm_map_pub.msgs}, {1})
                    counts[on] = [m.data[0] for m in w.n.nm_map_pub.msgs]
                self.assertEqual(counts[True], counts[False])

    def test_a_planner_grid_that_has_the_nogo_goes_out_untouched(self):
        """The usual planner grid is made from the site raster: same object out."""
        w = World(self.M)
        w.set_zones([ZONE_A])
        w.boot(source='planner')
        w.plan = stamped(PLAN, [ZONE_A])
        w.idle()
        self.assertIs(w.latch(), w.n.planner_map_msg)


class SwitchOffTest(Base):
    """map_nogo_always false = as before: the hole, kept reachable for a rollback."""

    def test_off_the_planner_grid_goes_out_as_it_is(self):
        for saving in self.each():
            if not self.switched(saving):
                continue
            w = World(self.M, on=False, saving=saving)
            w.set_zones([ZONE_A])
            w.boot()
            self.assertTrue(w.blocked())
            w.n._nm_map_source_set('planner')
            w.idle()
            self.assertEqual(set(w.latch().data), {PLAN})       # the hole
            self.assertFalse(w.blocked())
            on = World(self.M, on=True, saving=saving)
            on.set_zones([ZONE_A])
            on.boot()
            on.n._nm_map_source_set('planner')
            on.idle()
            self.assertTrue(on.blocked())

    def test_off_a_rules_change_asks_for_nothing(self):
        w = World(self.M, on=False)
        w.boot(source='planner')
        w.tick()
        self.assertFalse(w.n.is_map_to_publish)
        w.set_zones([ZONE_A])
        self.assertFalse(w.n.is_map_to_publish)

    def test_default_is_on(self):
        w = World(self.M, on=None)                              # key missing in the yaml
        w.set_zones([ZONE_A])
        w.boot(source='planner')
        self.assertTrue(w.blocked())
        with open(os.path.join(HERE, '..', 'config', 'navi_manager.yaml')) as f:
            self.assertIs(yaml.safe_load(f).get('map_nogo_always'), True)

    def test_a_failing_stamp_never_stops_the_map(self):
        w = World(self.M)
        w.set_zones([ZONE_A])
        w.boot(source='planner')

        class Broken(object):
            added, error = 0, 'boom'

            def apply(self, msg, nogos):
                return msg
        w.n._nm_nogo_stamper = Broken()
        w.n._nm_nogo_said = None
        w.planner_edit(0)
        w.idle()
        self.assertEqual(w.base(), 0)
        self.assertIs(w.latch(), w.n.planner_map_msg)


class WiringTest(unittest.TestCase):
    """The source of navi_man: nothing but the site raster goes out unstamped."""

    @classmethod
    def setUpClass(cls):
        with open(os.path.join(HERE, '..', 'nodes', 'navi_man')) as f:
            cls.src = f.read()

    def test_every_publish_but_the_site_raster_is_stamped(self):
        body = self.src.split('    def publish_map(self):')[1].split('\n    def ')[0]
        calls = [l.strip() for l in body.splitlines() if '_nm_map_publish(' in l]
        self.assertEqual(sorted(calls), sorted([
            'self._nm_map_publish(self._nm_nogo(self.planner_map_msg))',        # planner
            'self._nm_map_publish(self._nm_nogo(self.rtabmap_grid_map_msg))',   # rtabmap
            'self._nm_map_publish(self.site_map_msg)',                          # octomap
            'self._nm_map_publish(self._nm_nogo(self._bootstrap_map_msg))',     # no raster
            'self._nm_map_publish(self._nm_nogo(self.planner_map_msg))']))      # no raster
        self.assertEqual(self.src.count('self.nm_map_pub.publish('), 1)

    def test_start_navi_launch_switches_through_the_one_place(self):
        body = self.src.split('    def start_navi_launch(')[1].split('\n    def ')[0]
        self.assertIn('self._nm_map_source_set("planner")', body)

    def test_both_rule_inputs_tell_the_map(self):
        for name in ('callback_map_rules', 'callback_rules_status'):
            body = self.src.split('    def %s(' % name)[1].split('\n    def ')[0]
            self.assertIn('self._nm_nogo_changed()', body)
        self.assertIn('self.callback_rules_status, queue_size=1)', self.src)


if __name__ == '__main__':
    unittest.main()
