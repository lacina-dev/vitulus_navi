#!/usr/bin/env python3
"""The served map does not depend on map_edited being repeated (T39 C4), offline.

    python3 test/test_map_source_republish.py      (sourced workspace)

/navi_manager/map is what the global costmap of move_base_flex (static layer)
and the local costmap plan on; the keep-out areas of the editor are stamped
into it (mapping_manager -> /mapping_manager/site_map -> navi_man). navi_man
changes the map source by itself in two places without asking for a publish;
until now the next /web_plan/map_edited of node_planner (every 2 s) did. With
node_planner ~map_edited_on_change that message does not come.

A small world drives the real navi_man callbacks and publish_map:
  planner   every 2 s, or only on change + latched (map_edited_on_change)
  mapping   latched site raster (keep-out stamped)
  costmap   reads the latch of /navi_manager/map (a restarted move_base_flex
            and a static layer after clear_costmaps do exactly that)
and for every combination of the switches and every event checks that the
latch holds the CURRENT grid of the selected source.
"""
import importlib.machinery
import importlib.util
import itertools
import os
import sys
import types
import unittest

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:1'     # never the robot's master

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

sys.path.insert(0, SRC)
from navi_man import map_dedup  # noqa: E402
sys.path.remove(SRC)


def load():
    path = os.path.join(HERE, '..', 'nodes', 'navi_man')
    loader = importlib.machinery.SourceFileLoader('navi_man_under_test', path)
    spec = importlib.util.spec_from_loader('navi_man_under_test', loader)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def grid(fill, w=4, h=3):
    m = OccupancyGrid()
    m.header.frame_id = 'map'
    m.info.width, m.info.height, m.info.resolution = w, h, 0.05
    m.info.origin.orientation.w = 1.0
    m.data = tuple([fill] * (w * h))
    return m


class Sink(object):
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


# switches: (map_publish_on_change, map_edited_on_change)
OFF, PUB, BOTH = (False, False), (True, False), (True, True)
COMBOS = (OFF, PUB, BOTH)

SITE, SITE_ZONE = 10, 11          # site raster without / with a keep-out stamped
PLAN, PLAN_ZONE = 50, 51          # the planner's grid before / after an edit
RTAB = 70


class World(object):
    """node_planner + mapping_manager + navi_man + a costmap on the latch."""

    def __init__(self, M, combo, fix):
        self.M, self.fix = M, fix
        self.pub_on_change, self.edited_on_change = combo
        self.site = None              # latch of /mapping_manager/site_map
        self.plan = None              # the planner's current grid
        self.plan_latch = None        # latch of /web_plan/map_edited (on change only)
        self.plan_sent = None
        self.n = None

    # --- navi_man ----------------------------------------------------------
    def navi_man_start(self, source_on_start='octomap', launch=True):
        """A fresh process: empty latch, the latched topics arrive on connect."""
        n = self.M.Node.__new__(self.M.Node)
        n.params = {'map_publish_on_change': self.pub_on_change,
                    'map_source_republish': self.fix}
        n.nm_map_pub = Sink()
        n.pub_map_source = Sink()
        n.is_rtabmap_pub = Sink()
        n.pub_serve_site = Sink()
        n.map_show = 'planner'
        n.is_map_to_publish = True
        n.rtabmap_grid_map_msg = None
        n.planner_map_msg = None
        n.site_map_msg = None
        n._octomap_serve_deadline = None
        n._octomap_last_serve_req = None
        n._octomap_serve_window = 60.0
        n._octomap_fallback_active = False
        n._octomap_fallback_mode = 'planner'
        n._bootstrap_map_msg = None
        n._bootstrap_build_attempts = 0
        n._nm_map_dedup = map_dedup.MapDedup()
        n._nm_map_on_change = None
        n._nm_map_source_republish = None
        self.n = n
        if self.site is not None:
            n.callback_site_map(self.site)
        if self.plan_latch is not None:
            n.callback_web_plan_map(self.plan_latch)
        if launch:
            n._nm_map_source_set('planner')           # start_navi_launch
        if source_on_start:
            n.callback_web_plan_show_map(String(source_on_start))
        return n

    def tick(self, k=1):
        for _ in range(k):
            self.n.publish_map()

    # --- node_planner ------------------------------------------------------
    def planner_2s(self):
        """map_edited_publisher: every 2 s, or only when the grid changed."""
        if self.plan is None:
            return
        if self.edited_on_change:
            if self.plan_sent is not None and self.plan_sent.data == self.plan.data:
                return
            self.plan_latch = self.plan
        self.plan_sent = self.plan
        if self.n is not None:
            self.n.callback_web_plan_map(self.plan)

    def planner_edit(self, fill):
        self.plan = grid(fill)

    def planner_restart(self):
        """Its latch is gone; it sends its grid again once it has it."""
        self.plan_sent = None
        self.plan_latch = None
        self.planner_2s()

    # --- mapping_manager ---------------------------------------------------
    def serve_site(self, fill):
        self.site = grid(fill)
        if self.n is not None:
            self.n.callback_site_map(self.site)

    # --- time: 10 s of standing (planner 5x, slow loop 2x) ------------------
    def idle(self, rounds=2):
        for _ in range(rounds):
            for _ in range(3):
                self.planner_2s()
            self.tick()

    # --- what a costmap subscribing NOW gets -------------------------------
    def latch(self):
        msgs = self.n.nm_map_pub.msgs
        return msgs[-1].data[0] if msgs else None


class Base(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.rostime.set_rostime_initialized(True)
        cls.M = load()

    def world(self, combo, fix=True, source='octomap'):
        w = World(self.M, combo, fix)
        w.serve_site(SITE)
        w.planner_edit(PLAN)
        w.planner_2s()
        w.navi_man_start(source)
        w.idle()
        return w


class EventsTest(Base):
    """Fix on: every combination of the saving switches x every event."""

    def each(self):
        for combo in COMBOS:
            with self.subTest(pub_on_change=combo[0], edited_on_change=combo[1]):
                yield combo

    def test_zone_edit_reaches_the_costmap(self):
        for combo in self.each():
            w = self.world(combo)
            self.assertEqual(w.latch(), SITE)
            w.serve_site(SITE_ZONE)            # mapping_manager stamped the keep-out
            w.tick()
            self.assertEqual(w.latch(), SITE_ZONE)
            w.serve_site(SITE)                 # ... and the owner removed it
            w.tick()
            self.assertEqual(w.latch(), SITE)

    def test_zone_edit_with_the_planner_source(self):
        for combo in self.each():
            w = self.world(combo, source='planner')
            self.assertEqual(w.latch(), PLAN)
            w.planner_edit(PLAN_ZONE)
            w.idle(1)
            self.assertEqual(w.latch(), PLAN_ZONE)

    def test_internal_switch_to_planner(self):
        """start_navi_launch at run time (load / edit / save of a map)."""
        for combo in self.each():
            w = self.world(combo)
            w.idle()                           # the flag is long consumed
            w.n._nm_map_source_set('planner')
            w.idle()
            self.assertEqual(w.latch(), PLAN)
            self.assertEqual(w.n.pub_map_source.msgs[-1].data, 'planner')

    def test_internal_switch_to_rtabmap(self):
        """callback_new_map: the rtabmap grid comes when rtabmap is up."""
        for combo in self.each():
            w = self.world(combo)
            w.n._nm_map_source_set('rtabmap')
            w.idle()                           # no rtabmap grid yet
            w.n.callback_rtabmap_grid_map(grid(RTAB))
            w.tick()
            self.assertEqual(w.latch(), RTAB)

    def test_switch_by_show_map_and_back(self):
        for combo in self.each():
            w = self.world(combo)
            w.n.callback_web_plan_show_map(String('planner'))
            w.idle()
            self.assertEqual(w.latch(), PLAN)
            w.serve_site(SITE_ZONE)            # edited while the planner grid is shown
            w.idle()
            w.n.callback_web_plan_show_map(String('octomap'))
            w.tick()
            self.assertEqual(w.latch(), SITE_ZONE)

    def test_switch_right_after_a_consumed_flag(self):
        """map_edited came BEFORE the switch and was spent on the old source."""
        for combo in self.each():
            w = self.world(combo)
            w.planner_2s()
            w.tick()
            self.assertFalse(w.n.is_map_to_publish)
            w.n._nm_map_source_set('planner')
            w.idle()
            self.assertEqual(w.latch(), PLAN)

    def test_navi_man_restart(self):
        for combo in self.each():
            for source in ('octomap', 'planner'):
                w = self.world(combo, source=source)
                w.serve_site(SITE_ZONE)
                w.planner_edit(PLAN_ZONE)
                w.idle()
                w.navi_man_start(source)       # the latch of the old process is gone
                w.idle()
                self.assertEqual(w.latch(), SITE_ZONE if source == 'octomap' else PLAN_ZONE)

    def test_planner_restart(self):
        for combo in self.each():
            for source in ('octomap', 'planner'):
                w = self.world(combo, source=source)
                w.serve_site(SITE_ZONE)
                w.planner_edit(PLAN_ZONE)
                w.idle()
                sent = len(w.n.nm_map_pub.msgs)
                w.planner_restart()
                w.idle()
                self.assertEqual(w.latch(), SITE_ZONE if source == 'octomap' else PLAN_ZONE)
                if combo[0]:
                    self.assertEqual(len(w.n.nm_map_pub.msgs), sent)   # nothing new to send

    def test_mbf_restart_and_clear_costmaps_read_the_latch(self):
        """Both only re-subscribe: what counts is that the latch is current at
        any moment after the tick that follows a change."""
        for combo in self.each():
            w = self.world(combo)
            for fill in (SITE_ZONE, SITE, SITE_ZONE):
                w.serve_site(fill)
                w.tick()
                self.assertEqual(w.latch(), fill)      # a costmap subscribing now
                w.idle(3)
                self.assertEqual(w.latch(), fill)      # ... or any time later

    def test_start_in_any_order(self):
        steps = {
            'site': lambda w: w.serve_site(SITE_ZONE),
            'planner': lambda w: (w.planner_edit(PLAN_ZONE), w.planner_2s()),
            'navi_man': None,
        }
        for combo in self.each():
            for source in ('octomap', 'planner'):
                for order in itertools.permutations(steps):
                    w = World(self.M, combo, True)
                    for name in order:
                        if name == 'navi_man':
                            w.navi_man_start(source)
                            w.tick()
                        else:
                            steps[name](w)
                            if w.n is not None:
                                w.tick()
                    w.idle()
                    self.assertEqual(w.latch(), SITE_ZONE if source == 'octomap' else PLAN_ZONE,
                                     (source, order))

    def test_pending_publish_waits_for_the_grid(self):
        """navi_man up before node_planner, source "planner"."""
        w = World(self.M, BOTH, True)
        w.navi_man_start('planner')
        w.tick(3)
        self.assertEqual(w.n.nm_map_pub.msgs, [])
        self.assertTrue(w.n.is_map_to_publish)
        w.planner_edit(PLAN)
        w.planner_2s()
        w.tick()
        self.assertEqual(w.latch(), PLAN)
        self.assertFalse(w.n.is_map_to_publish)

    def test_saving_is_kept(self):
        """Standing still with both switches: the grid goes out once."""
        w = self.world(BOTH)
        sent = len(w.n.nm_map_pub.msgs)
        w.idle(20)
        self.assertEqual(len(w.n.nm_map_pub.msgs), sent)


class WithoutTheFixTest(Base):
    """Why map_edited_on_change could not be turned on, and that off = as before."""

    def test_both_on_without_the_fix_keeps_the_old_source(self):
        w = self.world(BOTH, fix=False)
        w.idle()
        w.n._nm_map_source_set('planner')
        w.idle(5)
        self.assertEqual(w.latch(), SITE)      # the hole: still the previous source
        w2 = self.world(BOTH, fix=True)
        w2.idle()
        w2.n._nm_map_source_set('planner')
        w2.idle()
        self.assertEqual(w2.latch(), PLAN)

    def test_off_the_source_change_asks_for_nothing(self):
        for combo in COMBOS:
            w = self.world(combo, fix=False)
            w.tick()
            self.assertFalse(w.n.is_map_to_publish)
            remembered = w.n._nm_map_dedup._data
            w.n._nm_map_source_set('rtabmap')
            self.assertEqual(w.n.map_show, 'rtabmap')
            self.assertFalse(w.n.is_map_to_publish)
            self.assertIs(w.n._nm_map_dedup._data, remembered)

    def test_off_a_publish_without_a_grid_is_dropped_as_before(self):
        for source in ('planner', 'rtabmap'):
            w = World(self.M, OFF, False)
            w.navi_man_start(None, launch=False)
            w.n.map_show = source
            w.tick()
            self.assertFalse(w.n.is_map_to_publish)
            self.assertEqual(w.n.nm_map_pub.msgs, [])

    def test_off_and_on_send_the_same_with_the_periodic_planner(self):
        """Saving switches off: the fix changes nothing but the moment."""
        seq = {}
        for fix in (False, True):
            w = self.world(OFF, fix=fix)
            w.n._nm_map_source_set('planner')
            w.idle()
            w.serve_site(SITE_ZONE)
            w.n.callback_web_plan_show_map(String('octomap'))
            w.idle()
            seq[fix] = [m.data[0] for m in w.n.nm_map_pub.msgs]
        self.assertEqual(seq[False], seq[True])

    def test_shipped_config_is_off(self):
        with open(os.path.join(HERE, '..', 'config', 'navi_manager.yaml')) as f:
            self.assertIs(yaml.safe_load(f).get('map_source_republish'), False)

    def test_every_source_change_goes_through_one_place(self):
        with open(os.path.join(HERE, '..', 'nodes', 'navi_man')) as f:
            lines = [l.strip() for l in f if l.strip().startswith('self.map_show =')]
        self.assertEqual(sorted(lines), ["self.map_show = 'planner'",      # __init__
                                         'self.map_show = msg.data',        # show_map
                                         'self.map_show = source'])         # _nm_map_source_set

    def test_a_latched_map_edited_before_init_is_kept(self):
        with open(os.path.join(HERE, '..', 'nodes', 'navi_man')) as f:
            src = f.read()
        self.assertIn("self.planner_map_msg = getattr(self, 'planner_map_msg', None)", src)
        self.assertEqual(src.count('self.planner_map_msg = None'), 0)


if __name__ == '__main__':
    unittest.main()
