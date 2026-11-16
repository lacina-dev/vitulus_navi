#!/usr/bin/env python3
"""/navi_manager/map only when the grid changed (T39 C2), offline: no master.

    python3 test/test_map_publish_on_change.py      (sourced workspace)

16. 11. 2026, measured on the robot standing docked: node_planner sent
/web_plan/map_edited every 2 s, navi_man (source "octomap") answered each slow
tick with the same site raster - the same 1035x1931 cells, the same stamp -
and move_base_flex + rosbridge took the 2 mil. cells over again every time.

Covered here, through the real navi_man callbacks and publish_map:
  * switch off (default): every re-armed publish goes out, as before
  * switch on: the same grid goes out once; a changed grid (cells, size,
    origin, frame, another source) goes out on the next tick
  * the source selection (/navi_manager/show_map) always sends the grid
  * map_source keeps its rhythm either way
  * the topic is latched, and the shipped default is off
"""
import importlib.machinery
import importlib.util
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

# This tree's packages, then devel again: vitulus_navi.srv is generated there.
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


def grid(w=4, h=3, fill=0, stamp=1.0, frame='map', ox=0.0, res=0.05):
    m = OccupancyGrid()
    m.header.frame_id = frame
    m.header.stamp = rospy.Time.from_sec(stamp)
    m.info.width, m.info.height, m.info.resolution = w, h, res
    m.info.origin.position.x = ox
    m.info.origin.orientation.w = 1.0
    m.data = tuple([fill] * (w * h))      # int8[] arrives as a tuple
    return m


class Sink(object):
    def __init__(self):
        self.msgs = []

    def publish(self, msg):
        self.msgs.append(msg)


class DedupTest(unittest.TestCase):

    def test_nothing_published_yet_is_never_the_same(self):
        self.assertFalse(map_dedup.MapDedup().is_same(grid()))

    def test_same_object_and_equal_copy_are_the_same(self):
        d = map_dedup.MapDedup()
        a = grid()
        d.note_published(a)
        self.assertTrue(d.is_same(a))
        self.assertTrue(d.is_same(grid(stamp=99.0)))          # the stamp does not count

    def test_any_real_difference_is_a_change(self):
        d = map_dedup.MapDedup()
        d.note_published(grid())
        one_cell = grid()
        one_cell.data = one_cell.data[:-1] + (100,)
        for other in (one_cell, grid(w=3, h=4), grid(ox=0.05), grid(frame='odom'),
                      grid(res=0.1), grid(fill=-1)):
            self.assertFalse(d.is_same(other))

    def test_another_container_type_is_a_change(self):
        d = map_dedup.MapDedup()
        d.note_published(grid())
        as_list = grid()
        as_list.data = list(as_list.data)
        self.assertFalse(d.is_same(as_list))                   # unsure -> publish

    def test_forget(self):
        d = map_dedup.MapDedup()
        a = grid()
        d.note_published(a)
        d.forget()
        self.assertFalse(d.is_same(a))

    def test_full_size_compare_is_cheap(self):
        import time
        d = map_dedup.MapDedup()
        d.note_published(grid(w=1035, h=1931))
        other = grid(w=1035, h=1931)
        t = time.process_time()
        self.assertTrue(d.is_same(other))
        self.assertLess(time.process_time() - t, 0.5)


class NaviManTest(unittest.TestCase):

    @classmethod
    def setUpClass(cls):
        rospy.rostime.set_rostime_initialized(True)
        cls.M = load()

    def node(self, on_change, show='octomap'):
        n = self.M.Node.__new__(self.M.Node)
        n.params = {'map_publish_on_change': on_change} if on_change is not None else {}
        n.nm_map_pub = Sink()
        n.pub_map_source = Sink()
        n.is_rtabmap_pub = Sink()
        n.pub_serve_site = Sink()
        n.map_show = show
        n.is_map_to_publish = True
        n.rtabmap_grid_map_msg = None
        n.planner_map_msg = None
        n.site_map_msg = None
        n._octomap_serve_deadline = None
        n._octomap_last_serve_req = None
        n._octomap_serve_window = 60.0
        n._octomap_fallback_active = False
        n._octomap_fallback_mode = 'bootstrap'
        n._bootstrap_map_msg = None
        n._bootstrap_build_attempts = 0
        n._nm_map_dedup = map_dedup.MapDedup()
        n._nm_map_on_change = None
        return n

    def docked_idle(self, n, ticks=5):
        """What the robot does standing docked: the planner grid arrives (2 s),
        the slow loop ticks."""
        for i in range(ticks):
            n.callback_web_plan_map(grid(fill=50, stamp=100.0 + 2 * i))
            n.publish_map()

    # --- switch off: as before -------------------------------------------
    def test_default_is_off_and_repeats_every_tick(self):
        for on_change in (None, False):
            n = self.node(on_change)
            n.callback_site_map(grid())
            self.docked_idle(n, 5)
            self.assertEqual(len(n.nm_map_pub.msgs), 5)
            self.assertEqual(len(n.pub_map_source.msgs), 5)

    def test_shipped_config_is_off(self):
        with open(os.path.join(HERE, '..', 'config', 'navi_manager.yaml')) as f:
            self.assertIs(yaml.safe_load(f).get('map_publish_on_change'), False)

    # --- switch on -------------------------------------------------------
    def test_on_the_same_site_raster_goes_out_once(self):
        n = self.node(True)
        site = grid()
        n.callback_site_map(site)
        self.docked_idle(n, 5)
        self.assertEqual(n.nm_map_pub.msgs, [site])
        self.assertEqual([m.data for m in n.pub_map_source.msgs], ['octomap'] * 5)
        self.assertEqual(n._nm_map_dedup.skipped, 4)

    def test_on_a_new_raster_goes_out_on_the_next_tick(self):
        n = self.node(True)
        n.callback_site_map(grid())
        self.docked_idle(n, 3)
        edited = grid(fill=100)                    # zone / edit / newer raster
        n.callback_site_map(edited)
        n.publish_map()
        self.assertIs(n.nm_map_pub.msgs[-1], edited)
        self.assertEqual(len(n.nm_map_pub.msgs), 2)
        resized = grid(w=6, h=6)
        n.callback_site_map(resized)
        n.publish_map()
        self.assertIs(n.nm_map_pub.msgs[-1], resized)

    def test_on_same_raster_served_again_is_not_repeated(self):
        """mapping_manager re-serving the identical raster (a new message object)."""
        n = self.node(True)
        n.callback_site_map(grid())
        n.publish_map()
        n.callback_site_map(grid(stamp=7.0))
        n.publish_map()
        self.assertEqual(len(n.nm_map_pub.msgs), 1)

    def test_on_planner_source_follows_the_edits(self):
        n = self.node(True, show='planner')
        for i in range(4):
            n.callback_web_plan_map(grid(fill=50, stamp=100.0 + 2 * i))
            n.publish_map()
        self.assertEqual(len(n.nm_map_pub.msgs), 1)
        n.callback_web_plan_map(grid(fill=0))      # the planner edited the map
        n.publish_map()
        self.assertEqual(len(n.nm_map_pub.msgs), 2)
        self.assertEqual(n.nm_map_pub.msgs[-1].data[0], 0)

    def test_on_source_switch_sends_the_other_grid(self):
        n = self.node(True)
        n.callback_site_map(grid(fill=0))
        self.docked_idle(n, 2)
        n.callback_web_plan_show_map(String('planner'))
        n.publish_map()
        self.assertEqual(n.nm_map_pub.msgs[-1].data[0], 50)
        n.callback_web_plan_show_map(String('octomap'))
        n.publish_map()
        self.assertEqual(n.nm_map_pub.msgs[-1].data[0], 0)
        self.assertEqual(len(n.nm_map_pub.msgs), 3)

    def test_on_selecting_the_same_source_again_sends_the_grid(self):
        """The UI's way to ask for the map again."""
        n = self.node(True)
        n.callback_site_map(grid())
        self.docked_idle(n, 2)
        n.callback_web_plan_show_map(String('octomap'))
        n.publish_map()
        self.assertEqual(len(n.nm_map_pub.msgs), 2)

    def test_on_empty_site_falls_back_and_comes_back(self):
        """A fresh site (empty grid) -> bootstrap stand-in -> the raster again."""
        n = self.node(True)
        site = grid()
        n.callback_site_map(site)
        n.publish_map()
        boot = grid(fill=-1)
        n._build_bootstrap_map = lambda: boot
        n.callback_site_map(OccupancyGrid())
        n.publish_map()
        self.assertIs(n.nm_map_pub.msgs[-1], boot)
        self.docked_idle(n, 3)                     # the stand-in is not repeated either
        self.assertEqual(len(n.nm_map_pub.msgs), 2)
        n.callback_site_map(site)                  # the very same raster returns
        n.publish_map()
        self.assertIs(n.nm_map_pub.msgs[-1], site)
        self.assertEqual(len(n.nm_map_pub.msgs), 3)

    def test_the_topic_is_latched(self):
        with open(os.path.join(HERE, '..', 'nodes', 'navi_man')) as f:
            src = f.read()
        self.assertIn("rospy.Publisher('navi_manager/map', OccupancyGrid, queue_size=1, latch=True)", src)
        self.assertEqual(src.count('self.nm_map_pub.publish('), 1)   # only via _nm_map_publish


if __name__ == '__main__':
    unittest.main()
