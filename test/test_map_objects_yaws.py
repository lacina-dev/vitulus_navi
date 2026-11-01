#!/usr/bin/env python3
"""The map editor's path round trip through the real navi_man callbacks.

map_objects hands the editor a heading per pose, save_path_at takes them back.
No ROS master needed: the Node is built without __init__, only what the two
callbacks touch is stubbed. Needs the sourced workspace.

Run:  python3 test/test_map_objects_yaws.py
"""
import importlib.machinery
import importlib.util
import json
import math
import os
import sys
import types
import unittest

import rospy
from geometry_msgs.msg import Point, Pose, Quaternion
from std_msgs.msg import String

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.append(os.path.join(HERE, '..', 'src'))   # after devel: vitulus_navi.srv lives there
try:
    import docker  # noqa: F401
except ImportError:
    sys.modules['docker'] = types.ModuleType('docker')


def load():
    path = os.path.join(HERE, '..', 'nodes', 'navi_man')
    loader = importlib.machinery.SourceFileLoader('navi_man_under_test', path)
    spec = importlib.util.spec_from_loader('navi_man_under_test', loader)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Sink:
    def __init__(self):
        self.last = None

    def publish(self, msg):
        self.last = msg


class PathYawsTest(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rospy.rostime.set_rostime_initialized(True)
        cls.M = load()

    def node(self, path_poses):
        n = self.M.Node.__new__(self.M.Node)
        n.running_map_data = self.M.MapData('t')
        n.status_msg_pub = Sink()
        n.map_objects_pub = Sink()
        n.save_running_map_data = lambda: None
        n._republish_map_lists = n.publish_map_objects
        p = self.M.MapPath('Rec', 'map')
        p.poses = path_poses
        p.create_path_msg()
        n.running_map_data.add_map_path(p)
        return n

    def objects(self, n):
        n.publish_map_objects()
        return json.loads(n.map_objects_pub.last)['paths'][0]

    @staticmethod
    def pose(x, y, yaw):
        return Pose(Point(x, y, 0.0), Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))

    def save(self, n, spec):
        n.callback_save_path_at(String(json.dumps(dict(spec, name='Rec'))))

    def test_map_objects_carry_a_heading_per_pose(self):
        yaws = [0.1, -2.5, 3.0]
        n = self.node([self.pose(i, 0, y) for i, y in enumerate(yaws)])
        got = self.objects(n)
        self.assertEqual(len(got['yaws']), len(got['points']))
        for a, b in zip(got['yaws'], yaws):
            self.assertAlmostEqual(a, b, places=12)

    def test_recorded_orientation_without_renormalising(self):
        # getCurrentPose zeroes x,y of a tilted orientation and keeps z,w
        z, w = math.sin(0.4) * 0.998, math.cos(0.4) * 0.998
        n = self.node([Pose(Point(0, 0, 0), Quaternion(0, 0, z, w)), self.pose(1, 0, 0)])
        self.assertAlmostEqual(self.objects(n)['yaws'][0], 0.8, places=12)

    def test_saving_the_same_headings_changes_nothing(self):
        n = self.node([self.pose(i, i * 0.5, 0.2 * i - 1) for i in range(6)])
        before = self.objects(n)
        self.save(n, before)
        after = self.objects(n)
        self.assertEqual(after['points'], before['points'])
        for a, b in zip(after['yaws'], before['yaws']):
            self.assertAlmostEqual(a, b, places=12)

    def test_only_the_changed_headings_change(self):
        n = self.node([self.pose(i, 0, 0.3) for i in range(5)])
        got = self.objects(n)
        got['yaws'][2] = 1.0
        self.save(n, got)
        after = self.objects(n)['yaws']
        for i in (0, 1, 3, 4):
            self.assertAlmostEqual(after[i], 0.3, places=12)
        self.assertAlmostEqual(after[2], 1.0, places=12)

    def test_without_yaws_the_line_decides(self):
        n = self.node([self.pose(i, 0, 0.3) for i in range(3)])
        self.save(n, {'points': [[0, 0], [0, 1], [1, 1]]})
        after = self.objects(n)['yaws']
        self.assertAlmostEqual(after[0], math.pi / 2)
        self.assertAlmostEqual(after[1], 0.0)
        self.assertAlmostEqual(after[2], 0.0)


if __name__ == '__main__':
    unittest.main()
