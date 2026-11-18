#!/usr/bin/env python3
"""T39: /gnss_heading/yaws (10 Hz) and /gnss_heading/markers (3 Hz) are built
and published only while somebody is connected.

Readers of the two topics: the IMU calibration page of the UI (rosbridge) and
recorders - both are connections. Nothing in the node reads the marker_*
attributes except publish_markers itself.
  - nobody connected: nothing published, no marker built;
  - somebody connected: the same messages as before (yaws in degrees rounded
    to 0.01, three markers NAV_RTK / NAV_FUSE / NAV_IMU).
The methods are taken from the node's source (no ROS master, the node is not run).

Run:  python3 tools/test_t39_yaws_markers_on_demand.py
"""
import ast
import math
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _ros_env  # noqa: E402
_ros_env.ensure()

import genpy  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from std_msgs.msg import Float32MultiArray  # noqa: E402
from visualization_msgs.msg import Marker, MarkerArray  # noqa: E402

NODE = os.path.join(os.path.dirname(HERE), 'nodes', 'navi_transform')
WANT = ('publish_yaws', 'publish_markers', 'update_navheading_marker', 'update_fuse_marker',
        'update_imu_marker', 'get_marker')


class _Time(object):
    @staticmethod
    def now():
        return genpy.Time(1000, 0)


class _Rospy(object):
    Time = _Time


class Pub(object):
    def __init__(self, connections):
        self.connections = connections
        self.sent = []

    def get_num_connections(self):
        return self.connections

    def publish(self, msg):
        self.sent.append(msg)


def load():
    tree = ast.parse(open(NODE).read())
    methods = [m for c in tree.body if isinstance(c, ast.ClassDef) for m in c.body
               if isinstance(m, ast.FunctionDef) and m.name in WANT]
    assert sorted(m.name for m in methods) == sorted(WANT), [m.name for m in methods]
    cls = ast.ClassDef(name='Node', bases=[], keywords=[], body=methods, decorator_list=[])
    mod = ast.Module(body=[cls], type_ignores=[])
    ast.fix_missing_locations(mod)
    ns = {'math': math, 'rospy': _Rospy, 'Marker': Marker, 'MarkerArray': MarkerArray,
          'Float32MultiArray': Float32MultiArray}
    exec(compile(mod, NODE, 'exec'), ns)
    return ns['Node']


Node = load()


def quat_msg(z, w):
    m = Imu()
    m.orientation.z, m.orientation.w = z, w
    return m


def node(connections):
    n = Node()
    n.yaws_pub, n.marker_pub = Pub(connections), Pub(connections)
    n.imu_yaw, n.navheading_yaw, n.fused_yaw = 0.5, -1.25, 3.0
    n.imu_msg, n.navheading_msg, n.fused_msg = quat_msg(0.1, 0.99), quat_msg(0.6, 0.8), quat_msg(1.0, 0.0)
    n.marker_imu, n.marker_nav, n.marker_fuse = Marker(), Marker(), Marker()
    return n


class OnDemand(unittest.TestCase):
    def test_nobody_listening_nothing_built(self):
        n = node(0)
        built = []
        n.get_marker = lambda *a, **k: built.append(a)
        n.publish_yaws()
        n.publish_markers()
        self.assertEqual(n.yaws_pub.sent, [])
        self.assertEqual(n.marker_pub.sent, [])
        self.assertEqual(built, [])

    def test_listener_gets_the_same_yaws_as_before(self):
        n = node(1)
        n.publish_yaws()
        self.assertEqual(len(n.yaws_pub.sent), 1)
        self.assertEqual(list(n.yaws_pub.sent[0].data),
                         [round(0.5 * 180.0 / math.pi, 2), round(-1.25 * 180.0 / math.pi, 2),
                          round(3.0 * 180.0 / math.pi, 2)])

    def test_listener_gets_the_three_markers(self):
        n = node(2)
        n.publish_markers()
        self.assertEqual(len(n.marker_pub.sent), 1)
        ms = n.marker_pub.sent[0].markers
        self.assertEqual([m.ns for m in ms], ['NAV_RTK', 'NAV_FUSE', 'NAV_IMU'])
        self.assertEqual([(m.pose.orientation.z, m.pose.orientation.w) for m in ms],
                         [(0.6, 0.8), (1.0, 0.0), (0.1, 0.99)])
        self.assertTrue(all(m.header.frame_id == 'bno_imu_link' for m in ms))

    def test_starts_as_soon_as_somebody_connects(self):
        n = node(0)
        n.publish_yaws()
        n.publish_markers()
        n.yaws_pub.connections = n.marker_pub.connections = 1
        n.publish_yaws()
        n.publish_markers()
        self.assertEqual((len(n.yaws_pub.sent), len(n.marker_pub.sent)), (1, 1))


if __name__ == '__main__':
    unittest.main(verbosity=1)
