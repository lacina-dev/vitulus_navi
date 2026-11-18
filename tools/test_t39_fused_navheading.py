#!/usr/bin/env python3
"""T39: callback_imu -> publish_fused_navheading (100 Hz) after the clean-up
publishes BIT FOR BIT what it published before.

Before: 3x euler_from_quaternion + tf's quaternion_from_euler (numpy) per IMU
sample. Now: the Euler angles of the sample are computed once and handed over,
and the quaternion is tf's arithmetic written out (quat_from_euler_sxyz).

  - quat_from_euler_sxyz == tf.transformations.quaternion_from_euler, bit for
    bit: 100 000 random angle triples (robot-like, any, huge, tiny, signed zero)
    and NaN / inf;
  - the two methods exactly as they stand in the node, run on a stand-in object,
    against the OLD code kept here verbatim: the published /gnss_heading/nav_fused
    orientation (bytes of the serialized message), imu_yaw and fused_yaw are bit
    equal for every recorded /bno085/imu sample of a drive
    (~/bags/nav_test/nav_n2_base_8.bag, first 20 000; skipped when the bag is
    not there) and 20 000 random ones, with init on/off and several offsets;
  - publish_fused_navheading() without arguments (other callers) still works.
Needs tf.transformations, numpy, sensor_msgs (robot environment). No ROS master.

Run:  python3 tools/test_t39_fused_navheading.py
"""
import ast
import io
import math
import os
import random
import struct
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _ros_env  # noqa: E402
_ros_env.ensure()

from sensor_msgs.msg import Imu  # noqa: E402
from tf.transformations import euler_from_quaternion as tf_euler  # noqa: E402
from tf.transformations import quaternion_from_euler as tf_quat  # noqa: E402

NODE = os.path.join(os.path.dirname(HERE), 'nodes', 'navi_transform')
BAG = os.path.expanduser('~/bags/nav_test/nav_n2_base_8.bag')


class _Time(object):
    @staticmethod
    def now():
        import genpy
        return genpy.Time(1000, 0)


class _Rospy(object):
    Time = _Time


class _Pub(object):
    def __init__(self):
        self.last = None

    def publish(self, msg):
        self.last = msg


def load():
    """The functions and the two methods exactly as they stand in the node."""
    tree = ast.parse(open(NODE).read())
    top = [n for n in tree.body
           if (isinstance(n, ast.FunctionDef)
               and n.name in ('euler_from_quaternion', 'quat_from_euler_sxyz'))
           or (isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') == '_EULER_EPS')]
    assert len(top) == 3, [getattr(n, 'name', '=') for n in top]
    methods = [m for c in tree.body if isinstance(c, ast.ClassDef) for m in c.body
               if isinstance(m, ast.FunctionDef)
               and m.name in ('callback_imu', 'publish_fused_navheading')]
    assert len(methods) == 2, [m.name for m in methods]
    cls = ast.ClassDef(name='Node', bases=[], keywords=[], body=methods, decorator_list=[])
    import numpy as np
    ns = {'math': math, 'np': np, '_tf_euler_from_quaternion': tf_euler, 'Imu': Imu,
          'rospy': _Rospy}
    mod = ast.Module(body=top + [cls], type_ignores=[])
    ast.fix_missing_locations(mod)
    exec(compile(mod, NODE, 'exec'), ns)
    return ns


NS = load()
FAST_Q = NS['quat_from_euler_sxyz']
EULER = NS['euler_from_quaternion']


class Old(object):
    """callback_imu + publish_fused_navheading as they were before (verbatim)."""

    def callback_imu(self, msg):
        self.imu_msg = msg
        (roll, pitch, yaw) = EULER(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.imu_yaw = yaw
        self.publish_fused_navheading()

    def publish_fused_navheading(self):
        (imu_roll, imu_pitch, imu_yaw) = EULER([self.imu_msg.orientation.x,
                                                self.imu_msg.orientation.y,
                                                self.imu_msg.orientation.z,
                                                self.imu_msg.orientation.w])
        if imu_yaw < 0:
            imu_yaw += 2 * math.pi

        if self.init:
            imu_yaw = imu_yaw + self.set_pose_yaw
        else:
            imu_yaw = imu_yaw + self.diff_yaw
        quaternion = tf_quat(imu_roll, imu_pitch, imu_yaw)
        fused_nav = Imu()
        fused_nav.header.stamp = _Time.now()
        fused_nav.header.frame_id = "base_link"
        fused_nav.orientation.x = quaternion[0]
        fused_nav.orientation.y = quaternion[1]
        fused_nav.orientation.z = quaternion[2]
        fused_nav.orientation.w = quaternion[3]
        fused_nav.orientation_covariance = self.imu_msg.orientation_covariance
        fused_nav.angular_velocity = self.imu_msg.angular_velocity
        fused_nav.angular_velocity_covariance = self.imu_msg.angular_velocity_covariance
        fused_nav.linear_acceleration = self.imu_msg.linear_acceleration
        fused_nav.linear_acceleration_covariance = self.imu_msg.linear_acceleration_covariance
        self.fused_nav_pub.publish(fused_nav)
        self.fused_msg = fused_nav
        (fused_roll, fused_pitch, fused_yaw) = EULER([self.fused_msg.orientation.x,
                                                      self.fused_msg.orientation.y,
                                                      self.fused_msg.orientation.z,
                                                      self.fused_msg.orientation.w])
        self.fused_yaw = fused_yaw


def setup(obj, init, off):
    obj.init, obj.set_pose_yaw, obj.diff_yaw = init, off, off * 0.7 - 0.3
    obj.fused_nav_pub = _Pub()
    return obj


def bits(x):
    return struct.pack('<d', float(x))


def wire(msg):
    buf = io.BytesIO()
    msg.serialize(buf)
    return buf.getvalue()


def bag_quats(limit=20000):
    if not os.path.isfile(BAG):
        return None
    import rosbag
    out = []
    with rosbag.Bag(BAG) as bag:
        for _tp, m, _t in bag.read_messages(topics=['/bno085/imu']):
            o = m.orientation
            out.append((o.x, o.y, o.z, o.w))
            if len(out) >= limit:
                break
    return out


def random_quats(n, seed=752):
    random.seed(seed)
    out = []
    for i in range(n):
        if i % 2:
            q = [random.gauss(0, 1) for _ in range(4)]
        else:
            r, p, yw = random.gauss(0, 0.05), random.gauss(0, 0.05), random.uniform(-4, 4)
            q = list(tf_quat(r, p, yw))
        out.append(tuple(float(a) for a in q))
    return out


class Quaternion(unittest.TestCase):
    def test_bit_equal_to_tf(self):
        random.seed(39)
        bad = 0
        for i in range(100000):
            k = i % 4
            if k == 0:
                a = (random.gauss(0, 0.05), random.gauss(0, 0.05), random.uniform(-10, 10))
            elif k == 1:
                a = tuple(random.uniform(-math.pi, math.pi) for _ in range(3))
            elif k == 2:
                a = tuple(random.gauss(0, 1) * 10 ** random.randint(-12, 6) for _ in range(3))
            else:
                a = (random.choice([0.0, -0.0, 1e-300]), random.choice([0.0, -0.0, math.pi]),
                     random.uniform(-7, 7))
            if [bits(v) for v in FAST_Q(*a)] != [bits(v) for v in tf_quat(*a)]:
                bad += 1
        self.assertEqual(bad, 0)

    def test_nan_inf(self):
        for a in ((float('nan'), 0.0, 0.0), (0.0, float('nan'), 1.0)):
            f, t = FAST_Q(*a), tf_quat(*a)
            self.assertEqual([v != v for v in f], [v != v for v in t])
        self.assertRaises(ValueError, FAST_Q, float('inf'), 0.0, 0.0)
        self.assertRaises(ValueError, tf_quat, float('inf'), 0.0, 0.0)


class WholePath(unittest.TestCase):
    def compare(self, quats):
        n = 0
        for init, off in ((False, 0.0), (False, 1.234), (True, -2.9), (True, 5.5)):
            new, old = setup(NS['Node'](), init, off), setup(Old(), init, off)
            for q in quats:
                m = Imu()
                m.orientation.x, m.orientation.y, m.orientation.z, m.orientation.w = q
                m.angular_velocity.z = q[0]
                new.callback_imu(m)
                old.callback_imu(m)
                if (wire(new.fused_nav_pub.last) != wire(old.fused_nav_pub.last)
                        or bits(new.fused_yaw) != bits(old.fused_yaw)
                        or bits(new.imu_yaw) != bits(old.imu_yaw)
                        or new.fused_msg is not new.fused_nav_pub.last):
                    self.fail('differs at sample %d (init %s, offset %s): %s' % (n, init, off, q))
                n += 1
        return n

    def test_recorded_imu_samples(self):
        quats = bag_quats()
        if quats is None:
            self.skipTest('no %s' % BAG)
        self.assertGreater(len(quats), 10000)
        n = self.compare(quats)
        print('recorded: %d comparisons bit equal' % n)

    def test_random_samples(self):
        n = self.compare(random_quats(20000))
        print('random: %d comparisons bit equal' % n)

    def test_without_arguments_as_other_callers(self):
        new, old = setup(NS['Node'](), False, 0.4), setup(Old(), False, 0.4)
        m = Imu()
        m.orientation.z, m.orientation.w = 0.6, 0.8
        new.imu_msg = old.imu_msg = m
        new.publish_fused_navheading()
        old.publish_fused_navheading()
        self.assertEqual(wire(new.fused_nav_pub.last), wire(old.fused_nav_pub.last))
        self.assertEqual(bits(new.fused_yaw), bits(old.fused_yaw))


if __name__ == '__main__':
    unittest.main(verbosity=1)
