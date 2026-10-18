#!/usr/bin/env python3
"""Offline test for TF-ATOMIC (2026-10-18, N1): navi_transform.loop30hz must send
odom->base_link and map->odom in ONE /tf message with ONE stamp (map->odom
first), so a consumer never holds odom->base(t) without map->odom(t).

Part 1 drives the real loop30hz on a stub node (no ROS master, no node init).
Part 2 replays the old vs new arrival order into a tf2 BufferCore and does the
lookup TEB does (robot pose stamped = latest odom->base, transformed to map
with no timeout) — the old order reproduces the field error
"extrapolation ~0.03 s into the future ... from [odom] to [map]".

Run:  python3 test_tf_atomic.py   (needs rospy/tf2 importable — robot env)
"""
import os
import sys
import importlib.util
import importlib.machinery

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _ros_env  # noqa: E402
_ros_env.ensure()

import rospy  # noqa: E402
import tf2_py  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402

NAVI_TF = os.path.join(HERE, '..', 'nodes', 'navi_transform')


def load(name, path):
    spec = importlib.util.spec_from_loader(
        name, importlib.machinery.SourceFileLoader(name, path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class FakeClock(object):
    def __init__(self, t0=1000.0, step=0.002):
        self.t = t0
        self.step = step

    def now(self):
        self.t += self.step   # every call advances: separate now() calls differ
        return rospy.Time.from_sec(self.t)


class FakeBroadcaster(object):
    def __init__(self):
        self.calls = []

    def sendTransform(self, t):
        self.calls.append(t if isinstance(t, list) else [t])


class FakePub(object):
    def publish(self, msg):
        pass


def make_node(nt, slew_raises=False):
    node = object.__new__(nt.NavTfNode)
    node.is_indoor = False
    node.tf2_TransformBroadcaster = FakeBroadcaster()
    node.odometry_odom_pub = FakePub()
    msg = nt.Odometry() if hasattr(nt, 'Odometry') else None
    if msg is None:
        from nav_msgs.msg import Odometry
        msg = Odometry()
    msg.pose.pose.position.x = 1.0
    msg.pose.pose.orientation.w = 1.0
    node.ekf_wheel_nav_odom_msg = msg
    mo = TransformStamped()
    mo.transform.translation.x = 5.0
    mo.transform.rotation.w = 1.0
    node.map_odom_transform = mo
    node.map_odom_yaw = 0.0
    import threading
    node._map_odom_lock = threading.RLock()
    node.nav_last_fused_time = rospy.Time.from_sec(1000.0)
    node.fix_last_fused_time = rospy.Time.from_sec(1000.0)
    node._bridge_compute_enable = False
    node.publish_odom_for_gps = lambda: None
    node.init_navsat = lambda: None
    node.dock_anchor_hold = lambda: None
    node.get_base_link_transform = lambda stamp=None: None

    def slew():
        if slew_raises:
            raise RuntimeError('slew boom')
    node.slew_map_pose = slew
    return node


def tf(parent, child, stamp, x):
    t = TransformStamped()
    t.header.frame_id = parent
    t.header.stamp = rospy.Time.from_sec(stamp)
    t.child_frame_id = child
    t.transform.translation.x = x
    t.transform.rotation.w = 1.0
    return t


def teb_lookup(buf):
    """What TEB transformGlobalPlan does with the robot pose: stamp = latest
    odom->base (costmap getRobotPose), then map<-odom AT that stamp, no wait."""
    latest_ob = buf.lookup_transform_core('odom', 'base_link', rospy.Time(0))
    buf.lookup_transform_core('map', 'odom', latest_ob.header.stamp)


def main():
    nt = load('navi_transform_node', NAVI_TF)
    fails = []

    def check(name, cond):
        print('  %-60s %s' % (name, 'OK' if cond else 'FAIL'))
        if not cond:
            fails.append(name)

    real_now = rospy.Time.now
    clock = FakeClock()
    rospy.Time.now = staticmethod(clock.now)
    try:
        print('loop30hz broadcast:')
        node = make_node(nt)
        node.loop30hz()
        calls = node.tf2_TransformBroadcaster.calls
        check('exactly one sendTransform per tick', len(calls) == 1)
        frames = [(t.header.frame_id, t.child_frame_id) for t in calls[0]]
        check('message = [map->odom, odom->base_link] in that order',
              frames == [('map', 'odom'), ('odom', 'base_link')])
        st = set(t.header.stamp for t in calls[0])
        check('both transforms carry the same stamp', len(st) == 1)
        check('values unchanged (map->odom x=5, odom->base x=1)',
              calls[0][0].transform.translation.x == 5.0
              and calls[0][1].transform.translation.x == 1.0)

        node = make_node(nt, slew_raises=True)
        raised = False
        try:
            node.loop30hz()
        except RuntimeError:
            raised = True
        calls = node.tf2_TransformBroadcaster.calls
        check('slew exception still propagates (unchanged)', raised)
        check('...and odom->base still goes out that tick',
              len(calls) == 1 and [(t.header.frame_id, t.child_frame_id)
                                   for t in calls[0]] == [('odom', 'base_link')])

        node = make_node(nt)
        node.is_indoor = True
        node.loop30hz()
        check('indoor: nothing broadcast (EKF owns TF, unchanged)',
              node.tf2_TransformBroadcaster.calls == [])
    finally:
        rospy.Time.now = real_now

    print('tf2 replay (TEB robot-pose lookup):')
    P = 1.0 / 30.0
    # OLD: two messages per tick, map->odom stamped ~2 ms after odom->base and
    # arriving after it. State "odom->base(N) arrived, map->odom(N) not yet".
    buf = tf2_py.BufferCore(rospy.Duration(10.0))
    for n in range(5):
        t = 2000.0 + n * P
        buf.set_transform(tf('odom', 'base_link', t, 0.0), 'old')
        buf.set_transform(tf('map', 'odom', t + 0.002, 0.0), 'old')
    buf.set_transform(tf('odom', 'base_link', 2000.0 + 5 * P, 0.0), 'old')
    err = None
    try:
        teb_lookup(buf)
    except tf2_py.ExtrapolationException as e:
        err = str(e)
    check('OLD order reproduces the ExtrapolationException', err is not None)
    if err:
        print('     -> %s' % err.split(' when ')[0])

    # NEW: map->odom(N) then odom->base(N), same stamp, every tick.
    buf = tf2_py.BufferCore(rospy.Duration(10.0))
    ok = True
    for n in range(6):
        t = 2000.0 + n * P
        buf.set_transform(tf('map', 'odom', t, 0.0), 'new')
        # lookup in the gap between the two set_transform calls of one message
        # (from tick 1 on — before tick 0's odom->base there is no tree yet)
        if n > 0:
            try:
                teb_lookup(buf)
            except tf2_py.TransformException:
                ok = False
                break
        buf.set_transform(tf('odom', 'base_link', t, 0.0), 'new')
        try:
            teb_lookup(buf)
        except tf2_py.TransformException:
            ok = False
            break
    check('NEW order: lookup succeeds at every point of every tick', ok)

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %s' % fails))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
