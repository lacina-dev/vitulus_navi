#!/usr/bin/env python3
"""Offline test for T28 (2026-11-08): navi_transform must never write the GNSS
altitude (/odometry/gps z) — nor a jump to 0 — into the nav EKF via set_pose.
Every odom-frame set_pose keeps the EKF's own current z; the external
map-frame set_pose stays as is (robot_localization's map->odom transform
already keeps z). So /odometry/odom z stays continuous (field: -32 cm on GPS-ownership resume, +13 cm on navsat relaunch).
Also: map->odom.z is built from THIS tick's odom->base (no one-tick spike).

Drives the real methods on a stub node (no ROS master, no node init); the
set_pose service is replaced by a recorder.

Run:  python3 test_t28_z_keep.py   (needs rospy importable — robot env / vitulus-dev)
"""
import os
import sys
import threading
import importlib.util
import importlib.machinery

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _ros_env  # noqa: E402
_ros_env.ensure()

import types  # noqa: E402


def _stub_missing_msgs():
    """Dev container without the robot's full devel space: sensor_msgs_ext and
    the generated vitulus_navi.srv are missing. navi_transform only imports
    them at module level (the tested methods never touch them), so stand-ins
    are enough. A real install keeps the real modules."""
    try:
        import sensor_msgs_ext.msg  # noqa: F401
    except ImportError:
        pkg = types.ModuleType('sensor_msgs_ext')
        msg = types.ModuleType('sensor_msgs_ext.msg')
        msg.magnetometer = msg.accelerometer = object
        pkg.msg = msg
        sys.modules['sensor_msgs_ext'] = pkg
        sys.modules['sensor_msgs_ext.msg'] = msg
    try:
        import vitulus_navi.srv  # noqa: F401
    except ImportError:
        try:
            import vitulus_navi as pkg
        except ImportError:
            pkg = types.ModuleType('vitulus_navi')
            pkg.__path__ = []
            sys.modules['vitulus_navi'] = pkg
        srv = types.ModuleType('vitulus_navi.srv')
        srv.Point_pose = srv.Point_poseResponse = object
        pkg.srv = srv
        sys.modules['vitulus_navi.srv'] = srv


_stub_missing_msgs()

import rospy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from geometry_msgs.msg import (PoseWithCovarianceStamped,  # noqa: E402
                               TransformStamped, Pose)

NAVI_TF = os.path.join(HERE, '..', 'nodes', 'navi_transform')

EKF_Z = 0.013      # nav EKF z right now
GNSS_Z = -0.303    # /odometry/gps z (navsat-datum relative GNSS altitude)


def load(name, path):
    spec = importlib.util.spec_from_loader(
        name, importlib.machinery.SourceFileLoader(name, path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class SetPoseRecorder(object):
    def __init__(self):
        self.reqs = []

    def proxy(self, name, srv_type):
        def call(req):
            self.reqs.append((name, req))
        return call


def odom(x, y, z):
    m = Odometry()
    m.pose.pose.position.x = x
    m.pose.pose.position.y = y
    m.pose.pose.position.z = z
    m.pose.pose.orientation.w = 1.0
    return m


def make_node(nt):
    n = object.__new__(nt.NavTfNode)
    n.ekf_wheel_nav_odom_msg = odom(0.0, 0.0, EKF_Z)
    n.ekf_wheel_odom_msg = odom(0.0, 0.0, 0.0)
    n.rtk_odom_msg = odom(1.0, 0.0, GNSS_Z)
    fused = nt.Imu() if hasattr(nt, 'Imu') else None
    if fused is None:
        from sensor_msgs.msg import Imu
        fused = Imu()
    fused.orientation.w = 1.0
    n.fused_msg = fused
    n._anchor_deadband_m = 0.015
    n._pos_slew_mps = 0.5
    return n


def last_z(rec):
    return rec.reqs[-1][1].pose.pose.pose.position.z


def main():
    nt = load('navi_transform_t28', NAVI_TF)
    fails = []

    def check(name, cond):
        print('  %-66s %s' % (name, 'OK' if cond else 'FAIL'))
        if not cond:
            fails.append(name)

    rec = SetPoseRecorder()
    real_now = rospy.Time.now
    rospy.Time.now = staticmethod(lambda: rospy.Time(1000))   # no init_node
    saved = (rospy.wait_for_service, rospy.ServiceProxy)
    rospy.wait_for_service = lambda *a, **k: None
    rospy.ServiceProxy = rec.proxy
    try:
        print('set_pose z (T28):')
        # 1) legacy RTK correction: x/y toward RTK, z = EKF (not GNSS)
        n = make_node(nt)
        n.set_wheel_base_nav_pose()
        check('set_wheel_base_nav_pose(None): one set_pose sent', len(rec.reqs) == 1)
        check('  ... z = EKF z, not GNSS altitude', abs(last_z(rec) - EKF_Z) < 1e-9)
        check('  ... x/y still slewed toward RTK (0.05 m step)',
              abs(rec.reqs[-1][1].pose.pose.pose.position.x - 0.05) < 1e-9)

        # 2) continuous anchor (the 2026-11-08 18:22:25 path)
        n = make_node(nt)
        n._pose_buffer = [object()]
        n._anchor_lag_s = 2.0
        n._anchor_verified = None
        cand = (100.0, (1.0, 0.0), (0.0, 0.0), 0.0, 0.0)
        n._buffer_newest_before = lambda t: cand
        n._entry_motion_verified = lambda c: True
        k = len(rec.reqs)
        ok = n._apply_continuous_anchor()
        check('_apply_continuous_anchor: wrote a target', ok and len(rec.reqs) == k + 1)
        check('  ... z = EKF z, not GNSS altitude', abs(last_z(rec) - EKF_Z) < 1e-9)

        # 3) bridge live pose (was z = 0.0)
        n = make_node(nt)
        n._bridge = {'x': 0.3, 'y': 0.1}
        k = len(rec.reqs)
        n.set_bridge_live_pose()
        check('set_bridge_live_pose: one set_pose sent', len(rec.reqs) == k + 1)
        check('  ... z = EKF z, no jump to 0', abs(last_z(rec) - EKF_Z) < 1e-9)

        # 4) rewind handoff
        n = make_node(nt)
        n._rewind_enable = True
        n._pose_buffer = [object()]
        n._last_rewind_t = 0.0
        n._rewind_cooldown_s = 30.0
        n._gps_bad_since = None
        n._rewind_verify_s = 3.0
        n._rewind_s = 2.0
        n._rewind_max_m = 3.0
        n._rewind_verify_anchor = lambda t: ((0.0, (0.5, 0.0), (0.0, 0.0), 0.0, 0.0), True)
        k = len(rec.reqs)
        real_get_time = rospy.get_time
        rospy.get_time = lambda: 1000.0
        try:
            n._do_rewind_handoff(1000.0)
        finally:
            rospy.get_time = real_get_time
        check('_do_rewind_handoff: one set_pose sent', len(rec.reqs) == k + 1)
        check('  ... z = EKF z', abs(last_z(rec) - EKF_Z) < 1e-9)

        # 5) manual/external set_pose (topic + service): frame 'map', z passed
        #    through unchanged; robot_localization moves it into odom with
        #    map->odom (z = -odom->base z) -> lands on the EKF's own z.
        def rl_odom_z(req, map_odom_z):
            """robot_localization setPoseCallback: pose (frame map) -> world
            frame odom = inverse(map->odom) . pose; translation-only z part."""
            return req.pose.pose.pose.position.z - map_odom_z

        n = make_node(nt)
        n.is_indoor = False
        mo = TransformStamped()
        mo.transform.rotation.w = 1.0
        n.map_odom_transform = mo
        n.map_odom_yaw = 0.0
        n._map_odom_lock = threading.RLock()
        ob = TransformStamped()
        ob.transform.translation.z = EKF_Z
        ob.transform.rotation.w = 1.0
        map_odom_z = n.transform_map_to_odom(rospy.Time(1), ob).transform.translation.z
        msg = PoseWithCovarianceStamped()
        msg.pose.pose.position.x = 2.0
        msg.pose.pose.orientation.w = 1.0
        n.callback_set_pose(msg)
        req = rec.reqs[-1][1]
        check('callback_set_pose: sent in frame map, z unchanged (0)',
              req.pose.header.frame_id == 'map' and last_z(rec) == 0.0)
        check('  ... after the map->odom transform the EKF z is kept (no +z step)',
              abs(rl_odom_z(req, map_odom_z) - EKF_Z) < 1e-9)
        check('  ... set_nav_pose got the same pose (x)', n.set_pose_pose.position.x == 2.0)
        sreq = nt.SetPoseRequest()
        sreq.pose.pose.pose.orientation.w = 1.0
        n.srv_cb_set_pose(sreq)
        check('srv_cb_set_pose: z unchanged, EKF z kept after transform',
              last_z(rec) == 0.0 and abs(rl_odom_z(rec.reqs[-1][1], map_odom_z) - EKF_Z) < 1e-9)

        # 6) very first start: no EKF message yet -> fresh Odometry() -> 0
        n = make_node(nt)
        n.ekf_wheel_nav_odom_msg = Odometry()
        n._bridge = {'x': 0.0, 'y': 0.0}
        n.set_bridge_live_pose()
        check('first start (no EKF msg yet): z = 0', last_z(rec) == 0.0)

        # 7) no set_pose ever carries the GNSS altitude
        check('no set_pose request carried the GNSS z',
              all(abs(r.pose.pose.pose.position.z - GNSS_Z) > 1e-6 for _, r in rec.reqs))
    finally:
        rospy.wait_for_service, rospy.ServiceProxy = saved

    print('map->odom z (one-tick spike):')
    n = object.__new__(nt.NavTfNode)
    n.is_indoor = False
    mo = TransformStamped()
    mo.transform.rotation.w = 1.0
    n.map_odom_transform = mo
    n.map_odom_yaw = 0.0
    n._map_odom_lock = threading.RLock()
    stale = TransformStamped()
    stale.transform.translation.z = 0.013     # previous tick (TF buffer)
    stale.transform.rotation.w = 1.0
    n.get_base_link_transform = lambda stamp=None: stale
    cur = TransformStamped()
    cur.transform.translation.z = -0.303      # this tick's odom->base
    cur.transform.rotation.w = 1.0
    t = n.transform_map_to_odom(rospy.Time(1), cur)
    check('with this tick odom->base: map->odom.z = -odom->base.z',
          abs(t.transform.translation.z - 0.303) < 1e-9)
    t = n.transform_map_to_odom(rospy.Time(1))
    check('without it: falls back to the TF lookup (unchanged)',
          abs(t.transform.translation.z + 0.013) < 1e-9)

    # loop30hz end-to-end: the broadcast pair is z-consistent
    class Bc(object):
        def __init__(self):
            self.calls = []

        def sendTransform(self, tl):
            self.calls.append(tl)

    class Pub(object):
        def publish(self, m):
            pass
    n.tf2_TransformBroadcaster = Bc()
    n.odometry_odom_pub = Pub()
    n.ekf_wheel_nav_odom_msg = odom(1.0, 0.0, -0.303)
    n.nav_last_fused_time = rospy.Time(1000)
    n.fix_last_fused_time = rospy.Time(1000)
    n._bridge_compute_enable = False
    n.publish_odom_for_gps = lambda: None
    n.init_navsat = lambda: None
    n.dock_anchor_hold = lambda: None
    n.slew_map_pose = lambda: None
    try:
        n.loop30hz()
    finally:
        rospy.Time.now = real_now
    tl = n.tf2_TransformBroadcaster.calls[0]
    check('loop30hz: map->odom.z + odom->base.z == 0 in the same message',
          abs(tl[0].transform.translation.z + tl[1].transform.translation.z) < 1e-9)

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
