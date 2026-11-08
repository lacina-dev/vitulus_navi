#!/usr/bin/env python3
"""Offline test for T28-carr (2026-11-08): the dual-antenna heading is accepted
only with a fresh NAV-RELPOSNED carrSoln == 2 (FIXED) + relPosHeadingValid
from /gnss_heading/navrelposned. Float/none, invalid heading or a stale
RELPOSNED -> rejected. No RELPOSNED ever received or gate disabled -> the old
behaviour. Band, delay and fix gates unchanged.

Drives the real methods on a stub node (no ROS master, no node init).
Run:  python3 test_t28_carr_gate.py   (needs rospy — robot env / vitulus-dev)
"""
import os
import sys
import types
import importlib.util
import importlib.machinery

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _ros_env  # noqa: E402
_ros_env.ensure()


def _stub_missing_msgs():
    """Dev container without the robot's full devel space: sensor_msgs_ext and
    the generated vitulus_navi.srv are missing (only imported at module level)."""
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
from sensor_msgs.msg import Imu  # noqa: E402
from ublox_msgs.msg import NavRELPOSNED9  # noqa: E402

NAVI_TF = os.path.join(HERE, '..', 'nodes', 'navi_transform')
T0 = rospy.Time(1000)
FIXED_VALID = 311       # live robot 2026-11-08 (docked): gnssFixOK|diffSoln|relPosValid|FIXED|isMoving|headValid


def flags(carr, head_valid=True):
    f = NavRELPOSNED9.FLAGS_GNSS_FIX_OK | NavRELPOSNED9.FLAGS_DIFF_SOLN | NavRELPOSNED9.FLAGS_REL_POS_VALID
    f |= {0: 0, 1: NavRELPOSNED9.FLAGS_CARR_SOLN_FLOAT, 2: NavRELPOSNED9.FLAGS_CARR_SOLN_FIXED}[carr]
    if head_valid:
        f |= NavRELPOSNED9.FLAGS_REL_POS_HEAD_VALID
    return f


def load(name, path):
    spec = importlib.util.spec_from_loader(
        name, importlib.machinery.SourceFileLoader(name, path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


CLOCK = [500.0]


def make_node(nt):
    n = object.__new__(nt.NavTfNode)
    n._heading_var_min = 0.00043
    n._heading_var_max = 0.00079
    n._heading_require_relpos = True
    n._relpos_max_age_s = 0.3
    n._relpos_flags = None
    n._relpos_rx_t = None
    n.fix_usable = True
    n.heading_fix_usable = True
    imu = Imu()
    imu.header.stamp = T0
    imu.orientation.w = 1.0
    n.imu_msg = imu
    n.nav_last_fused_time = 0
    n.init = True
    n._ha_enable = True
    n.diff_yaw = 0.0
    n._heading_slew_dps = 5.0
    n.map_odom_yaw = 0.0
    n._heading_good_after_acq = False
    return n


def relpos(n, f):
    m = NavRELPOSNED9()
    m.flags = f
    n.callback_relposned(m)


def heading(var=0.00055):
    m = Imu()
    m.header.stamp = T0
    m.orientation.z, m.orientation.w = 0.3826834, 0.9238795     # 45 deg
    m.orientation_covariance[8] = var
    return m


def accepted(n):
    return n.nav_last_fused_time != 0


def main():
    nt = load('navi_transform_t28c', NAVI_TF)
    fails = []

    def check(name, cond):
        print('  %-66s %s' % (name, 'OK' if cond else 'FAIL'))
        if not cond:
            fails.append(name)

    check('flags helper matches live robot value 311 (FIXED + headValid)',
          flags(2) | NavRELPOSNED9.FLAGS_IS_MOVING == FIXED_VALID)
    real_now, real_gt = rospy.Time.now, rospy.get_time
    rospy.Time.now = staticmethod(lambda: rospy.Time(2000))
    rospy.get_time = lambda: CLOCK[0]
    try:
        print('RELPOSNED gate:')
        n = make_node(nt)
        relpos(n, flags(2))
        CLOCK[0] += 0.05
        n.offset_navheading_imu(heading())
        check('FIXED + headingValid, fresh -> accepted (and fused)',
              accepted(n) and n._heading_good_after_acq)

        for name, f in (('FLOAT', flags(1)), ('NONE', flags(0)),
                        ('FIXED but heading invalid', flags(2, False))):
            n = make_node(nt)
            relpos(n, f)
            CLOCK[0] += 0.05
            n.offset_navheading_imu(heading())
            check('%s -> rejected' % name, not accepted(n) and n.diff_yaw == 0.0)

        n = make_node(nt)
        relpos(n, flags(2))
        CLOCK[0] += 0.5
        n.offset_navheading_imu(heading())
        check('stale RELPOSNED (0.5 s > 0.3 s) -> rejected', not accepted(n))

        n = make_node(nt)
        relpos(n, flags(2))
        CLOCK[0] += 0.25
        n.offset_navheading_imu(heading())
        check('previous epoch (0.25 s, 5 Hz race) -> accepted', accepted(n))

        n = make_node(nt)
        CLOCK[0] += 1.0
        n.offset_navheading_imu(heading())
        check('no RELPOSNED ever received -> old behaviour (accepted)', accepted(n))

        n = make_node(nt)
        n._heading_require_relpos = False
        relpos(n, flags(1))
        CLOCK[0] += 0.05
        n.offset_navheading_imu(heading())
        check('gate disabled by param -> FLOAT accepted as before', accepted(n))

        print('unchanged gates:')
        n = make_node(nt)
        relpos(n, flags(2))
        n.offset_navheading_imu(heading(0.00034))
        check('FIXED but var below band -> still rejected', not accepted(n))
        n = make_node(nt)
        relpos(n, flags(2))
        n.heading_fix_usable = False
        n.offset_navheading_imu(heading())
        check('FIXED but heading_fix_usable False -> still rejected', not accepted(n))
    finally:
        rospy.Time.now, rospy.get_time = real_now, real_gt

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
