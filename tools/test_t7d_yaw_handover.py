#!/usr/bin/env python3
"""Offline test for T7d (2026-11-09): GNSS heading hand-over and the first GPS
georef at UNDOCK must not jump the robot's map pose.

Field 01:09:04-06: map->base jumped 1.29 m and +15.8 deg (3 of 4 drives).
_apply_heading_target zeroed map_odom_yaw (the dock's -15.7 deg) at once while
diff_yaw slewed at 5 deg/s -> map->odom turned about the map ORIGIN with
odom->base ~4.7 m away; then the first georef snapped and zeroed it again.

Scenario: robot stands, map_odom_yaw -15.7 deg (dock), odom->base (2.1, -4.24,
-119 deg); GNSS heading targets arrive at 10 Hz from t=1 s, the georef at 1 Hz
from t=6.6 s (0.16 m real offset). Between two 30 Hz ticks map->base must never
move > 5 cm or turn > 1 deg. The same run on the pre-T7d code shows the jump.

Run:  python3 test_t7d_yaw_handover.py   (needs rospy — robot env / vitulus-dev)
"""
import math
import os
import subprocess
import sys
import tempfile
import threading
import types
import importlib.util
import importlib.machinery

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _ros_env  # noqa: E402
_ros_env.ensure()


def _stub_missing_msgs():
    """Dev container without the robot's full devel space (only module-level imports)."""
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
from geometry_msgs.msg import TransformStamped, PoseStamped  # noqa: E402
from tf.transformations import quaternion_from_euler  # noqa: E402

NAVI_TF = os.path.join(HERE, '..', 'nodes', 'navi_transform')
D = math.radians
CLOCK = [100.0]

OB = (2.1, -4.24)                 # odom->base, field 01:09
ODOM_YAW0 = D(-119.0)
MO0 = D(-15.7)                    # dock's map->odom yaw
P0 = (3.36, -4.53)                # robot in the map (UNDOCK)
DIFF0 = D(40.0)                   # diff_yaw before (arbitrary; imu = odom_yaw - diff)
IMU = ODOM_YAW0 - DIFF0
TARGET = DIFF0 + MO0 + D(0.3)     # GNSS: consistent heading, 0.3 deg real error
GEOREF_OFF = (0.16, 0.0)          # real position difference GPS vs dock


def load(name, path):
    spec = importlib.util.spec_from_loader(
        name, importlib.machinery.SourceFileLoader(name, path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def make_node(nt):
    n = object.__new__(nt.NavTfNode)
    n._map_odom_lock = threading.Lock()
    mo = TransformStamped()
    mo.transform.rotation.w = 1.0
    c, s = math.cos(MO0), math.sin(MO0)
    mo.transform.translation.x = P0[0] - (c * OB[0] - s * OB[1])
    mo.transform.translation.y = P0[1] - (s * OB[0] + c * OB[1])
    n.map_odom_transform = mo
    n.map_odom_yaw = MO0
    n._map_odom_placed = True
    n._georef_max_map_yaw = D(0.5)
    odom = Odometry()
    odom.pose.pose.position.x, odom.pose.pose.position.y = OB
    odom.pose.pose.orientation.w = 1.0
    n.ekf_wheel_nav_odom_msg = odom
    n.diff_yaw = DIFF0
    n.init = False
    n._heading_slew_dps = 5.0
    n._heading_good_after_acq = False
    n._map_pose_target = None
    n._georef_target = None
    n._georef_slew_last_t = None
    n._georef_stall_s = 10.0
    n._georef_stall_ref = None
    n._yaw_level = False
    n._yaw_level_last_t = None
    # georef
    n._map_coords_rx_t = 0.0
    n._gps_stable = lambda: True
    n.position_fix_usable = True
    n._georef_last_t = None
    n._georef_slew_mps = 0.10
    n._georef_kidnap_m = 1.0
    n.launch_navsat = object()
    n.map_coords_msg = None
    for k in ('_budget_path_m', '_budget_rot_deg', '_budget_used_m', '_budget_used_deg'):
        setattr(n, k, 0.0)

    class Buf(object):
        def lookup_transform(self, *a, **k):
            t = TransformStamped()
            t.transform.translation.x, t.transform.translation.y = 493075.0, 5540714.0
            t.transform.rotation.w = 1.0
            return t
    n.tf2_Buffer = Buf()
    return n


def map_pose(n, ekf_yaw=None):
    t = n.map_odom_transform.transform.translation
    c, s = math.cos(n.map_odom_yaw), math.sin(n.map_odom_yaw)
    x = c * OB[0] - s * OB[1] + t.x
    y = s * OB[0] + c * OB[1] + t.y
    # ideal EKF: odom yaw = imu + diff; or the given (lagging) EKF yaw
    yaw = n.map_odom_yaw + ((IMU + n.diff_yaw) if ekf_yaw is None else ekf_yaw)
    return x, y, yaw


def georef_msg():
    m = PoseStamped()
    m.pose.position.x, m.pose.position.y = 493073.71, 5540714.687
    q = quaternion_from_euler(0, 0, D(-0.07))
    m.pose.orientation.x, m.pose.orientation.y, m.pose.orientation.z, m.pose.orientation.w = q
    return m


def simulate(nt, seconds=40.0, target=TARGET, hdg_period=0.1, ekf_tau=0.0, stats=None):
    """30 Hz ticks; heading every hdg_period from 1 s, georef at 1 Hz from 6.6 s.
    ekf_tau > 0: the nav EKF yaw follows imu + diff_yaw with that time constant
    (field: the EKF lags the fused heading). stats: dict, gets 'level_big' = the
    levelling engaged while |map_odom_yaw| > 0.5 deg (the safety net fired)."""
    # georef target (identity rotation): base = ob + p -> p = P0 + off - ob
    nt.georef_map_odom_xy = lambda *a, **k: (P0[0] + GEOREF_OFF[0] - OB[0],
                                             P0[1] + GEOREF_OFF[1] - OB[1])
    n = make_node(nt)
    ekf = IMU + n.diff_yaw
    prev = map_pose(n, ekf)
    worst_d, worst_a, worst_t = 0.0, 0.0, None
    tick = 1.0 / 30.0
    k = 0
    next_hdg, next_geo = 1.0, 6.6
    t = 0.0
    while t < seconds:
        t = k * tick
        CLOCK[0] = 100.0 + t
        if t >= next_hdg:
            noise = D(0.2) * math.sin(7.0 * t)
            n._apply_heading_target(target + noise)
            next_hdg += hdg_period
        if t >= next_geo:
            n.callback_map_coords(georef_msg())
            next_geo += 1.0
        if hasattr(n, 'slew_georef'):
            n.slew_georef()                 # loop30hz
        if hasattr(n, 'slew_yaw_level'):
            n.slew_yaw_level()
        if stats is not None and getattr(n, '_yaw_level', False) and abs(n.map_odom_yaw) > D(0.5):
            stats['level_big'] = True
        fused = IMU + n.diff_yaw
        ekf = fused if ekf_tau <= 0 else ekf + (fused - ekf) * min(1.0, tick / ekf_tau)
        cur = map_pose(n, ekf)
        dd = math.hypot(cur[0] - prev[0], cur[1] - prev[1])
        da = abs(math.atan2(math.sin(cur[2] - prev[2]), math.cos(cur[2] - prev[2])))
        if dd > worst_d:
            worst_d, worst_t = dd, t
        worst_a = max(worst_a, da)
        prev = cur
        k += 1
    return n, worst_d, worst_a, worst_t, map_pose(n, ekf)


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-70s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    rospy.Time.now = staticmethod(lambda: rospy.Time.from_sec(CLOCK[0]))
    rospy.get_time = lambda: CLOCK[0]
    nt = load('navi_transform_t7d', NAVI_TF)

    print('T7d hand-over at UNDOCK (new code):')
    n, wd, wa, wt, end = simulate(nt)
    check('map->base step between ticks <= 5 cm', wd <= 0.05, '(max %.3f m at %.1f s)' % (wd, wt or 0))
    check('map->base turn between ticks <= 1 deg', wa <= D(1.0), '(max %.2f deg)' % math.degrees(wa))
    check('map_odom_yaw reached exactly 0 (GNSS owns the heading)', n.map_odom_yaw == 0.0,
          '(%.4f deg)' % math.degrees(n.map_odom_yaw))
    check('diff_yaw at the GNSS target', abs(n.diff_yaw - TARGET) < D(0.25))
    check('position converged to the GPS offset (0.16 m)',
          math.hypot(end[0] - P0[0] - GEOREF_OFF[0], end[1] - P0[1] - GEOREF_OFF[1]) < 0.02,
          '(%.3f, %.3f)' % (end[0], end[1]))
    check('heading = GNSS heading (0.3 deg real correction)',
          abs(math.atan2(math.sin(end[2] - (IMU + TARGET)), math.cos(end[2] - (IMU + TARGET)))) < D(0.25))

    print('dock heading 15 deg off (real correction): turned smoothly, robot stays put')
    n, wd, wa, wt, end = simulate(nt, target=TARGET + D(15.0))
    check('step <= 5 cm, turn <= 1 deg per tick', wd <= 0.05 and wa <= D(1.0),
          '(%.3f m, %.2f deg)' % (wd, math.degrees(wa)))
    check('... ends on the GNSS heading with map_odom_yaw 0', n.map_odom_yaw == 0.0 and
          abs(math.atan2(math.sin(end[2] - (IMU + TARGET + D(15))), math.cos(end[2] - (IMU + TARGET + D(15))))) < D(0.25))

    print('GNSS already owns the heading (map_odom_yaw 0): unchanged behaviour')
    n = make_node(nt)
    n.map_odom_yaw = 0.0
    t0 = (n.map_odom_transform.transform.translation.x, n.map_odom_transform.transform.translation.y)
    n._apply_heading_target(n.diff_yaw + D(1.5))
    check('small correction applied directly to diff_yaw (as before)', abs(n.diff_yaw - DIFF0 - D(1.5)) < 1e-9)
    check('map_odom_yaw stays 0, translation untouched', n.map_odom_yaw == 0.0 and
          (n.map_odom_transform.transform.translation.x, n.map_odom_transform.transform.translation.y) == t0)
    n._apply_heading_target(n.diff_yaw + D(10.0))
    check('large correction still slews 0.5 deg/sample', abs(n.diff_yaw - DIFF0 - D(2.0)) < 1e-9)

    print('external /nav_tf/set_heading: heading jumps on purpose, position kept')
    n = make_node(nt)
    n.imu_yaw = IMU
    n._ha_buf = []
    n._ha_frozen_since = None
    before = map_pose(n)
    from std_msgs.msg import Float64
    n.callback_set_heading(Float64(data=D(-130.0)))
    after = map_pose(n)
    check('map position unchanged by set_heading', math.hypot(after[0] - before[0], after[1] - before[1]) < 1e-9)
    check('map heading = wanted', abs(math.atan2(math.sin(after[2] - D(-130.0)),
                                                math.cos(after[2] - D(-130.0)))) < 1e-6)

    print('georef gating:')
    nt.georef_map_odom_xy = lambda *a, **k: (5.0, 5.0)
    n = make_node(nt)
    n.callback_map_coords(georef_msg())
    check('waits while map_odom_yaw is still turning (> 0.5 deg)',
          n.map_odom_yaw == MO0 and n._georef_last_t is None)
    print('normal slow hand-over (field 01:42: samples ~3 Hz, 15.7 deg in > 10 s, EKF lag 0.5 s)')
    st = {}
    n, wd, wa, wt, end = simulate(nt, seconds=60.0, hdg_period=0.33, ekf_tau=0.5, stats=st)
    check('no map->base step > 1 deg per cycle', wa <= D(1.0), '(max %.2f deg)' % math.degrees(wa))
    check('no position step > 5 cm', wd <= 0.05, '(max %.3f m)' % wd)
    check('safety net did NOT fire (no levelling while |yaw| > 0.5 deg)', not st.get('level_big'))
    check('hand-over finished (map_odom_yaw 0)', n.map_odom_yaw == 0.0)

    print('stalled hand-over: GPS back, tracker yaw -15.7 deg, no accepted heading')
    nt.georef_map_odom_xy = lambda *a, **k: (P0[0] + 0.3 - OB[0], P0[1] - OB[1])
    n = make_node(nt)
    CLOCK[0] = 1000.0
    before = map_pose(n)
    prev = before
    worst_a = worst_d = 0.0
    level_at = None
    for k in range(30 * 30):                # 30 s at 30 Hz, georef 1 Hz
        CLOCK[0] = 1000.0 + k / 30.0
        if k % 30 == 0:
            n.callback_map_coords(georef_msg())
        n.slew_georef()
        n.slew_yaw_level()
        if level_at is None and n._yaw_level:
            level_at = k / 30.0
        cur = map_pose(n)
        worst_a = max(worst_a, abs(math.atan2(math.sin(cur[2] - prev[2]), math.cos(cur[2] - prev[2]))))
        if n._georef_target is None and n._georef_last_t is None:
            worst_d = max(worst_d, math.hypot(cur[0] - prev[0], cur[1] - prev[1]))
        prev = cur
    check('waits ~10 s before levelling (stall)', level_at is not None and 9.9 <= level_at <= 10.1,
          '(%s s)' % level_at)
    check('levelling smooth: no step > 0.5 deg per cycle', worst_a <= D(0.5), '(max %.2f deg)' % math.degrees(worst_a))
    check('position kept while levelling', worst_d < 1e-6, '(max %.2g m)' % worst_d)
    check('map_odom_yaw 0 and georef released afterwards', n.map_odom_yaw == 0.0 and
          (n._georef_target is not None or n._georef_last_t is not None))
    nt.georef_map_odom_xy = lambda *a, **k: (5.0, 5.0)

    n = make_node(nt)
    n.map_odom_yaw = 0.0
    n._map_odom_placed = False
    n.map_odom_transform.transform.translation.x = 0.0
    n.map_odom_transform.transform.translation.y = 0.0
    n.callback_map_coords(georef_msg())
    t = n.map_odom_transform.transform.translation
    check('first georef on the boot identity still snaps (as before)', (t.x, t.y) == (5.0, 5.0))
    n = make_node(nt)
    n.map_odom_yaw = 0.0
    n.map_odom_transform.transform.translation.x = 4.5
    n.map_odom_transform.transform.translation.y = 5.0
    n.callback_map_coords(georef_msg())
    t = n.map_odom_transform.transform.translation
    check('first georef after the dock placed map->odom: no snap, target kept',
          (t.x, t.y) == (4.5, 5.0) and n._georef_target[:2] == (5.0, 5.0))
    for i in range(31):
        CLOCK[0] += 1.0 / 30.0
        n.slew_georef()
    t = n.map_odom_transform.transform.translation
    check('... walked at 0.1 m/s in the 30 Hz loop (~0.1 m after 1 s)',
          abs(t.x - 4.6) < 0.005 and abs(t.y - 5.0) < 1e-9, '(%.4f, %.3f)' % (t.x, t.y))
    n = make_node(nt)
    n.map_odom_yaw = 0.0
    n.map_odom_transform.transform.translation.x = 2.0
    n.map_odom_transform.transform.translation.y = 5.0
    n.callback_map_coords(georef_msg())
    t = n.map_odom_transform.transform.translation
    check('beyond kidnap (3 m) it snaps', (t.x, t.y) == (5.0, 5.0))

    # pre-T7d code: the same scenario reproduces the field jump
    old = os.environ.get('T7D_OLD_NAVI_TF')
    if old is None:
        try:
            repo = os.path.join(HERE, '..')
            c = subprocess.check_output(['git', '-C', repo, 'log', '-1', '--format=%H',
                                         '--grep=(T7d)', 'HEAD'], stderr=subprocess.DEVNULL).decode().strip()
            ref = (c + '^') if c else 'HEAD'
            src = subprocess.check_output(['git', '-C', repo, 'show', ref + ':nodes/navi_transform'],
                                          stderr=subprocess.DEVNULL)
            fd, old = tempfile.mkstemp(suffix='_navi_transform_old')
            os.write(fd, src)
            os.close(fd)
        except Exception:
            old = None
    if old:
        print('pre-T7d code, same scenario (expected to jump):')
        nto = load('navi_transform_old', old)
        _, wd, wa, wt, _ = simulate(nto)
        print('    old: max step %.2f m at %.1f s, max turn %.1f deg' % (wd, wt or 0, math.degrees(wa)))
        check('old code reproduces the field jump (> 1 m)', wd > 1.0)

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
