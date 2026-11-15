#!/usr/bin/env python3
"""T36 D1 (~odom_twist_from_wheels): /odometry/odom carries the measured twist
(linear.x of the wheel odometry, angular.z of the IMU gyro) instead of the
twist of the nav EKF, which every set_pose of the GPS anchor zeroes.

Replays two driving windows of the recorded drive magtest3 (2026-11-14) through
the node (see _t36_replay.py) with the parameter off and on:
  - on: pose, TF, heading state are EQUAL to off; only the twist differs,
  - the twist is the latest wheel linear.x / gyro z, with the sign of reversing,
  - the share of ticks reporting v = 0 while the wheels turn, and of ticks
    reporting less than half of the real turn rate, drops to 0,
  - a missing / stale / non-finite source falls back to the EKF message itself,
  - the EKF message kept by the node is never modified.
Only this parameter is switched on.

Run:  python3 test_t36_d1_odom_twist.py   (needs rospy - robot env)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

import numpy as np  # noqa: E402
import rospy  # noqa: E402

ON = {'odom_twist_from_wheels': True}
WINDOWS = (('reversing', 315.0, 333.0), ('turning', 531.0, 559.0))


def at_ticks(data, key, col, ticks):
    """Value of the latest message of data[key] at each tick time (bag time)."""
    arr = data[key]
    i = np.searchsorted(arr[:, 0], data['T0'] + ticks, side='right') - 1
    return arr[i, col]


def shares(tr, wheel_v, gyro):
    mov = np.abs(wheel_v) > 0.05
    turn = np.abs(gyro) > 0.2
    zero = float((tr['od_v'][mov] == 0.0).mean()) if mov.any() else 0.0
    low = float((np.abs(tr['od_w'][turn]) < 0.5 * np.abs(gyro[turn])).mean()) if turn.any() else 0.0
    return zero, low, int(mov.sum()), int(turn.sum())


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-66s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    data = R.load_data()
    nt = R.load('navi_transform_t36_d1')
    tot = {'off': [0, 0, 0, 0], 'on': [0, 0, 0, 0]}
    for name, t0, t1 in WINDOWS:
        print('driving on GPS, %s (%.0f-%.0f s):' % (name, t0, t1))
        off, n_off = R.replay(nt, data, t0, t1, pin_until=1e9)
        on, n_on = R.replay(nt, data, t0, t1, params=ON, pin_until=1e9)
        same = [k for k in sorted(off) if k not in ('od_v', 'od_w')
                and not np.array_equal(off[k], on[k])]
        check('on: pose, TF, heading state equal to off (only the twist differs)',
              not same, ('differs: %s' % ', '.join(same)) if same else '')
        wv = at_ticks(data, 'wo', 6, on['t'])
        gz = at_ticks(data, 'imu', 3, on['t'])
        fed = on['t'] > t0 + 0.5          # the first ticks: sources not fed yet (fallback)
        check('on: linear.x = latest wheel odometry linear.x',
              np.array_equal(on['od_v'][fed], wv[fed]))
        check('on: angular.z = latest IMU gyro z', np.array_equal(on['od_w'][fed], gz[fed]))
        check('off: twist is the EKF twist (as before)',
              np.array_equal(off['od_v'], at_ticks(data, 'ekf', 6, off['t']))
              and np.array_equal(off['od_w'], at_ticks(data, 'ekf', 7, off['t'])))
        if name == 'reversing':
            rev = wv < -0.1
            check('reversing: linear.x negative with the wheels (%d ticks, min %.2f m/s)'
                  % (rev.sum(), on['od_v'].min()), rev.sum() > 50 and (on['od_v'][rev] < -0.1).all())
        for label, tr in (('off', off), ('on', on)):
            z, lo, nm, ntn = shares(tr, wv, gz)
            tot[label][0] += z * nm
            tot[label][1] += nm
            tot[label][2] += lo * ntn
            tot[label][3] += ntn
            print('    %-3s v = 0 while the wheels turn: %5.1f %% of %d ticks; '
                  '|w| < half the gyro while turning: %5.1f %% of %d' % (label, 100 * z, nm, 100 * lo, ntn))
    z_off, z_on = tot['off'][0] / tot['off'][1], tot['on'][0] / tot['on'][1]
    l_off, l_on = tot['off'][2] / max(tot['off'][3], 1), tot['on'][2] / max(tot['on'][3], 1)
    print('both windows:')
    check('v = 0 while driving: %.1f %% -> %.1f %% (expected ~0)' % (100 * z_off, 100 * z_on),
          z_off > 0.02 and z_on == 0.0)
    check('|w| under half the real rate while turning: %.1f %% -> %.1f %% (expected < 2)'
          % (100 * l_off, 100 * l_on), l_off > 0.2 and l_on < 0.02)

    print('fallbacks and side effects:')
    R.CLOCK[0] = 5000.0
    n = R.make_node(nt, ON)
    ekf = R.odom_msg(5000.0, 1.0, 2.0, 0.3, 0.5, 0.0, 0.0)
    ekf.pose.covariance = [float(i) for i in range(36)]
    ekf.twist.covariance = [float(-i) for i in range(36)]
    ekf.twist.twist.linear.y, ekf.twist.twist.angular.x = 0.011, 0.022
    n.callback_ekf_wheel_nav_odom(ekf)
    stamp = rospy.Time.from_sec(5000.0)

    def pub(wheel_age=0.01, imu_age=0.01, v=-0.31, w=0.42):
        n.callback_wheel_odom(R.odom_msg(5000.0 - wheel_age, 0, 0, 0, 0, v, 9.9))
        n.imu_msg = R.imu_msg(5000.0 - imu_age, 0.0, w)
        tf = n.transform_odom_to_base_link(stamp)
        return n.odometry_odom_pub.last, tf

    out, tf = pub()
    check('fresh sources: a copy with twist (-0.31 m/s, 0.42 rad/s)',
          out is not ekf and out.twist.twist.linear.x == -0.31 and out.twist.twist.angular.z == 0.42)
    check('... header, frames and pose are the EKF message\'s',
          out.header is ekf.header and out.pose is ekf.pose
          and out.header.frame_id == 'odom' and out.child_frame_id == 'base_link')
    check('... both covariances and the other twist components unchanged',
          list(out.pose.covariance) == list(ekf.pose.covariance)
          and list(out.twist.covariance) == list(ekf.twist.covariance)
          and out.twist.twist.linear.y == 0.011 and out.twist.twist.angular.x == 0.022)
    check('... the EKF message kept by the node is not modified',
          ekf.twist.twist.linear.x == 0.0 and ekf.twist.twist.angular.z == 0.0
          and n.ekf_wheel_nav_odom_msg is ekf)
    check('... odom->base_link TF is the EKF pose', tf.transform.translation.x == 1.0
          and tf.transform.translation.y == 2.0 and tf.transform.translation.z == 0.3)
    check('wheel odometry 0.19 s old: still used', pub(wheel_age=0.19)[0] is not ekf)
    check('wheel odometry 0.25 s old: the EKF message itself', pub(wheel_age=0.25)[0] is ekf)
    check('IMU 0.25 s old: the EKF message itself', pub(imu_age=0.25)[0] is ekf)
    check('non-finite wheel speed / gyro: the EKF message itself',
          pub(v=float('nan'))[0] is ekf and pub(w=float('inf'))[0] is ekf)
    n2 = R.make_node(nt, ON)
    n2.callback_ekf_wheel_nav_odom(ekf)
    n2.transform_odom_to_base_link(stamp)
    check('no wheel odometry / IMU received yet: the EKF message itself',
          n2.odometry_odom_pub.last is ekf)
    n3 = R.make_node(nt, None)
    n3.callback_ekf_wheel_nav_odom(ekf)
    n3.callback_wheel_odom(R.odom_msg(5000.0, 0, 0, 0, 0, 0.3, 0.0))
    n3.imu_msg = R.imu_msg(5000.0, 0.0, 0.4)
    n3.transform_odom_to_base_link(stamp)
    check('parameter off (default): the EKF message itself, as before',
          n3._odom_twist_wheels is False and n3.odometry_odom_pub.last is ekf)
    n4 = object.__new__(nt.NavTfNode)          # stub nodes of the older tests
    n4.is_indoor = False
    n4.ekf_wheel_nav_odom_msg = ekf
    n4.odometry_odom_pub = R.FakePub()
    n4.transform_odom_to_base_link(stamp)
    check('node built without __init__ (older tests): off', n4.odometry_odom_pub.last is ekf)

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
