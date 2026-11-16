#!/usr/bin/env python3
"""T36 G1 (~heading_latency_s, 0 = off): a GNSS heading sample is stamped when
it arrives, but the receiver measured it earlier, so against the IMU it lags
(recorded drives: ~45 ms against the IMU messages as published). While the
robot turns, the heading target is then off by rate x lag (0.9 rad/s -> 2.3
deg). With the parameter the sample is paired with the IMU yaw that many
seconds BEFORE its stamp. Works only together with heading_imu_interp (D2).

Checks on built samples (turning robot, heading measured 45 ms before its
stamp) and replays the recorded hand-over at UNDOCK (robot standing: nothing
may change). Only this parameter is switched on (above D2, which is on in
navi_man.launch).

Run:  python3 test_t36_g1_heading_latency.py   (needs rospy - robot env)
"""
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

import numpy as np  # noqa: E402

D2 = {'heading_imu_interp': True}
G1 = {'heading_imu_interp': True, 'heading_latency_s': 0.045}
D = math.radians
FIXED = 0x100 | (2 << 3) | 0x7


def gated_node(nt, params, t):
    R.CLOCK[0] = t
    n = R.make_node(nt, params)
    n.init = False
    n.fix_usable = n.heading_fix_usable = True
    n.callback_relposned(types.SimpleNamespace(flags=FIXED))
    got = []
    n._heading_anchor_v2 = got.append
    return n, got


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-70s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    nt = R.load('navi_transform_t36_g1')
    t0 = 7000.0

    def target(rtk, imu_yaw):
        return (rtk % (2 * math.pi)) - (imu_yaw % (2 * math.pi)) + 1.5708

    def turn(params, w, lag=0.045, n_imu=32, k_h=(8, 13, 19, 24), phase=0.004):
        """Robot turning at w rad/s, true heading offset 0.9 rad. IMU every
        10 ms (yaw = 0.2 + w t), heading samples stamped `phase` after the
        IMU message k and measured `lag` earlier (all within 0.3 s: the
        RELPOSNED flags of the start stay fresh). Returns errors of the
        targets against the true offset [rad] and the node."""
        n, got = gated_node(nt, params, t0)
        hs = {k: t0 + 0.01 * k + phase for k in k_h}
        for k in range(n_imu):
            st = t0 + 0.01 * k
            R.CLOCK[0] = st
            n.callback_imu(R.imu_msg(st, 0.2 + w * 0.01 * k, w))
            if k in hs:
                R.CLOCK[0] = hs[k]
                rtk = 0.9 + 0.2 + w * (hs[k] - t0 - lag)     # heading of the moment measured
                n.callback_navheading(R.navheading_msg(hs[k], rtk, 1.35))
        true_t = target(0.9 + 0.2, 0.2)
        return [R.wrap(g - true_t) for g in got], n

    print('turning robot, heading measured 45 ms before its stamp:')
    e_d2, _ = turn(D2, 0.9)
    e_g1, n = turn(G1, 0.9)
    check('D2 alone: target off by rate x lag (0.9 rad/s x 45 ms = 2.32 deg): %.2f deg'
          % math.degrees(np.mean(e_d2)), len(e_d2) == 4 and
          all(abs(math.degrees(e) + 2.32) < 0.02 for e in e_d2))
    check('with heading_latency_s 0.045: on the true offset (%.3f deg)'
          % math.degrees(np.max(np.abs(e_g1))), len(e_g1) == 4 and
          all(abs(e) < D(0.01) for e in e_g1))
    check('... every sample paired, none dropped', n._hi_used == 4 and n._hi_dropped == 0)
    e_neg, _ = turn(G1, -0.9)
    check('turning the other way: also on the true offset',
          len(e_neg) == 4 and all(abs(e) < D(0.01) for e in e_neg))
    e_half, _ = turn(G1, 0.9, lag=0.065)
    check('real lag 65 ms, parameter 45 ms: the rest stays (0.9 x 20 ms = 1.03 deg): %.2f'
          % math.degrees(np.mean(e_half)), all(abs(math.degrees(e) + 1.03) < 0.02 for e in e_half))
    e_st_d2, _ = turn(D2, 0.0)
    e_st_g1, _ = turn(G1, 0.0)
    check('robot not turning: same target with and without',
          len(e_st_g1) == 4 and all(abs(a - b) < 1e-9 for a, b in zip(e_st_d2, e_st_g1)))

    print('switch and limits:')
    e_off, n = turn({'heading_imu_interp': True, 'heading_latency_s': 0.0}, 0.9)
    check('heading_latency_s 0 (default): exactly as D2', n._hi_latency_s == 0.0 and
          all(abs(a - b) < 1e-12 for a, b in zip(e_off, e_d2)))
    n, _ = gated_node(nt, D2, t0)
    check('parameter absent: 0', n._hi_latency_s == 0.0)
    n, _ = gated_node(nt, {'heading_imu_interp': True, 'heading_latency_s': -1.0}, t0)
    n2, _ = gated_node(nt, {'heading_imu_interp': True, 'heading_latency_s': 5.0}, t0)
    check('clamped to 0..0.2 s', n._hi_latency_s == 0.0 and n2._hi_latency_s == 0.2)
    # without D2 the old pairing (last IMU message, 6 ms gate) is untouched
    n, got = gated_node(nt, {'heading_latency_s': 0.045}, t0)
    for k in range(6):
        R.CLOCK[0] = t0 + 0.01 * k
        n.callback_imu(R.imu_msg(t0 + 0.01 * k, 0.2 + 0.9 * 0.01 * k, 0.9))
    R.CLOCK[0] = t0 + 0.054
    n.callback_navheading(R.navheading_msg(t0 + 0.054, 1.0, 1.35))
    check('without heading_imu_interp nothing changes (last IMU message, no history)',
          len(got) == 1 and abs(R.wrap(got[0] - target(1.0, 0.2 + 0.9 * 0.05))) < 1e-6
          and len(n._hi_hist) == 0)
    # the IMU history (32 messages = 0.3 s) covers the largest allowed shift
    e_max, n = turn({'heading_imu_interp': True, 'heading_latency_s': 0.2}, 0.9, lag=0.2,
                    k_h=(21, 23, 25))
    check('largest shift 0.2 s still inside the IMU history: paired (%d), on the offset'
          % n._hi_used, n._hi_used == 3 and all(abs(e) < D(0.01) for e in e_max))
    e_early, n = turn({'heading_imu_interp': True, 'heading_latency_s': 0.2}, 0.9, lag=0.2,
                      k_h=(5,))
    check('shifted before the first IMU message: dropped, nothing applied',
          e_early == [] and n._hi_dropped == 1)

    print('recorded hand-over at UNDOCK (robot standing, 85-112 s): nothing changes')
    data = R.load_data()
    a, n_a = R.replay(nt, data, 85.0, 112.0, params=D2, pin_until=94.5)
    b, n_b = R.replay(nt, data, 85.0, 112.0, params=G1, pin_until=94.5)
    dd = abs(math.degrees(R.wrap(np.median(b['diff_yaw'][b['t'] > 110.0])
                                 - np.median(a['diff_yaw'][a['t'] > 110.0]))))
    check('same heading offset after it (%.3f deg), map_odom_yaw exactly 0 in both' % dd,
          dd < 0.05 and a['map_odom_yaw'][-1] == 0.0 and b['map_odom_yaw'][-1] == 0.0)
    check('same number of samples used (%d, %d), none dropped (%d)'
          % (n_a._hi_used, n_b._hi_used, n_b._hi_dropped),
          abs(n_a._hi_used - n_b._hi_used) <= 1 and n_b._hi_dropped == 0)

    print('driving on GPS, turning (531-559 s): heading offset inside the same range')
    a2, _ = R.replay(nt, data, 531.0, 559.0, params=D2, pin_until=1e9)
    b2, n2 = R.replay(nt, data, 531.0, 559.0, params=G1, pin_until=1e9)
    ua, ub = np.degrees(np.unwrap(a2['diff_yaw'])), np.degrees(np.unwrap(b2['diff_yaw']))
    check('diff_yaw range with %.1f..%.1f deg, without %.1f..%.1f deg (+-0.5), applied %d / %d'
          % (ub.min(), ub.max(), ua.min(), ua.max(), len(b2['hdg_t']), len(a2['hdg_t'])),
          ub.min() > ua.min() - 0.5 and ub.max() < ua.max() + 0.5
          and len(b2['hdg_t']) >= 0.8 * len(a2['hdg_t']))

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
