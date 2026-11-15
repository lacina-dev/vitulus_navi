#!/usr/bin/env python3
"""T36 D2 (~heading_imu_interp): the IMU yaw a GNSS heading sample is paired
with is interpolated at the sample's stamp instead of taking the last IMU
message and throwing the sample away when that one is more than 6 ms older.

Replays the GNSS heading hand-over of the recorded drive magtest3 (2026-11-14,
robot standing after UNDOCK, map->odom yaw -15.7 deg handed over to GNSS)
through the node (see _t36_replay.py) with the parameter off and on:
  - share of the gate-passing samples that are used: ~half -> over 95 %,
  - the hand-over takes under 8 s instead of ~12-15 s and ends the same way
    (map_odom_yaw exactly 0, same diff_yaw),
  - the carrSoln / fix / variance gates decide exactly as before,
and checks the pairing itself on built samples (interpolation across +-pi,
samples without an IMU message within 50 ms on both sides are dropped, a
turning robot). Only this parameter is switched on.

Run:  python3 test_t36_d2_heading_interp.py   (needs rospy - robot env)
"""
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

import numpy as np  # noqa: E402

ON = {'heading_imu_interp': True}
D = math.radians
FIXED = 0x100 | (2 << 3) | 0x7        # relPosHeadingValid + carrSoln fixed
FLOAT = 0x100 | (1 << 3) | 0x7


def count_v2(counter):
    """setup hook: count the samples that reach the heading anchor."""
    def setup(n):
        real = n._heading_anchor_v2

        def counted(target):
            counter.append(target)
            return real(target)
        n._heading_anchor_v2 = counted
    return setup


def handover_span(tr, t_from):
    """(first applied sample after t_from, time map_odom_yaw reached 0)."""
    first = tr['hdg_t'][tr['hdg_t'] >= t_from][0]
    done = tr['t'][np.flatnonzero(tr['map_odom_yaw'] == 0.0)[0]]
    return first, done


def gated_node(nt, params=None, t=7000.0):
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

    data = R.load_data()
    nt = R.load('navi_transform_t36_d2')

    print('GNSS heading hand-over at UNDOCK, replayed (85-112 s):')
    c_off, c_on = [], []
    off, n_off = R.replay(nt, data, 85.0, 112.0, pin_until=94.5, setup=count_v2(c_off))
    on, n_on = R.replay(nt, data, 85.0, 112.0, params=ON, pin_until=94.5, setup=count_v2(c_on))
    gated = n_on._hi_used + n_on._hi_dropped       # passed every gate but the pairing
    u_off, u_on = len(c_off) / float(gated), len(c_on) / float(gated)
    check('samples used: %.0f %% -> %.0f %% of the %d that pass the other gates (expected > 95)'
          % (100 * u_off, 100 * u_on, gated), u_off < 0.75 and u_on > 0.95)
    check('... none dropped for a missing IMU neighbour (%d)' % n_on._hi_dropped,
          n_on._hi_dropped == 0 and n_on._hi_used == len(c_on))
    f_off, d_off = handover_span(off, 94.5)
    f_on, d_on = handover_span(on, 94.5)
    check('hand-over of %.1f deg: %.1f s -> %.1f s (expected < 8)'
          % (abs(math.degrees(off['map_odom_yaw'][0])), d_off - f_off, d_on - f_on),
          (d_on - f_on) < 8.0 and (d_off - f_off) > 10.0)
    check('both end with map_odom_yaw exactly 0', off['map_odom_yaw'][-1] == 0.0
          and on['map_odom_yaw'][-1] == 0.0)
    dd = abs(math.degrees(R.wrap(np.median(on['diff_yaw'][on['t'] > 110.0])
                                 - np.median(off['diff_yaw'][off['t'] > 110.0]))))
    check('... and the same heading offset (median diff_yaw after it within 0.2 deg: %.2f)' % dd,
          dd < 0.2)
    step = np.abs(np.degrees(np.diff(on['diff_yaw'])))
    check('diff_yaw still moves <= 0.5 deg per sample during the hand-over (max %.2f)'
          % step[on['map_odom_yaw'][1:] != 0.0].max(),
          step[on['map_odom_yaw'][1:] != 0.0].max() <= 0.5 + 1e-9)
    # the samples the old pairing took give the same target (robot standing)
    check('off: the IMU history is not kept (no cost in the IMU callback)',
          len(n_off._hi_hist) == 0 and n_off._hi_used == 0 and n_off._hi_pending is None)

    # while driving diff_yaw itself wanders 1-2.5 deg between verified samples
    # (off as well), so the two runs are compared by their range, not tick by tick
    print('driving on GPS, turning (531-559 s): the heading offset stays where it was')
    off2, _ = R.replay(nt, data, 531.0, 559.0, pin_until=1e9)
    on2, n2 = R.replay(nt, data, 531.0, 559.0, params=ON, pin_until=1e9)
    dev = np.abs(np.degrees(np.arctan2(np.sin(on2['diff_yaw'] - off2['diff_yaw']),
                                       np.cos(on2['diff_yaw'] - off2['diff_yaw']))))
    u_off, u_on = np.degrees(np.unwrap(off2['diff_yaw'])), np.degrees(np.unwrap(on2['diff_yaw']))
    check('diff_yaw on vs off: median difference %.2f deg (applied %d -> %d samples)'
          % (np.median(dev), len(off2['hdg_t']), len(on2['hdg_t'])),
          np.median(dev) < 0.5 and len(on2['hdg_t']) > len(off2['hdg_t']))
    check('... range on %.1f..%.1f deg inside the range off %.1f..%.1f deg (+-0.5)'
          % (u_on.min(), u_on.max(), u_off.min(), u_off.max()),
          u_on.min() > u_off.min() - 0.5 and u_on.max() < u_off.max() + 0.5)

    print('pairing:')
    t0 = 7000.0

    def run(imu, hdg_t, hdg_yaw, late_imu=(), params=ON, flags=FIXED, acc_deg=1.35, fix=True):
        """imu / late_imu: [(stamp, yaw)] fed before / after the heading sample."""
        n, got = gated_node(nt, params, t0)
        n.fix_usable = fix
        n.callback_relposned(types.SimpleNamespace(flags=flags))
        for st, yaw in imu:
            R.CLOCK[0] = st
            n.callback_imu(R.imu_msg(st, yaw, 0.0))
        R.CLOCK[0] = hdg_t
        n.callback_navheading(R.navheading_msg(hdg_t, hdg_yaw, acc_deg))
        before = len(got)
        for st, yaw in late_imu:
            R.CLOCK[0] = st
            n.callback_imu(R.imu_msg(st, yaw, 0.0))
        return n, got, before

    def target(rtk, imu_yaw):
        return (rtk % (2 * math.pi)) - (imu_yaw % (2 * math.pi)) + 1.5708

    def close(a, b, tol=1e-6):          # stamps are rounded to ns
        return abs(R.wrap(a - b)) < tol

    n, got, before = run([(t0, 0.10), (t0 + 0.010, 0.12)], t0 + 0.017, 1.0, [(t0 + 0.020, 0.14)])
    check('waits for the IMU message after the sample, then pairs',
          before == 0 and len(got) == 1 and n._hi_pending is None)
    check('... with the yaw interpolated at the stamp (0.7 of the way)',
          len(got) == 1 and close(got[0], target(1.0, 0.12 + 0.7 * 0.02)))
    n, got, before = run([(t0, 0.10), (t0 + 0.010, 0.12), (t0 + 0.020, 0.14)], t0 + 0.017, 1.0)
    check('IMU message after the stamp already there: paired at once',
          before == 1 and close(got[0], target(1.0, 0.134)))
    n, got, _ = run([(t0, D(179.0))], t0 + 0.005, 0.3, [(t0 + 0.010, D(-179.0))])
    check('interpolation across +-pi goes the short way (179, -179 -> 180 deg)',
          len(got) == 1 and close(got[0], target(0.3, math.pi)))
    n, got, _ = run([(t0, 0.5)], t0, 0.3, [(t0 + 0.010, 0.6)])
    check('stamp exactly on an IMU message: that yaw', len(got) == 1 and close(got[0], target(0.3, 0.5)))
    n, got, _ = run([(t0, 0.1)], t0 + 0.060, 1.0, [(t0 + 0.065, 0.1)])
    check('last IMU message 60 ms before the sample: dropped',
          got == [] and n._hi_dropped == 1 and n._hi_pending is None)
    n, got, _ = run([(t0, 0.1)], t0 + 0.005, 1.0, [(t0 + 0.070, 0.1)])
    check('next IMU message 65 ms after the sample: dropped', got == [] and n._hi_dropped == 1)
    n, got, _ = run([], t0 + 0.005, 1.0, [(t0 + 0.010, 0.1), (t0 + 0.020, 0.1)])
    check('no IMU message before the sample: dropped', got == [] and n._hi_dropped == 1)
    n, got, before = run([(t0, 0.1)], t0 + 0.005, 1.0)
    check('IMU silent after the sample: nothing applied, sample waits', got == [] and
          n._hi_pending is not None)
    R.CLOCK[0] = t0 + 0.230
    n.callback_navheading(R.navheading_msg(t0 + 0.230, 1.0, 1.35))
    n.callback_imu(R.imu_msg(t0 + 0.232, 0.1, 0.0))
    check('... the next sample replaces it; a stale pair is never applied',
          got == [] and n._hi_dropped == 2 and n._hi_pending is None)

    # turning at 1 rad/s: IMU every 10 ms, heading stamped 7 ms after an IMU message
    w = 1.0
    imu = [(t0 + 0.01 * k, 0.2 + w * 0.01 * k) for k in range(6)]
    th = t0 + 0.047
    true_t = target(0.9, 0.2 + w * 0.047)
    n, got, _ = run(imu[:5], th, 0.9, imu[5:])
    e_on = abs(R.wrap(got[0] - true_t)) if got else 9.0
    n, got_off, _ = run(imu[:5], th, 0.9, imu[5:], params=None)
    check('turning 1 rad/s, sample 7 ms after an IMU message: on %.3f deg off the true offset'
          % math.degrees(e_on), e_on < D(0.01))
    check('... off: not taken at all (7 ms > 6 ms)', got_off == [])
    th = t0 + 0.045
    n, got_off, _ = run(imu[:5], th, 0.9, imu[5:], params=None)
    e_off = abs(R.wrap(got_off[0] - target(0.9, 0.2 + w * 0.045))) if got_off else 9.0
    n, got, _ = run(imu[:5], th, 0.9, imu[5:])
    e_on = abs(R.wrap(got[0] - target(0.9, 0.2 + w * 0.045))) if got else 9.0
    check('... 5 ms after: off takes it with the 5 ms old yaw (%.2f deg off), on %.3f deg'
          % (math.degrees(e_off), math.degrees(e_on)), D(0.25) < e_off < D(0.32) and e_on < D(0.01))

    print('the other gates decide as before (on):')
    imu = [(t0, 0.1), (t0 + 0.010, 0.1)]
    late = [(t0 + 0.020, 0.1)]
    for label, kw in (('carrSoln FLOAT', dict(flags=FLOAT)), ('fix not usable', dict(fix=False)),
                      ('yaw variance too tight (0.5 deg)', dict(acc_deg=0.5)),
                      ('yaw variance too loose (3 deg)', dict(acc_deg=3.0))):
        n, got, _ = run(imu, t0 + 0.015, 1.0, late, **kw)
        check('%s: rejected, nothing waits' % label, got == [] and n._hi_pending is None
              and n._hi_used == 0 and n._hi_dropped == 0)
    n, got, _ = run(imu, t0 + 0.015, 1.0, late)
    check('same sample with every gate open: used', len(got) == 1)
    n, got, _ = run(imu, t0 + 0.015, 1.0, late)
    n2_, _ = gated_node(nt, ON, t0)
    n2_.init = True
    applied = []
    n2_._apply_heading_target = applied.append
    for st, yaw in imu:
        n2_.callback_imu(R.imu_msg(st, yaw, 0.0))
    n2_.callback_navheading(R.navheading_msg(t0 + 0.015, 1.0, 1.35))
    n2_.callback_imu(R.imu_msg(t0 + 0.020, 0.1, 0.0))
    check('first-ever anchor (init) still goes straight to _apply_heading_target',
          len(applied) == 1 and close(applied[0], target(1.0, 0.1)))

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
