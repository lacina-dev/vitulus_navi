#!/usr/bin/env python3
"""T36 E6 (~heading_target_smooth): the GNSS heading offset (diff_yaw) follows
the circular mean of the verified window, a share dt / tau per sample, instead
of jumping onto the newest sample of the window.

Built samples through the heading anchor of the node (see _t36_replay.py),
with the parameter off and on:
  - standing, sample noise 0.5 deg: the offset wanders by under 0.5 deg
    instead of over 2 deg and no longer steps by more than 0.3 deg; the same
    at the sample rate with heading_imu_interp on (5 Hz) and off (2.5 Hz),
  - turning: every sample goes the old way - diff_yaw is EQUAL to the run
    with the parameter off (no added delay),
  - after a turn the mean takes only the samples taken after it (the offset
    of before the turn is not mixed in),
  - an offset that moved by more than heading_target_smooth_max_deg and a
    hand-over (map_odom_yaw != 0) go the old way; a smaller move is walked
    (the price: 90 % of it ~12 s after the change instead of 2.5 s),
  - +-pi and the 2*pi hops of the raw target do not disturb it,
and the replay of the recorded drive magtest3 (hand-over at UNDOCK, driving
and turning on GPS), heading_imu_interp off and on. Only this parameter (and,
where said, heading_imu_interp) is switched on.

Run:  python3 test_t36_e6_heading_smooth.py   (needs rospy - robot env)
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

import numpy as np  # noqa: E402

D = math.radians
BASE = {'heading_anchor_spread_max_deg': 6.0}       # as navi_man.launch
ON = dict(BASE, heading_target_smooth=True)
T0 = 7000.0


def noise(n, sigma_deg=0.5, seed=1):
    """Fixed pseudo-noise of one heading sample, clipped at 2.5 sigma."""
    r = np.random.RandomState(seed).normal(0.0, sigma_deg, n)
    return np.radians(np.clip(r, -2.5 * sigma_deg, 2.5 * sigma_deg))


def run(nt, params, samples, diff0=0.3, map_odom_yaw=0.0):
    """samples: [(t, imu_yaw, offset target)] fed to the heading anchor.
    Returns (diff_yaw after every sample [rad], node)."""
    R.CLOCK[0] = T0
    n = R.make_node(nt, params)
    n.init = False
    n.diff_yaw = diff0
    n.map_odom_yaw = map_odom_yaw
    n._set_map_odom_yaw_keep_base = lambda yaw: setattr(n, 'map_odom_yaw', yaw)
    out = []
    for t, imu_yaw, target in samples:
        R.CLOCK[0] = T0 + t
        n.imu_yaw = imu_yaw
        n._heading_anchor_v2(target)
        out.append(n.diff_yaw)
    return np.array(out), n


def wrapd(a):
    return np.degrees(np.arctan2(np.sin(a), np.cos(a)))


def p2p_steps(diff, skip):
    d = wrapd(diff[skip:] - diff[skip])
    return d.max() - d.min(), int((np.abs(wrapd(np.diff(diff[skip:]))) > 0.3).sum())


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-76s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    nt = R.load('navi_transform_t36_e6')

    print('pure functions:')
    f = nt.heading_smooth_step
    r = f(0.30, [0.32, 0.34, 0.36], 0.2, 4.0, D(5.0))
    check('step = dt / tau of the error to the mean of the window',
          abs(r - (0.30 + 0.05 * 0.04)) < 1e-9)
    check('tau <= 0, or dt >= tau: the whole error', abs(f(0.30, [0.34], 0.2, 0.0, 1.0) - 0.34) < 1e-9
          and abs(f(0.30, [0.34], 9.0, 4.0, 1.0) - 0.34) < 1e-9)
    check('dt 0 (first sample): no step', f(0.30, [0.34], 0.0, 4.0, 1.0) == 0.30)
    r = f(D(179.5), [D(179.0), D(-179.0), D(-178.0)], 4.0, 4.0, D(5.0))
    check('circular mean across +-pi (179, -179, -178 -> -179.33 deg)',
          abs(wrapd(r - D(-179.0 - 1.0 / 3.0))) < 1e-3)
    r = f(0.30, [0.34 + 2 * math.pi, 0.34 - 2 * math.pi, 0.34], 4.0, 4.0, D(5.0))
    check('targets 2*pi apart are the same angle; diff_yaw moves the short way',
          abs(r - 0.34) < 1e-9)
    check('mean more than max off diff_yaw: None (the old way)',
          f(0.30, [0.30 + D(1.2)] * 3, 0.2, 4.0, D(1.0)) is None
          and f(0.30, [0.30 + D(0.9)] * 3, 0.2, 4.0, D(1.0)) is not None)
    g = nt.heading_smooth_turning
    check('turning: 2 deg in 0.2 s at a limit of 5 deg/s yes, 0.8 deg no',
          g(0.1, 0.1 + D(2.0), 0.2, D(5.0)) and not g(0.1, 0.1 + D(0.8), 0.2, D(5.0)))
    check('... across +-pi the short way', not g(D(179.8), D(-179.8), 0.2, D(5.0)))
    check('... over a 30 s gap a 3 deg turn counts (limit of a 0.4 s gap)',
          g(0.1, 0.1 + D(3.0), 30.0, D(5.0)) and not g(0.1, 0.1 + D(1.5), 30.0, D(5.0)))

    print('standing 60 s, sample noise 0.5 deg:')
    for label, rate in (('5 Hz (heading_imu_interp on)', 5.0), ('2.5 Hz (off)', 2.5)):
        k = int(60 * rate)
        nz = noise(k)
        smp = [(i / rate, 0.7, 0.3 + nz[i]) for i in range(k)]
        skip = int(6 * rate)                # the window fills in 4.5 s
        off, n_off = run(nt, BASE, smp)
        on, n_on = run(nt, ON, smp)
        p_off, s_off = p2p_steps(off, skip)
        p_on, s_on = p2p_steps(on, skip)
        check('%s: range %.2f -> %.2f deg (expected < 0.5)' % (label, p_off, p_on),
              p_off > 2.0 and p_on < 0.5)
        check('... steps over 0.3 deg: %d -> %d' % (s_off, s_on), s_off > 20 and s_on == 0)
        check('... the mean offset is kept (%.3f deg off the true one)'
              % abs(wrapd(np.mean(on[skip:]) - 0.3)), abs(wrapd(np.mean(on[skip:]) - 0.3)) < 0.15)
        check('... off: no smoothing state is touched', n_off._hs_prev is None
              and len(n_off._hs_turn_ts) == 0 and n_off._hs_n_smooth == 0 and n_off._hs_n_raw == 0)

    print('turning 30 deg/s, the offset drifts 0.5 deg/s, noise 0.5 deg:')
    k = 150
    nz = noise(k, seed=2)
    smp = [(i / 5.0, 0.7 + D(30.0) * i / 5.0, 0.3 + D(0.5) * i / 5.0 + nz[i]) for i in range(k)]
    off, _ = run(nt, BASE, smp)
    on, n_on = run(nt, ON, smp)
    check('every sample the old way: diff_yaw EQUAL to the run with the parameter off',
          np.array_equal(off, on) and n_on._hs_n_smooth == 0 and n_on._hs_n_raw > 100)

    print('standing, a turn of 6 s, standing (the offset is 0.8 deg higher after the turn):')
    smp = []
    for i in range(250):
        t = i / 5.0
        yaw = 0.7 + D(40.0) * min(max(t - 20.0, 0.0), 6.0)
        target = 0.3 + (D(0.8) if t >= 26.0 else D(0.8) * max(t - 20.0, 0.0) / 6.0)
        smp.append((t, yaw, target))
    off, _ = run(nt, BASE, smp)
    on, n_on = run(nt, ON, smp)
    t = np.array([s[0] for s in smp])
    true = np.array([s[2] for s in smp])
    turn = (t > 20.0 + 2.5 + 0.3) & (t < 26.0 + 2.5)     # the window holds turn samples
    check('while the window holds samples of the turn: EQUAL to off',
          np.array_equal(off[turn], on[turn]))
    late = t > 26.0 + 2.5 + 2.0 + 0.5
    check('after it: on the new offset, the old one not mixed in (max %.3f deg off)'
          % np.abs(wrapd(on[late] - true[late])).max(),
          np.abs(wrapd(on[late] - true[late])).max() < 0.02)
    lag_on = np.abs(wrapd(on - true)).max()
    lag_off = np.abs(wrapd(off - true)).max()
    check('largest error of the offset over the whole run: off %.2f, on %.2f deg (not worse)'
          % (lag_off, lag_on), lag_on <= lag_off + 1e-9)

    print('the offset really moves while standing (gyro drift after a freeze):')
    smp = [(i / 5.0, 0.7, 0.3 if i < 100 else 0.3 + D(1.5)) for i in range(200)]
    off, _ = run(nt, BASE, smp)
    on, n_on = run(nt, ON, smp)
    t = np.array([s[0] for s in smp])
    hit = lambda d: t[np.flatnonzero(np.abs(wrapd(d - (0.3 + D(1.5)))) < 0.2)[0]]  # noqa: E731
    check('1.5 deg (> max 1.0): there %.1f s after off (%.1f s / %.1f s after the change)'
          % (hit(on) - hit(off), hit(off) - 20.0, hit(on) - 20.0), hit(on) - hit(off) <= 2.5)
    smp = [(i / 5.0, 0.7, 0.3 if i < 100 else 0.3 + D(0.6)) for i in range(250)]
    on, _ = run(nt, ON, smp)
    off, _ = run(nt, BASE, smp)
    hit2 = lambda d, tt: tt[np.flatnonzero(np.abs(wrapd(d - (0.3 + D(0.6)))) < 0.06)[0]]  # noqa: E731
    t = np.array([s[0] for s in smp])
    e_on = np.abs(wrapd(on - (0.3 + D(0.6))))
    t90 = t[np.flatnonzero(e_on < 0.06)[0]] - 20.0
    check('0.6 deg (< max): walked there without a step over 0.3 deg, 90 %% %.1f s after the '
          'change (off: at once after %.1f s)' % (t90, hit2(off, t) - 20.0),
          np.abs(wrapd(np.diff(on))).max() < 0.3 and e_on[-1] < 0.02 and t90 < 15.0)

    print('hand-over (map_odom_yaw != 0):')
    nz = noise(150, seed=3)
    smp = [(i / 5.0, 0.7, 0.3 + nz[i]) for i in range(150)]
    off, n_off = run(nt, BASE, smp, diff0=0.3 - D(8.0), map_odom_yaw=D(8.0))
    on, n_on = run(nt, ON, smp, diff0=0.3 - D(8.0), map_odom_yaw=D(8.0))
    k_done = n_on._hs_n_raw
    check('the old way until map_odom_yaw is 0: EQUAL to off up to there (%d samples)' % k_done,
          k_done > 20 and np.array_equal(off[:k_done + 10], on[:k_done + 10])
          and n_on.map_odom_yaw == 0.0 and n_off.map_odom_yaw == 0.0)
    check('... smoothed after it', n_on._hs_n_smooth > 30)

    print('raw target hopping by 2*pi (rtk or imu yaw passing 0), around +-pi:')
    nz = noise(200, seed=4)
    smp = [(i / 5.0, 0.7, math.pi + nz[i] + (2 * math.pi if i % 7 == 0 else 0.0)) for i in range(200)]
    on, _ = run(nt, ON, smp, diff0=math.pi)
    p, s = p2p_steps(on, 30)
    check('range %.2f deg, steps over 0.3 deg: %d' % (p, s), p < 0.5 and s == 0)

    print('recorded drive magtest3, replayed (off / on, heading_imu_interp off and on):')
    data = R.load_data()
    for d2 in (False, True):
        tag = 'interp on ' if d2 else 'interp off'
        p_off = {'heading_imu_interp': d2}
        p_on = {'heading_imu_interp': d2, 'heading_target_smooth': True}
        off, n_off = R.replay(nt, data, 85.0, 112.0, params=p_off, pin_until=94.5)
        on, n_on = R.replay(nt, data, 85.0, 112.0, params=p_on, pin_until=94.5)
        h = off['map_odom_yaw'] != 0.0
        k = int(np.flatnonzero(h)[-1]) + 1
        check('%s hand-over at UNDOCK: EQUAL to off until map_odom_yaw is 0 (%.1f s)'
              % (tag, off['t'][k] - 94.5),
              np.array_equal(off['diff_yaw'][:k], on['diff_yaw'][:k])
              and np.array_equal(off['map_odom_yaw'], on['map_odom_yaw']))
        dd = abs(wrapd(np.median(on['diff_yaw'][k:]) - np.median(off['diff_yaw'][k:])))
        check('%s ... the same offset after it (medians %.2f deg apart)' % (tag, dd), dd < 0.3)
        off, n_off = R.replay(nt, data, 531.0, 559.0, params=p_off, pin_until=1e9)
        on, n_on = R.replay(nt, data, 531.0, 559.0, params=p_on, pin_until=1e9)
        u_off = np.degrees(np.unwrap(off['diff_yaw']))
        u_on = np.degrees(np.unwrap(on['diff_yaw']))
        dev = np.abs(wrapd(on['diff_yaw'] - off['diff_yaw']))
        check('%s driving and turning: offset within the range of off +-0.3 deg, median %.2f deg apart'
              % (tag, np.median(dev)),
              u_on.min() > u_off.min() - 0.3 and u_on.max() < u_off.max() + 0.3
              and np.median(dev) < 0.5)
        s_off = int((np.abs(np.diff(u_off)) > 0.3).sum())
        s_on = int((np.abs(np.diff(u_on)) > 0.3).sum())
        check('%s ... steps of the offset over 0.3 deg: %d -> %d (smoothed %d, old way %d)'
              % (tag, s_off, s_on, n_on._hs_n_smooth, n_on._hs_n_raw), s_on <= s_off)

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
