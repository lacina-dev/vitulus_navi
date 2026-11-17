#!/usr/bin/env python3
"""T36 P1 (~anchor_stale_reset_s): when GPS takes the position over again after
a long break (docking, undocking), the position anchor no longer starts from
its buffer of before the break.

Found on recorded undocks (2026-11-16/17): the pose buffer and the verified
anchor were never dropped, so for the first anchor_lag_s after the takeover the
anchor target was "GPS point of 2.5-4 min ago + wheel-EKF path since", rotated
by the heading offset of that old moment. The nav EKF was pulled 5-9 cm away
from the GPS, and 2 s later the fresh anchor pulled it back.

  - anchor_stale_reset_limit, anchor_stale_reset_due (pure): the parameter has
    a lower bound of 10 s (a shorter limit would bypass the anchor's lag and
    motion check after a short GPS outage); 0 = off,
  - the node's own loop3sec on a built drive (Drive of the F2 test: wheel EKF
    30 Hz, GPS 5 Hz, the nav EKF modelled from the set_pose calls): GPS owns
    the position for 30 s, does not for 50 s while the robot drives on and the
    heading offset of the nav EKF changes by 1 deg (as a heading hand-over
    does), then owns it again with the robot standing where the GPS says:
    parameter 0 = set_pose calls EQUAL to the node on main (git 84f1f01),
    the EKF is pulled off the GPS and back; parameter 15 = it stays; a break
    shorter than the limit changes nothing; loc_status field only when on.
Only this parameter is switched on.

Run:  python3 test_t36_p1_anchor_stale_reset.py   (needs rospy - robot env)
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402
import test_t36_f2_anchor_mode_report as F2  # noqa: E402

import numpy as np  # noqa: E402

ON = {'anchor_stale_reset_s': 15.0}
BASE_REF = '84f1f01'      # main the branch started from
T_SYS = 7000.0


def built(nt, params, lost=(30.0, 80.0), dur=90.0, turn_deg=1.0):
    """Straight wheel-EKF path with stops (F2.profile). The nav EKF's heading
    offset against the wheel EKF goes 0 -> turn_deg between 40 and 50 s; the
    GPS shows the path the nav EKF then really drives. Returns (drive, truth)
    with truth(tau) -> (x, y)."""
    F2.install()
    dt = 1.0 / 30.0
    k_end = int(dur / dt)
    tk = np.arange(k_end + 1) * dt
    s = np.concatenate([[0.0], np.cumsum([F2.profile(k * dt) * dt for k in range(k_end)])])
    phi = np.radians(turn_deg) * np.clip((tk - 40.0) / 10.0, 0.0, 1.0)
    ds = np.diff(s)
    tx = np.concatenate([[0.0], np.cumsum(ds * np.cos(phi[1:]))])
    ty = np.concatenate([[0.0], np.cumsum(ds * np.sin(phi[1:]))])
    ip = lambda a, tau: float(np.interp(tau, tk, a))
    truth = lambda tau: (ip(tx, tau), ip(ty, tau))
    ev = [(k * dt + 0.006, 0, 'w', k * dt) for k in range(k_end)]
    ev += [(0.013 + k * 0.2 + 0.03, 1, 'f', 0.013 + k * 0.2) for k in range(int(dur * 5))]
    ev += [(0.013 + k * 0.2 + 0.08, 2, 'g', 0.013 + k * 0.2) for k in range(int(dur * 5))]
    ev += [(0.031 + k * 0.1, 3, 't', None) for k in range(int(dur * 10))]
    ev.sort()
    noise = np.random.RandomState(3).normal(0.0, 0.004, (int(dur * 5) + 1, 2))
    d = F2.Drive(nt, params, T_SYS, truth(0.0))
    for tau, _p, kind, tm in ev:
        if kind == 'w':
            d.wheel(T_SYS + tau, T_SYS + tm, ip(s, tm), 0.0, 0.0, ip(phi, tm))
        elif kind == 'f':
            d.fix(T_SYS + tau, T_SYS + tm)
        elif kind == 'g':
            gx, gy = truth(tm)
            nz = noise[int(round((tm - 0.013) / 0.2))]
            d.gps(T_SYS + tau, T_SYS + tm, gx + float(nz[0]), gy + float(nz[1]))
        else:
            d.owned = not (lost[0] <= tau < lost[1])
            d.tick(T_SYS + tau)
    return d, truth


def off_gps(d, truth, a, b):
    """Largest distance of a set_pose call from the true position in <a, b> s."""
    out = [math.hypot(x - truth(t - T_SYS)[0], y - truth(t - T_SYS)[1])
           for t, x, y in d.calls if a <= t - T_SYS <= b]
    return (max(out) if out else 0.0), len(out)


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-76s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    F2.install()
    nt = R.load('navi_transform_t36_p1')
    due = nt.anchor_stale_reset_due
    lim = nt.anchor_stale_reset_limit
    print('anchor_stale_reset_limit (pure):')
    check('0, negative, NaN, not a number: off', lim(0.0) == 0.0 and lim(-5) == 0.0
          and lim(float('nan')) == 0.0 and lim('x') == 0.0 and lim(None) == 0.0)
    check('3 s and 0.5 s are raised to the lower bound 10 s', lim(3) == 10.0 and lim(0.5) == 10.0)
    check('10, 15, 600 s are taken as they are', lim(10) == 10.0 and lim(15.0) == 15.0
          and lim(600) == 600.0)
    R.CLOCK[0] = T_SYS
    check('node: ~anchor_stale_reset_s 3 -> 10 s in use; unset -> off',
          R.make_node(nt, {'anchor_stale_reset_s': 3.0})._asr_s == 10.0
          and R.make_node(nt, {})._asr_s == 0.0)
    print('anchor_stale_reset_due (pure):')
    check('limit 0: never', not due(100.0, 5000.0, 0.0))
    check('nothing buffered yet: never', not due(None, 5000.0, 15.0))
    check('newest entry 14.9 s old, limit 15: kept', not due(100.0, 114.9, 15.0))
    check('newest entry 15.1 s old, limit 15: dropped', due(100.0, 115.1, 15.0))
    check('newest entry in the future (clock offset): kept', not due(100.7, 100.0, 15.0))

    print('built drive through loop3sec (GPS owns 0-30 s and from 80 s, 90 s in all):')
    off, truth = built(nt, {})
    on, _ = built(nt, ON)
    ref = R.load_ref('navi_transform_p1_ref', BASE_REF)
    if ref is not None:
        r, _ = built(ref, {})
        check('off: set_pose calls EQUAL to main %s (%d calls)' % (BASE_REF, len(off.calls)),
              off.calls == r.calls and len(off.calls) > 20)
    else:
        print('  (git %s not available - comparison with main skipped)' % BASE_REF)
    pre_off = [c for c in off.calls if c[0] - T_SYS < 80.0]
    pre_on = [c for c in on.calls if c[0] - T_SYS < 80.0]
    check('on: set_pose calls before the break and during it EQUAL to off', pre_on == pre_off)
    m_off, n_off = off_gps(off, truth, 80.0, 83.0)
    m_on, n_on = off_gps(on, truth, 80.0, 83.0)
    print('    first 3 s after the takeover: off %d calls, up to %.1f cm off the GPS; '
          'on %d calls, up to %.1f cm' % (n_off, 100 * m_off, n_on, 100 * m_on))
    check('off: the EKF is pulled over 10 cm off the GPS (the stale anchor)', m_off > 0.10)
    check('on: no set_pose further than 2 cm from the GPS', m_on < 0.02)
    check('on: one reset counted, loc_status says so',
          on.n._asr_count == 1 and on.loc[-1][1].get('anchor_stale_resets') == 1)
    check('off: no reset, loc_status has no such field',
          off.n._asr_count == 0 and not any('anchor_stale_resets' in j for _, j in off.loc))
    e_off, _ = off_gps(off, truth, 86.0, 90.0)
    e_on, _ = off_gps(on, truth, 86.0, 90.0)
    check('6 s after the takeover both follow the GPS (within 3 cm)', e_off < 0.03 and e_on < 0.03,
          '(%.1f / %.1f cm)' % (100 * e_off, 100 * e_on))
    check('on: the anchor is the continuous one again at the end',
          on.n._anchor_verified is not None and on.n._anchor_verified[0] > T_SYS + 84.0)
    s_off, _ = built(nt, {}, lost=(30.0, 40.0), dur=50.0)
    s_on, _ = built(nt, ON, lost=(30.0, 40.0), dur=50.0)
    check('break of 10 s (under the limit): no reset, set_pose calls EQUAL to off',
          s_on.n._asr_count == 0 and s_on.calls == s_off.calls and len(s_on.calls) > 10)
    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
