#!/usr/bin/env python3
"""T36 P2 (~takeover_carry): the set_pose steps with which the position anchor
pulls the nav EKF to the GPS after a GPS takeover do not move the robot's map
pose - each is taken off map->odom in the tick that broadcasts the stepped
pose. (Recorded undocks 2026-11-16/17: the map pose travelled 19-39 cm in two
or three jerks for a real difference of 3-14 cm.)

  - takeover_carry_start / _note / _take / _fields (pure),
  - the node's own loop3sec and loop30hz on a built drive (as the P1 test; the
    nav EKF is modelled from the set_pose calls and stamps its output with the
    time of the last set_pose, as robot_localization does): GPS owns the
    position for 30 s, not for 50 s, then again - the odom frame has drifted
    12 cm against the GPS meanwhile and the stale anchor (P1 left off) first
    pulls the EKF 19 cm the other way:
    off = set_pose calls and broadcast transforms EQUAL to main (git 84f1f01);
    on = set_pose calls EQUAL to off, the map pose does not move, map->odom
    takes the steps; with map->odom turned 10 deg the same; a step over the
    limit, the total limit, a short break; GPS back in the middle of a drive
    (the dock did not hold the pose in the break) and a georef that cannot
    run (no site datum / no utm->odom): nothing carried; the georef then
    walks the rest at its 0.10 m/s.
Only this parameter is switched on.

Run:  python3 test_t36_p2_takeover_carry.py   (needs rospy - robot env)
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402
import test_t36_f2_anchor_mode_report as F2  # noqa: E402

import numpy as np  # noqa: E402

ON = {'takeover_carry': True}
BASE_REF = '84f1f01'
T_SYS = 7000.0
DRIFT = (0.10, -0.066)      # GPS against the odom frame after the break [m]
MO0 = (3.0, -1.5)           # map->odom translation the dock left
REAL = (0.08, 0.0)          # real error of the dock pose against the GPS [m] (georef case)


class Drive(F2.Drive):
    """F2.Drive + the EKF's output stamp (never before the last set_pose) and
    the 30 Hz broadcast."""

    def __init__(self, *a, **k):
        F2.Drive.__init__(self, *a, **k)
        self.t_set = 0.0
        self.tf = []           # (tau, mo_x, mo_y, mo_yaw, ob_x, ob_y)

    def _sink(self, req):
        p = req.pose.pose.pose.position
        self.calls.append((round(R.CLOCK[0], 4), round(p.x, 9), round(p.y, 9)))
        self.xy = [p.x, p.y]
        self.t_set = R.CLOCK[0]      # robot_localization: output stamped from now on

    def _feed_nav(self):
        self.n.callback_ekf_wheel_nav_odom(R.odom_msg(
            max(self._stamp, self.t_set), self.xy[0], self.xy[1], 0.0, self._nav_yaw, 0.0, 0.0))

    def tick30(self, t_sys):
        R.CLOCK[0] = t_sys
        self.n.loop30hz()
        sent = self.n.tf2_TransformBroadcaster.last
        m = [x for x in sent if x.child_frame_id == 'odom'][0].transform
        o = [x for x in sent if x.child_frame_id == 'base_link'][0].transform
        self.tf.append((t_sys - T_SYS, m.translation.x, m.translation.y, R.quat_yaw(m.rotation),
                        o.translation.x, o.translation.y))


def built(nt, params, lost=(30.0, 80.0), dur=90.0, mo_yaw=0.0, georef_at=None,
          dock=True, datum=True, utm_odom=True):
    """The drive of the P1 test with the 30 Hz loop; after the break the GPS
    is DRIFT away from where the odom frame has the robot."""
    F2.install()
    dt = 1.0 / 30.0
    k_end = int(dur / dt)
    tk = np.arange(k_end + 1) * dt
    s = np.concatenate([[0.0], np.cumsum([F2.profile(k * dt) * dt for k in range(k_end)])])
    phi = np.radians(1.0) * np.clip((tk - 40.0) / 10.0, 0.0, 1.0)
    ds = np.diff(s)
    tx = np.concatenate([[0.0], np.cumsum(ds * np.cos(phi[1:]))])
    ty = np.concatenate([[0.0], np.cumsum(ds * np.sin(phi[1:]))])
    ip = lambda a, tau: float(np.interp(tau, tk, a))
    ev = [(k * dt + 0.006, 0, 'w', k * dt) for k in range(k_end)]
    ev += [(k * dt + 0.011, 4, 'b', None) for k in range(k_end)]
    ev += [(0.013 + k * 0.2 + 0.03, 1, 'f', 0.013 + k * 0.2) for k in range(int(dur * 5))]
    ev += [(0.013 + k * 0.2 + 0.08, 2, 'g', 0.013 + k * 0.2) for k in range(int(dur * 5))]
    ev += [(0.031 + k * 0.1, 3, 't', None) for k in range(int(dur * 10))]
    ev.sort()
    noise = np.random.RandomState(3).normal(0.0, 0.004, (int(dur * 5) + 1, 2))
    d = Drive(nt, params, T_SYS, (0.0, 0.0))
    n = d.n
    n.map_odom_transform.transform.translation.x = MO0[0]
    n.map_odom_transform.transform.translation.y = MO0[1]
    n.map_odom_yaw = mo_yaw
    n._map_odom_placed = True
    n._anchor_lost_t = T_SYS - 1.0      # the start is not a takeover (GPS owned it 1 s ago)
    if dock:                            # the dock held the pose during the break
        n._map_pose_last_accept['dock'] = T_SYS + 60.0
    if datum:
        n._site_datum = object()        # a georef source exists
    if utm_odom:
        n.tf2_Buffer.lookup_transform = lambda *a, **k: None    # navsat's utm->odom is there
    for tau, _p, kind, tm in ev:
        if kind == 'w':
            d.wheel(T_SYS + tau, T_SYS + tm, ip(s, tm), 0.0, 0.0, ip(phi, tm))
        elif kind == 'b':
            if georef_at is not None and abs(tau - georef_at) < 0.5 * dt:
                # what callback_map_coords does once map->odom is level: the target
                n._georef_target = (MO0[0] - DRIFT[0] + REAL[0], MO0[1] - DRIFT[1] + REAL[1], 0.0)
            d.tick30(T_SYS + tau)
        elif kind == 'f':
            d.fix(T_SYS + tau, T_SYS + tm)
        elif kind == 'g':
            off = DRIFT if tm >= lost[0] else (0.0, 0.0)
            nz = noise[int(round((tm - 0.013) / 0.2))]
            d.gps(T_SYS + tau, T_SYS + tm, ip(tx, tm) + off[0] + float(nz[0]),
                  ip(ty, tm) + off[1] + float(nz[1]))
        else:
            d.owned = not (lost[0] <= tau < lost[1])
            d.tick(T_SYS + tau)
    return d


def map_pose(d, a, b):
    """Map position of the robot per broadcast in <a, b>: arrays t, x, y."""
    tf = np.array([r for r in d.tf if a <= r[0] <= b])
    c, s = np.cos(tf[:, 3]), np.sin(tf[:, 3])
    return tf[:, 0], tf[:, 1] + c * tf[:, 4] - s * tf[:, 5], tf[:, 2] + s * tf[:, 4] + c * tf[:, 5]


def travel(d, a, b):
    """Path and net shift of the map pose in <a, b> (robot stands there) [m]."""
    _, x, y = map_pose(d, a, b)
    st = np.hypot(np.diff(x), np.diff(y))
    return float(st.sum()), float(math.hypot(x[-1] - x[0], y[-1] - y[0])), float(st.max())


def run_pure(nt, check):
    new, start, note, take, fields = (nt.takeover_carry_new_state, nt.takeover_carry_start,
                                      nt.takeover_carry_note, nt.takeover_carry_take,
                                      nt.takeover_carry_fields)
    st = new()
    check('no window: a step is not queued, no fields active',
          note(st, 100.0, 100.0, (0, 0), (0.05, 0)) == 'inactive'
          and not fields(st, 100.0)['takeover_carry_active'])
    check('break of 3 s (< 5): no window', not start(st, 100.0, 97.0))
    check('first takeover ever (never lost): window', start(st, 100.0, None))
    check('a running window is not opened again', not start(st, 101.0, None))
    check('step 5 cm queued', note(st, 100.1, 100.1, (1.0, 2.0), (1.05, 2.0)) == '')
    check('EKF message stamped before the set_pose: nothing taken yet',
          take(st, 100.11, 100.09, (1.0, 2.0)) == (0.0, 0.0) and len(st['pending']) == 1)
    d = take(st, 100.14, 100.1, (1.05, 2.0))
    check('message stamped at the set_pose: the step is taken once',
          abs(d[0] - 0.05) < 1e-12 and d[1] == 0.0 and not st['pending']
          and take(st, 100.17, 100.13, (1.05, 2.0)) == (0.0, 0.0))
    note(st, 100.2, 100.2, (1.05, 2.0), (1.05, 2.04))
    check('no usable stamp, 50 ms after the call: waits',
          take(st, 100.25, None, (1.05, 2.04)) == (0.0, 0.0))
    d = take(st, 100.28, None, (1.05, 2.035))
    check('no usable stamp, 80 ms after, pose nearer to the stepped one: taken',
          abs(d[1] - 0.04) < 1e-12)
    note(st, 100.3, 100.3, (1.05, 2.04), (1.08, 2.04), None, 100.31)
    check('call 100.300, answer 100.310, message stamped 100.305 with the old pose: not yet',
          take(st, 100.32, 100.305, (1.05, 2.04)) == (0.0, 0.0) and len(st['pending']) == 1)
    d = take(st, 100.33, 100.305, (1.08, 2.04))
    check('... the same stamp with the stepped pose: taken',
          abs(d[0] - 0.03) < 1e-12 and not st['pending'])
    note(st, 100.34, 100.34, (1.08, 2.04), (1.11, 2.04), None, 100.35)
    d = take(st, 100.36, 100.35, (1.08, 2.04))
    check('message stamped at the answer: taken whatever the pose', abs(d[0] - 0.03) < 1e-12)
    note(st, 100.4, 100.4, (1.11, 2.04), (1.14, 2.04))
    take(st, 100.5, None, (1.11, 2.04))
    check('a step the EKF never shows is dropped after 0.5 s (counted)',
          take(st, 100.95, None, (1.11, 2.04)) == (0.0, 0.0) and not st['pending']
          and st['dropped'] == 1)
    f = fields(st, 100.97)
    check('fields: active, 4 steps, 11.7 cm',
          f['takeover_carry_active'] and f['takeover_carry_n'] == 4
          and f['takeover_carry_m'] == 0.117 and f['takeover_carry_stopped'] == '')
    check('step of 7 cm (> 6): not carried, the window ends',
          note(st, 101.0, 101.0, (0, 0), (0.07, 0)) == 'step'
          and note(st, 101.1, 101.1, (0, 0), (0.02, 0)) == 'inactive'
          and fields(st, 101.2)['takeover_carry_stopped'] == 'step')
    check('after the window (4 s): inactive', not fields(st, 104.5)['takeover_carry_active'])
    check('next takeover after a break of 60 s: a new window, sums cleared',
          start(st, 200.0, 140.0) and st['n'] == 0 and st['sum'] == [0.0, 0.0]
          and st['stopped'] == '' and st['windows'] == 2)
    st2 = new()
    check('GPS back without the dock having held the pose (mid-drive): no window',
          not start(st2, 300.0, 200.0, None, False, True) and st2['windows'] == 0
          and st2['no_georef'] == 0)
    check('after the dock but the georef cannot run: no window, counted',
          not start(st2, 300.0, 200.0, None, True, False) and st2['no_georef'] == 1
          and fields(st2, 300.0)['takeover_carry_no_georef'] == 1)
    check('after the dock with the georef ready: window', start(st2, 300.0, 200.0, None, True, True))
    st3 = new()
    check('EKF 2 cm from the GPS at the takeover (under 3): nothing to carry, no window',
          not start(st3, 400.0, None, None, True, True, 0.02) and st3['windows'] == 0)
    check('EKF 12 cm from the GPS: window', start(st3, 400.0, None, None, True, True, 0.12))
    r = [note(st3, 400.1, 400.1, (0, 0), (0.05, 0), None, None, 0.07),
         note(st3, 400.2, 400.2, (0.05, 0), (0.10, 0), None, None, 0.02),
         note(st3, 400.3, 400.3, (0.10, 0), (0.115, 0), None, None, 0.01)]
    check('the step that brings the EKF within 3 cm of the GPS is carried and ends the window',
          r == ['', '', 'inactive'] and len(st3['pending']) == 2
          and not fields(st3, 400.25)['takeover_carry_active'])
    cfg = dict(nt.TAKEOVER_CARRY_CFG, total_max_m=0.12)
    r = [note(st, 200.0 + 0.1 * k, 200.0 + 0.1 * k, (0.05 * k, 0), (0.05 * k + 0.05, 0), cfg)
         for k in range(4)]
    check('total limit 12 cm: two steps of 5 cm queued, the third ends the window',
          r == ['', '', 'total', 'inactive'])


def run_built(nt, check):
    off = built(nt, {})
    on = built(nt, ON)
    ref = R.load_ref('navi_transform_p2_ref', BASE_REF)
    if ref is not None:
        r = built(ref, {})
        check('off: set_pose calls EQUAL to main %s (%d calls)' % (BASE_REF, len(off.calls)),
              off.calls == r.calls and len(off.calls) > 20)
        check('off: every broadcast transform EQUAL to main (%d ticks)' % len(off.tf),
              off.tf == r.tf and len(off.tf) > 2000)
    else:
        print('  (git %s not available - comparison with main skipped)' % BASE_REF)
    check('off: loc_status has no takeover_carry field',
          not any(k.startswith('takeover_carry') for _, j in off.loc for k in j))
    check('on: set_pose calls EQUAL to off (the EKF is driven as before)', on.calls == off.calls)
    pre_on = [r_ for r_ in on.tf if r_[0] < 80.0]
    pre_off = [r_ for r_ in off.tf if r_[0] < 80.0]
    check('on: broadcast transforms before the takeover EQUAL to off', pre_on == pre_off)
    p_off, n_off, s_off = travel(off, 79.9, 83.0)
    p_on, n_on, s_on = travel(on, 79.9, 83.0)
    print('    map pose in the 3 s after the takeover (robot stands): off path %.1f cm, net '
          '%.1f cm, largest tick %.1f cm; on path %.2f cm, net %.2f cm, largest tick %.2f cm'
          % (100 * p_off, 100 * n_off, 100 * s_off, 100 * p_on, 100 * n_on, 100 * s_on))
    check('off: the map pose travels over 30 cm for a net 12 cm', p_off > 0.30 and 0.10 < n_off < 0.14)
    check('on: the map pose does not move (path under 1 mm)', p_on < 0.001)
    mo = np.array([r_[1:3] for r_ in on.tf if 82.9 < r_[0] < 83.0])[-1]
    ob_on = np.array([r_[4:6] for r_ in on.tf if 82.9 < r_[0] < 83.0])[-1]
    ob_off = np.array([r_[4:6] for r_ in off.tf if 82.9 < r_[0] < 83.0])[-1]
    check('on: odom->base ends where it does with off (at the GPS)',
          float(np.hypot(*(ob_on - ob_off))) < 1e-9)
    check('on: map->odom took the 12 cm (opposite to the EKF)',
          abs(mo[0] - (MO0[0] - DRIFT[0])) < 0.012 and abs(mo[1] - (MO0[1] - DRIFT[1])) < 0.012,
          '(%.3f, %.3f)' % (mo[0] - MO0[0], mo[1] - MO0[1]))
    j = on.loc[-1][1]
    check('on: loc_status - one window, steps counted, about 0.12 m, nothing stopped or dropped',
          j['takeover_carry_windows'] == 1 and j['takeover_carry_n'] >= 6
          and abs(j['takeover_carry_m'] - 0.12) < 0.015 and j['takeover_carry_stopped'] == ''
          and j['takeover_carry_dropped'] == 0 and not j['takeover_carry_active'],
          str({k: v for k, v in j.items() if k.startswith('takeover_carry')}))
    t10 = built(nt, ON, mo_yaw=math.radians(10.0))
    p10, _, _ = travel(t10, 79.9, 83.0)
    check('on, map->odom turned 10 deg (heading hand-over running): path under 1 mm',
          p10 < 0.001 and abs(t10.n.map_odom_yaw - math.radians(10.0)) < 1e-12)
    big = dict(ON, pos_slew_mps=1.0)                 # anchor steps of up to 10 cm
    b_on, b_off = built(nt, big), built(nt, {'pos_slew_mps': 1.0})
    check('anchor steps of 10 cm (> 6 cm limit): nothing carried, transforms EQUAL to off, reported',
          b_on.tf == b_off.tf and b_on.loc[-1][1]['takeover_carry_stopped'] == 'step'
          and b_on.loc[-1][1]['takeover_carry_n'] == 0)
    cap = built(nt, dict(ON, takeover_carry_max_m=0.10))
    jc = cap.loc[-1][1]
    check('total limit 10 cm: carried up to it, then stopped and reported',
          jc['takeover_carry_stopped'] == 'total' and 0.05 <= jc['takeover_carry_m'] <= 0.10)
    s_on = built(nt, ON, lost=(30.0, 33.0), dur=45.0)
    s_off = built(nt, {}, lost=(30.0, 33.0), dur=45.0)
    check('break of 3 s: no new window, transforms EQUAL to off',
          s_on.tf == s_off.tf and s_on.loc[-1][1]['takeover_carry_windows'] == 0)
    nd = built(nt, ON, dock=False)
    check('GPS returns in the middle of a drive (no dock in the break): nothing carried, '
          'transforms EQUAL to off',
          nd.tf == off.tf and nd.loc[-1][1]['takeover_carry_windows'] == 0
          and nd.loc[-1][1]['takeover_carry_no_georef'] == 0)
    ng = built(nt, ON, datum=False)
    check('no site datum and no live map georef: nothing carried, transforms EQUAL to off, counted',
          ng.tf == off.tf and ng.loc[-1][1]['takeover_carry_windows'] == 0
          and ng.loc[-1][1]['takeover_carry_no_georef'] == 1)
    nu = built(nt, ON, utm_odom=False)
    check('no utm->odom from navsat: nothing carried, transforms EQUAL to off, counted',
          nu.tf == off.tf and nu.loc[-1][1]['takeover_carry_no_georef'] == 1)
    g = built(nt, ON, georef_at=84.5)
    mo = np.array([r_[1:3] for r_ in g.tf if 84.4 < r_[0] < 86.0])
    st = np.hypot(*np.diff(mo, axis=0).T)
    end = np.array(g.tf[-1][1:3])
    check('then the georef walks map->odom the real 8 cm at its 0.10 m/s (3.4 mm a tick at most)',
          st.max() < 0.0035 and abs(st.sum() - 0.08) < 0.012
          and abs(end[0] - (MO0[0] - DRIFT[0] + REAL[0])) < 1e-9,
          '(largest tick %.1f mm, path %.1f cm)' % (1000 * st.max(), 100 * st.sum()))


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-76s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    F2.install()
    nt = R.load('navi_transform_t36_p2')
    print('takeover_carry_* (pure):')
    run_pure(nt, check)
    print('built drive through loop3sec and loop30hz (GPS owns 0-30 s and from 80 s):')
    run_built(nt, check)
    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
