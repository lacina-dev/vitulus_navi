#!/usr/bin/env python3
"""T36 H1 (~tf_ekf_stamp): odom->base_link and map->odom are broadcast once per
nav-EKF message, when it arrives, stamped with the message - instead of from
the 30 Hz loop, which stamps the last EKF pose with its own time (a pose up to
~45 ms old goes out as "now"; 10 % of the ticks of the recorded drive repeat
the previous pose and as many EKF messages never reach /tf).

Pure part: tf_ekf_stamp_decide / tf_ekf_stamp_loop_due.
Replay of the recorded drive magtest3 (2026-11-14) through the node (see
_t36_replay.py), parameter off and on:
  - off: one broadcast per loop tick, stamped with the loop time (as before),
  - on: one broadcast per EKF message, both transforms carry the stamp of the
    message, odom->base_link is its pose, stamps strictly increase, no pose is
    repeated and no EKF message is lost; /odometry/odom once per message,
  - map->odom, the heading state and the dock hold are the same as off,
  - EKF silent: the loop takes over after ~tf_ekf_stamp_hold_s (loop time,
    last pose) and hands back when the messages return - also when every
    message arrives just after a loop tick with a stamp older than the tick
    (start of the node, both at 30 Hz with a steady phase),
  - the dock engage walk (D4) takes as long as with the parameter off,
  - the set of switches that is live on the robot + this one,
  - repeated / old / future stamps send nothing; indoors nothing is sent.
Only this parameter is switched on, except where the test says otherwise.

Run:  python3 test_t36_h1_tf_ekf_stamp.py   (needs rospy - robot env)
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

import numpy as np  # noqa: E402
import rospy  # noqa: E402

ON = {'tf_ekf_stamp': True}
# what navi_man.launch switches on today (D1-D4, E5, F1, F2, M1, M2, mag check)
LIVE = {'odom_twist_from_wheels': True, 'heading_imu_interp': True,
        'handover_follow_ekf': True, 'dock_engage_slew_mps': 0.10, 'navsat_async': True,
        'gps_stamp_offset_track': True, 'anchor_mode_report': True,
        'mag_check_enable': True, 'mag_check_wide': True, 'rtk_mag_report': True,
        'heading_anchor_spread_max_deg': 6.0}
WINDOWS = (('reversing', 315.0, 333.0), ('turning', 531.0, 559.0))


class Recorder(object):
    """TransformBroadcaster stub that keeps every broadcast:
    (clock, [frames in order], stamp map->odom, stamp odom->base, ob x, y, yaw,
    mo x, y, yaw)."""
    def __init__(self):
        self.last = None
        self.n = 0
        self.rows = []
        self.order = set()

    def sendTransform(self, t):
        t = t if isinstance(t, list) else [t]
        self.last = t
        self.n += 1
        self.order.add(tuple(x.child_frame_id for x in t))
        mo = [x for x in t if x.child_frame_id == 'odom'][0]
        ob = [x for x in t if x.child_frame_id == 'base_link'][0]
        self.rows.append((R.CLOCK[0], mo.header.stamp.to_sec(), ob.header.stamp.to_sec(),
                          ob.transform.translation.x, ob.transform.translation.y,
                          R.quat_yaw(ob.transform.rotation),
                          mo.transform.translation.x, mo.transform.translation.y,
                          R.quat_yaw(mo.transform.rotation)))


def run(nt, data, t0, t1, params=None, drop=None, hook=None, more=None, **kw):
    """Replay with every broadcast and every /odometry/odom kept. drop=(a, b):
    the EKF messages of that time span (s from the start) are not fed."""
    rec = Recorder()
    odom = []

    def setup(n):
        rec.last = n.tf2_TransformBroadcaster.last   # the state fed before the window
        n.tf2_TransformBroadcaster = rec
        n.odometry_odom_pub.keep = odom
        if drop is not None:
            real = n.callback_ekf_wheel_nav_odom

            def gated(msg):
                if not (drop[0] <= R.CLOCK[0] - data['T0'] <= drop[1]):
                    real(msg)
            n.callback_ekf_wheel_nav_odom = gated
        if more is not None:
            more(n)
    tr, n = R.replay(nt, data, t0, t1, params=params, setup=setup, hook=hook, **kw)
    return tr, n, np.array(rec.rows), odom, rec


def fed(data, t0, t1):
    ekf = data['ekf']
    i0, i1 = np.searchsorted(ekf[:, 0], [data['T0'] + t0, data['T0'] + t1])
    return ekf[i0:i1]


def stale_share(rows, data):
    """Share of the broadcasts, while the wheels turn, that repeat the previous
    odom->base_link position (nav_score tf_stale_pct)."""
    wo = data['wo']
    v = np.abs(np.interp(rows[1:, 0], wo[:, 0], wo[:, 6]))
    same = (np.diff(rows[:, 3]) == 0) & (np.diff(rows[:, 4]) == 0)
    mv = v > 0.1
    return float((same & mv).sum()) / max(int(mv.sum()), 1), int(mv.sum())


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-70s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    data = R.load_data()
    nt = R.load('navi_transform_t36_h1')

    print('tf_ekf_stamp_decide / tf_ekf_stamp_loop_due:')
    dec, due = nt.tf_ekf_stamp_decide, nt.tf_ekf_stamp_loop_due
    check('a newer stamp, 8 ms behind the clock: broadcast with it',
          dec(100.000, 99.967, 100.008, 0.1) == 'ekf')
    check('the same stamp again / an older one: nothing (tf2 would drop it)',
          dec(100.0, 100.0, 100.01, 0.1) == 'repeat' and dec(99.9, 100.0, 99.95, 0.1) == 'repeat')
    check('stamp more than max_age behind the clock: nothing',
          dec(100.0, 99.0, 100.11, 0.1) == 'old' and dec(100.0, 99.0, 100.09, 0.1) == 'ekf')
    check('stamp ahead of the clock: nothing', dec(100.01, 99.0, 100.0, 0.1) == 'future')
    check('zero / non-finite stamp or clock: nothing',
          dec(0.0, -1.0, 100.0, 0.1) == 'invalid' and dec(float('nan'), 0.0, 100.0, 0.1) == 'invalid'
          and dec(100.0, 0.0, float('inf'), 0.1) == 'invalid')
    check('loop due: before the first EKF broadcast, after hold_s, clock gone back',
          due(5.0, None, 0.1) and due(5.11, 5.0, 0.1) and due(4.9, 5.0, 0.1)
          and not due(5.05, 5.0, 0.1) and not due(5.1, 5.0, 0.1))

    tot = {'off': [0.0, 0], 'on': [0.0, 0]}
    lost = [0, 0]
    for name, t0, t1 in WINDOWS:
        print('driving on GPS, %s (%.0f-%.0f s):' % (name, t0, t1))
        off, n_off, r_off, od_off, rec_off = run(nt, data, t0, t1, pin_until=1e9)
        on, n_on, r_on, od_on, rec_on = run(nt, data, t0, t1, params=ON, pin_until=1e9)
        msgs = fed(data, t0, t1)
        check('off: one broadcast per loop tick, stamped with the loop time',
              len(r_off) == len(off['t']) and np.array_equal(r_off[:, 1], r_off[:, 0])
              and np.array_equal(r_off[:, 2], r_off[:, 0]))
        r = r_on
        check('on: one broadcast per EKF message (%d), none from the loop' % len(msgs),
              len(r) == len(msgs) and n_on._tes_n['loop'] == 0, 'got %d' % len(r))
        check('on: both transforms carry the stamp of the EKF message',
              np.array_equal(r[:, 1], msgs[:, 1]) and np.array_equal(r[:, 2], msgs[:, 1]))
        check('on: odom->base_link is the pose of that message',
              np.array_equal(r[:, 3], msgs[:, 2]) and np.array_equal(r[:, 4], msgs[:, 3])
              and np.allclose(r[:, 5], msgs[:, 5], atol=1e-12))
        check('on: stamps strictly increase (tf2 drops a repeated stamp)',
              bool((np.diff(r_on[:, 1]) > 0).all()))
        check('one /tf message, map->odom first (TF-ATOMIC), off and on',
              rec_on.order == {('odom', 'base_link')} and rec_off.order == {('odom', 'base_link')})
        lag = r[:, 0] - r[:, 1]
        check('on: stamp behind the clock at the broadcast: median %.1f, max %.1f ms (< 50)'
              % (1e3 * np.median(lag), 1e3 * lag.max()), lag.min() >= 0 and lag.max() < 0.05)
        # off: which EKF message each tick carries -> how old the pose stamped "now" is
        k = np.searchsorted(msgs[:, 0], r_off[:, 0], side='right') - 1
        ok = (k >= 0) & (msgs[np.maximum(k, 0), 2] == r_off[:, 3])
        age = r_off[ok, 0] - msgs[k[ok], 1]
        print('    off: age of the pose that goes out stamped "now": median %.0f, p95 %.0f, '
              'max %.0f ms' % (1e3 * np.median(age), 1e3 * np.percentile(age, 95), 1e3 * age.max()))
        lost[0] += len(msgs) - len(np.unique(k[ok]))
        lost[1] += len(msgs)
        for label, rows in (('off', r_off), ('on', r_on)):
            s, nmv = stale_share(rows, data)
            tot[label][0] += s * nmv
            tot[label][1] += nmv
            print('    %-3s repeated pose while the wheels turn: %5.2f %% of %d broadcasts'
                  % (label, 100 * s, nmv))
        check('on: /odometry/odom once per EKF message, its stamp = the TF stamp',
              len(od_on) == len(msgs)
              and all(m.header.stamp.to_sec() == s for m, s in zip(od_on, msgs[:, 1])))
        check('on: map->odom and the heading state at the loop ticks equal to off',
              all(np.array_equal(off[c], on[c]) for c in ('mo_x', 'mo_y', 'mo_yaw', 'diff_yaw',
                                                           'map_odom_yaw', 'fused_yaw')))
    s_off, s_on = tot['off'][0] / tot['off'][1], tot['on'][0] / tot['on'][1]
    print('both windows:')
    check('repeated pose while driving: %.1f %% -> %.1f %% (expected 0)' % (100 * s_off, 100 * s_on),
          s_off > 0.005 and s_on == 0.0)
    check('EKF messages that never reach /tf: off %.1f %% -> on 0' % (100.0 * lost[0] / lost[1]),
          lost[0] > 0)

    print('with D1 (~odom_twist_from_wheels) on as well:')
    both = dict(ON, odom_twist_from_wheels=True)
    _, n_b, r_b, od_b, _ = run(nt, data, 315.0, 333.0, params=both, pin_until=1e9)
    wo = data['wo']
    kk = np.searchsorted(wo[:, 0], r_b[:, 0], side='right') - 1
    twist = np.array([m.twist.twist.linear.x for m in od_b])
    late = r_b[:, 0] > r_b[0, 0] + 0.5
    check('twist of /odometry/odom is the wheel odometry\'s (D1 works on the EKF stamp)',
          np.array_equal(twist[late], wo[kk, 6][late]) and (twist < -0.1).sum() > 50)

    print('EKF silent for 1 s while driving (540.0-541.0 s):')
    _, n_d, r_d, od_d, _ = run(nt, data, 531.0, 559.0, params=ON, pin_until=1e9,
                               drop=(540.0, 541.0))
    rel = r_d[:, 0] - data['T0']
    loop = r_d[:, 1] == r_d[:, 0]                # stamped with the loop time
    check('the loop broadcasts only inside the silence (%d ticks)' % loop.sum(),
          loop.sum() > 20 and rel[loop].min() > 540.0 and rel[loop].max() < 541.05)
    first = rel[loop].min()
    before = rel[(rel < first)].max()
    check('take-over: /tf gap %.0f ms (hold 50 ms + at most one loop period)'
          % (1e3 * (first - before)), 0.05 < first - before < 0.05 + 0.04)
    hold = r_d[loop]
    check('loop ticks: loop time on both transforms, the last EKF pose (as before)',
          np.array_equal(hold[:, 1], hold[:, 2]) and len(np.unique(hold[:, 3])) == 1)
    after = r_d[rel > 541.05]
    check('hand-back: EKF-stamped again within 70 ms, none from the loop after',
          rel[~loop & (rel > 541.0)].min() < 541.07 and bool((after[:, 1] < after[:, 0]).all()))
    check('stamps strictly increase across both hand-overs', bool((np.diff(r_d[:, 1]) > 0).all()))
    check('counters: loop %d, repeat %d' % (n_d._tes_n['loop'], n_d._tes_n['repeat']),
          n_d._tes_n['loop'] == loop.sum() and n_d._tes_n['repeat'] <= 2)

    print('stamps that cannot be used, indoors, default:')
    R.CLOCK[0] = 5000.0
    n = R.make_node(nt, ON)
    rec = Recorder()
    n.tf2_TransformBroadcaster = rec

    def feed(stamp, clock, x=1.0):
        R.CLOCK[0] = clock
        n.callback_ekf_wheel_nav_odom(R.odom_msg(stamp, x, 2.0, 0.0, 0.3, 0.2, 0.0))
        return rec.n

    # times are binary fractions: a stamp survives rospy.Time unchanged
    c0 = feed(5000.0, 5000.0078125)
    check('first message: broadcast with its stamp', c0 == 1 and rec.rows[-1][1] == 5000.0)
    check('the same message again: nothing', feed(5000.0, 5000.015625, x=1.5) == 1)
    check('a message with an older stamp: nothing', feed(4999.984375, 5000.015625) == 1)
    check('stamp ahead of the clock: nothing', feed(5000.125, 5000.03125) == 1)
    check('stamp 0.156 s old (max age 0.1): nothing', feed(5000.03125, 5000.1875) == 1)
    n.loop30hz()
    check('... and the loop takes over: loop time, last received pose',
          rec.n == 2 and rec.rows[-1][1] == 5000.1875 and rec.rows[-1][3] == 1.0)
    check('a fresh message after the loop tick: EKF-stamped again',
          feed(5000.25, 5000.2578125, x=3.0) == 3 and rec.rows[-1][1] == 5000.25
          and rec.rows[-1][3] == 3.0)
    R.CLOCK[0] = 5000.2734375
    n.loop30hz()
    check('loop tick 16 ms after an EKF broadcast: nothing', rec.n == 3)
    feed(4990.0, 4990.0078125)                   # the clock was set back by 10 s
    R.CLOCK[0] = 4990.125
    n.loop30hz()
    check('clock set back: the loop takes over at once, then the EKF stamps again',
          rec.n == 4 and rec.rows[-1][1] == 4990.125
          and feed(4990.25, 4990.2578125) == 5 and rec.rows[-1][1] == 4990.25)
    check('counters %s' % n._tes_n,
          n._tes_n == {'ekf': 3, 'loop': 2, 'repeat': 3, 'old': 1, 'future': 1, 'invalid': 0,
                       'handback': 0})
    n.is_indoor = True
    c = rec.n
    feed(4990.5, 4990.5078125)
    check('indoors: nothing is broadcast (as before)', rec.n == c)

    n0 = R.make_node(nt, None)
    rec0 = Recorder()
    n0.tf2_TransformBroadcaster = rec0
    R.CLOCK[0] = 6000.0
    n0.callback_ekf_wheel_nav_odom(R.odom_msg(5999.99, 1.0, 2.0, 0.0, 0.3, 0.2, 0.0))
    check('parameter off (default): an EKF message broadcasts nothing',
          n0._tes_enable is False and rec0.n == 0)
    n0.loop30hz()
    n0.loop30hz()
    check('... every loop tick broadcasts, stamped with the loop time (repeats included)',
          rec0.n == 2 and rec0.rows[-1][1] == 6000.0 and rec0.rows[-1][2] == 6000.0)
    n4 = object.__new__(nt.NavTfNode)            # stub nodes of the older tests
    n4.ekf_wheel_nav_odom_msg = None
    n4._nav_yaw_hist = []
    n4.callback_ekf_wheel_nav_odom(R.odom_msg(1.0, 0, 0, 0, 0, 0, 0))
    check('node built without __init__ (older tests): off', len(n4._nav_yaw_hist) == 1)

    print('hand-back when every message comes just after a loop tick, stamped before it:')
    hb = nt.tf_ekf_stamp_handback
    check('tf_ekf_stamp_handback: a new message the loop stamped past silences the loop',
          hb('repeat', True, 10.02, 9.99) and not hb('repeat', False, 10.02, 9.99)
          and not hb('repeat', True, 10.02, 10.02) and not hb('old', True, 10.02, 9.99)
          and not hb('ekf', True, 10.02, 9.99))

    def phase_locked(lead_ticks, n_msgs=300, late=0.002, age=0.0078):
        """A node from its start: loop ticks every 1/30 s; from tick
        lead_ticks on an EKF message arrives `late` after each tick with a
        stamp `age` old - i.e. older than the tick."""
        R.CLOCK[0] = 8000.0
        nd = R.make_node(nt, ON)
        rc = Recorder()
        nd.tf2_TransformBroadcaster = rc
        for k in range(lead_ticks + n_msgs):
            t = 8000.0 + k / 30.0
            R.CLOCK[0] = t
            nd.loop30hz()
            if k >= lead_ticks:
                R.CLOCK[0] = t + late
                nd.callback_ekf_wheel_nav_odom(
                    R.odom_msg(t + late - age, 0.01 * k, 0.0, 0.0, 0.0, 0.3, 0.0))
        return nd, np.array(rc.rows)

    for lead, label in ((0, 'EKF there from the first tick'), (30, 'EKF starts 1 s after the node')):
        nd, rr = phase_locked(lead)
        ekf_rows = rr[rr[:, 0] - rr[:, 1] > 1e-3]
        check('%s: %d of 300 messages broadcast, the loop quiet after %d ticks'
              % (label, nd._tes_n['ekf'], nd._tes_n['loop']),
              nd._tes_n['ekf'] >= 298 and nd._tes_n['loop'] <= lead + 2
              and nd._tes_loop_mode is False and nd._tes_n['handback'] == 1
              and len(ekf_rows) == nd._tes_n['ekf'], str(nd._tes_n))
        check('... stamps strictly increase across the hand-back, largest /tf gap %.0f ms'
              % (1e3 * np.diff(rr[:, 0]).max()),
              bool((np.diff(rr[:, 1]) > 0).all()) and np.diff(rr[:, 0]).max() < 0.075)
    # a stuck EKF stamp (every message the same) must never silence the loop
    R.CLOCK[0] = 8100.0
    nd = R.make_node(nt, ON)
    rc = Recorder()
    nd.tf2_TransformBroadcaster = rc
    for k in range(60):
        R.CLOCK[0] = 8100.0 + k / 30.0
        nd.loop30hz()
        R.CLOCK[0] += 0.002
        nd.callback_ekf_wheel_nav_odom(R.odom_msg(8099.99, 1.0, 0.0, 0.0, 0.0, 0.0, 0.0))
    check('stuck EKF stamp: the loop goes on broadcasting (%d of 60 ticks), hand-backs %d '
          '(the first message only)' % (nd._tes_n['loop'], nd._tes_n['handback']),
          nd._tes_n['loop'] >= 59 and nd._tes_n['handback'] <= 1 and rc.n == nd._tes_n['loop'])

    print('dock engage walk (D4, dock_engage_slew_mps 0.10) with this parameter (794-822 s):')
    anchor = R.dock_anchor_from(data, 821.0)
    kw = dict(pin_until=799.0, dock_anchor=anchor)
    d4 = {'dock_engage_slew_mps': 0.10}

    def walk_time(params, more=None):
        seen, nominal = [], [0.0]

        def hook(nd, t, kind):
            off = getattr(nd, '_dock_engage_off', None)
            if off is not None:
                if not seen:
                    nominal[0] = max(math.hypot(off[0], off[1]) / nd._dock_engage_slew_mps,
                                     abs(off[2]) / math.radians(nd._dock_engage_slew_dps))
                seen.append(t)
        tr, nd, rr, _, _ = run(nt, data, 794.0, 822.0, params=params, hook=hook, more=more, **kw)
        return (seen[-1] - seen[0]) if seen else 0.0, tr, nd, nominal[0]

    w_off, tr_off, _, nom = walk_time(d4)
    w_on, tr_on, nd_on, nom_on = walk_time(dict(d4, **ON))
    w_raw, _, _, _ = walk_time(dict(d4, **ON),
                               more=lambda nd: setattr(nd, '_tes_walk_carry', lambda before: None))
    # D4 caps the time step of a tick at 1/30 s, so every late tick loses time:
    # the walk is never faster than the slew rate, with late ticks slower
    check('walk: the slew rate gives %.2f s; off (recorded loop ticks) %.2f s, on %.2f s, '
          'on without the carry %.2f s' % (nom_on, w_off, w_on, w_raw),
          nom > 0.5 and nom_on - 0.04 <= w_on <= 1.03 * nom_on + 0.04
          and w_on <= w_off + 0.04 and w_on <= w_raw + 1e-9)
    xo, yo, wo_ = R.map_base(tr_off)
    xn, yn, wn = R.map_base(tr_on)
    end = tr_off['t'] > tr_off['t'][-1] - 3.0
    check('... the held pose at the end equal to off',
          np.hypot(xo[end] - xn[end], yo[end] - yn[end]).max() < 1e-6 and nd_on._dock_hold_active)

    print('the switches live on the robot (D1-D4, E5, F1, F2, M1, M2) + this one:')
    for name, t0, t1, pin, anchor_t in (('hand-over of the heading at undock', 85.0, 112.0, 94.5, None),
                                        ('turning on GPS', 531.0, 559.0, 1e9, None),
                                        ('docking', 794.0, 822.0, 799.0, 821.0)):
        kw2 = dict(pin_until=pin,
                   dock_anchor=R.dock_anchor_from(data, anchor_t) if anchor_t else None)
        lo, n_lo, r_lo, _, _ = run(nt, data, t0, t1, params=LIVE, **kw2)
        ln, n_ln, r_ln, od_ln, _ = run(nt, data, t0, t1, params=dict(LIVE, **ON), **kw2)
        xo, yo, wo_ = R.map_base(lo)
        xn, yn, wn = R.map_base(ln)
        d = np.hypot(xo - xn, yo - yn)
        dy = np.degrees(np.abs(wo_ - wn))
        end = lo['t'] > lo['t'][-1] - 2.0
        msgs = fed(data, t0, t1)
        check('%s: map->base within %.1f mm / %.2f deg of the live set alone (end %.2g mm)'
              % (name, 1e3 * d.max(), dy.max(), 1e3 * d[end].max()),
              d.max() < 0.03 and dy.max() < 1.0 and d[end].max() < 0.005 and dy[end].max() < 0.2)
        check('... %d of %d EKF messages broadcast, %d loop ticks (EKF late > 50 ms), stamps '
              'increase' % (n_ln._tes_n['ekf'], len(msgs) + 1, n_ln._tes_n['loop']),
              n_ln._tes_n['ekf'] >= 0.98 * len(msgs) and n_ln._tes_n['loop'] <= 0.01 * len(msgs)
              and bool((np.diff(r_ln[:, 1]) > 0).all()))
        check('... heading state at the end equal (diff_yaw, map_odom_yaw)',
              abs(R.wrap(n_lo.diff_yaw - n_ln.diff_yaw)) < 1e-9
              and abs(n_lo.map_odom_yaw - n_ln.map_odom_yaw) < math.radians(0.05),
              '%.3g deg' % math.degrees(abs(n_lo.map_odom_yaw - n_ln.map_odom_yaw)))

    print('docking, the dock hold engages (794-822 s):')
    anchor = R.dock_anchor_from(data, 821.0)
    kw = dict(pin_until=799.0, dock_anchor=anchor)
    off, _, r_off, _, _ = run(nt, data, 794.0, 822.0, **kw)
    on, n_on, r_on, _, _ = run(nt, data, 794.0, 822.0, params=ON, **kw)
    xo, yo, wo_ = R.map_base(off)
    xn, yn, wn = R.map_base(on)
    end = off['t'] > off['t'][-1] - 3.0
    d_end = np.hypot(xo[end] - xn[end], yo[end] - yn[end]).max()
    check('held map->base at the end equal to off: %.2g mm, %.2g deg'
          % (1e3 * d_end, math.degrees(np.abs(wo_[end] - wn[end]).max())),
          d_end < 1e-6 and np.abs(wo_[end] - wn[end]).max() < 1e-6 and n_on._dock_hold_active)
    d_all = np.hypot(xo - xn, yo - yn).max()
    check('map->base over the whole window within one EKF step of off: %.1f mm' % (1e3 * d_all),
          d_all < 0.03)
    check('on: stamps strictly increase, loop ticks %d (EKF late > 50 ms) of %d'
          % (n_on._tes_n['loop'], len(r_on)),
          bool((np.diff(r_on[:, 1]) > 0).all()) and n_on._tes_n['loop'] <= 0.01 * len(r_on))

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
