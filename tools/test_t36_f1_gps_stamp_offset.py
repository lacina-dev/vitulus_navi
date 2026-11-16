#!/usr/bin/env python3
"""T36 F1 (~gps_stamp_offset_track): the stamp of a GPS sample (UTC of the
receiver) is moved onto the system clock before the position anchor looks it
up in the wheel-EKF history. The offset is measured on /gnss/fix.

Without it a GPS stamp ahead of the system clock (the clock of the robot is
not disciplined: -0.093 s/h, series 6 of 2026-11-16 ran 0.5 s behind) is never
found in the history, the buffer of the anchor stays empty and loop3sec falls
back to set_pose on the newest GPS message 10 times a second; a stamp behind
the system clock is found at the wrong place.

  - gps_stamp_offset_step / gps_stamp_offset_time (pure): constant offset of
    either sign, drift, a step of the system clock (+-45 s, +-0.3 s: taken
    out exactly through the monotonic clock), the stamps of the receiver
    stepping, a late message, a delay that stays, GPS gap, the limit max_s,
  - the node's own callback_gnss_fix and loop3sec on a built drive (wheel EKF
    30 Hz; /gnss/fix 5 Hz received 0.03 s +- 0.02 s after the measurement;
    /odometry/gps out of a 10 Hz timer 0.02-0.12 s after the fix, the phase
    walking and hopping by 0.1 s as navsat_transform does; the nav EKF
    modelled: wheel delta + the set_pose calls of the node), parameter off
    and on: share of GPS samples that reach the buffer, fallback set_pose
    ticks, pairing error of the buffered samples,
  - parameter off: the set_pose calls are EQUAL to those of the node before
    T36 (git 30eca80).

transport_s: the built drive knows its own (0.03 s) and passes it. The
default 0.0 is what 26 recorded drives (series 4-6) give: the shift that
lines the GPS positions up with the path of the wheel EKF, minus the median
of (receive time - UTC) of /gnss/navpvt, is -0.01 s (mean, +-0.01; single
drives +-0.04) - the delay of the fix and the delay of the wheel-EKF stamps
cancel.

A recorded drive through the same code (numbers only, nothing is asserted):
    test_t36_f1_gps_stamp_offset.py --bag DRIVE.bag [--cache DIR]
(the extract of nav_score.py + receive time and UTC of /gnss/navpvt, which
/gnss/fix is stamped with; /gnss/fix itself is not recorded).

Run:  python3 test_t36_f1_gps_stamp_offset.py   (needs rospy - robot env)
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

import numpy as np  # noqa: E402
import rospy  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402

ON = {'gps_stamp_offset_track': True, 'gps_stamp_offset_transport_s': 0.03}
FIX_DELAY = 0.03
SINK = [None]
MONO = [0.0]
TORN = [0]                 # > 0: the next reads of the monotonic clock are 0.08 s apart


def _mono():
    if TORN[0]:
        TORN[0] += 1
        return MONO[0] + (0.08 if TORN[0] % 2 else 0.0)
    return MONO[0]


class Proxy(object):
    """rospy.ServiceProxy: the set_pose of the nav EKF goes to SINK."""
    def __init__(self, name='', *a, **k):
        self.name = name

    def __call__(self, req):
        if self.name == '/ekf_wheel_nav_odometry/set_pose' and SINK[0] is not None:
            SINK[0](req)

    def __getattr__(self, name):
        return lambda *a, **k: None


def install():
    R.install_stubs()
    rospy.ServiceProxy = Proxy
    rospy.wait_for_service = lambda *a, **k: None


def load(name):
    install()
    nt = R.load(name)
    if nt is not None:
        nt.time.monotonic = _mono
    return nt


def fix_msg(stamp):
    m = NavSatFix()
    m.header.stamp = rospy.Time.from_sec(stamp)
    m.status.status = 2
    m.latitude, m.longitude = 50.0, 14.0
    m.position_covariance = [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4]
    return m


class Drive(object):
    """callback_gnss_fix + loop3sec of the node with a modelled nav EKF: its
    position is the last set_pose plus the wheel-EKF delta since (rotated by
    nav yaw - wheel yaw)."""

    def __init__(self, nt, params, t_sys, xy0):
        R.CLOCK[0] = t_sys
        n = R.make_node(nt, params)
        n.init = False
        n.launch_navsat = object()
        n.position_fix_usable_history = [True, True, True]
        n._heading_ok = lambda: True
        n._update_gps_probation = lambda *a, **k: None
        self.owned = True
        n._gps_stable = lambda: self.owned
        self.n = n
        self.xy = list(xy0)
        self.prev_w = None
        self.calls = []            # (t, kind, step m, tau) kind: 'fallback' | 'anchor'
        self.tau = 0.0             # time of the drive (no clock steps)
        self.track = []            # (tau, x, y) of the modelled nav EKF at every tick
        self.offs = []             # (tau, offset) after every fix, parameter on
        self.n_fallback = self.n_anchor = self.ticks = 0
        self.fb_tau = []           # tau of every fallback tick
        self.seen = self.stored = 0
        self._seen_key = None
        self._kind = None
        real = n.set_wheel_base_nav_pose

        def wrapped(pose=None, frame='map'):
            self._kind = 'fallback' if pose is None else 'anchor'
            if pose is None:
                self.n_fallback += 1
                self.fb_tau.append(self.tau)
            return real(pose, frame)
        n.set_wheel_base_nav_pose = wrapped
        real_anchor = n._apply_continuous_anchor

        def anchor():
            ok = real_anchor()
            self.n_anchor += int(bool(ok))
            return ok
        n._apply_continuous_anchor = anchor

    def _sink(self, req):
        p = req.pose.pose.pose.position
        self.calls.append((R.CLOCK[0], self._kind,
                           math.hypot(p.x - self.xy[0], p.y - self.xy[1]), self.tau))
        self.xy = [p.x, p.y]
        self._feed_nav()

    def _feed_nav(self):
        self.n.callback_ekf_wheel_nav_odom(
            R.odom_msg(self._stamp, self.xy[0], self.xy[1], 0.0, self._nav_yaw, 0.0, 0.0))

    def wheel(self, t_sys, stamp, x, y, yaw, nav_yaw):
        R.CLOCK[0] = t_sys
        if self.prev_w is not None:
            th = nav_yaw - yaw
            dx, dy = x - self.prev_w[0], y - self.prev_w[1]
            self.xy[0] += math.cos(th) * dx - math.sin(th) * dy
            self.xy[1] += math.sin(th) * dx + math.cos(th) * dy
        self.prev_w = (x, y)
        self._stamp, self._nav_yaw = stamp, nav_yaw
        self.n.callback_ekf_wheel_odom(R.odom_msg(stamp, x, y, 0.0, yaw, 0.0, 0.0))
        self._feed_nav()

    def fix(self, t_sys, stamp):
        R.CLOCK[0] = t_sys
        SINK[0] = self._sink
        self.n.callback_gnss_fix(fix_msg(stamp))
        SINK[0] = None
        if getattr(self.n, '_gso_enable', False):
            self.offs.append((self.tau, self.n._gso_st['offset']))

    def gps(self, t_sys, stamp, x, y):
        R.CLOCK[0] = t_sys
        self.n.callback_rtk_odom(R.odom_msg(stamp, x, y, 0.0, 0.0, 0.0, 0.0))

    def tick(self, t_sys):
        R.CLOCK[0] = t_sys
        n = self.n
        SINK[0] = self._sink
        before = n._last_buffered_gps_t
        fresh = (t_sys - n._rtk_odom_rx_t) < n._rtk_fresh_s
        key = n.rtk_odom_msg.header.stamp.to_sec()
        if self.owned and fresh:
            self.ticks += 1
            if key != self._seen_key:
                self._seen_key = key
                self.seen += 1
        n.loop3sec()
        if n._last_buffered_gps_t != before and n._last_buffered_gps_t != 0.0:
            self.stored += 1
        SINK[0] = None
        self.track.append((self.tau, self.xy[0], self.xy[1]))


def profile(tau):
    """Speed of the built drive [m/s]: 20 s periods - 3 s standing, 2 s up,
    10 s at 0.5 m/s, 2 s down, 3 s standing."""
    p = tau % 20.0
    if p < 3.0 or p >= 17.0:
        return 0.0
    if p < 5.0:
        return 0.25 * (p - 3.0)
    if p < 15.0:
        return 0.5
    return 0.5 - 0.25 * (p - 15.0)


def built(nt, params, dur=120.0, sys_off=lambda tau: 0.0, gps_off=lambda tau: 0.0,
          gps_gap=None, fix_delay=lambda tau: FIX_DELAY, torn_every=0):
    """A straight built drive. System clock = 7000 + tau + sys_off(tau), GPS
    clock = 7000 + tau + gps_off(tau). GPS = the true position at the time of
    the measurement + a fixed pseudo-noise of 8 mm; /gnss/fix arrives
    fix_delay +- 0.02 s after the measurement, /odometry/gps at the next beat
    of a 10 Hz timer (period 0.1003 s: its phase walks and hops by 0.1 s); the
    nav EKF starts 0.3 m off. Returns (Drive, pairing errors [m] of the
    samples in the buffer: wheel position paired with the sample - the true
    one of its measurement)."""
    dt = 1.0 / 30.0
    k_end = int(dur / dt)
    s = np.concatenate([[0.0], np.cumsum([profile(k * dt) * dt for k in range(k_end)])])
    pos = lambda tau: float(np.interp(tau, np.arange(k_end + 1) * dt, s))
    sysc = lambda tau: 7000.0 + tau + sys_off(tau)
    jit = np.random.RandomState(5).uniform(-0.02, 0.02, int(dur * 5) + 1)
    ev = []
    for k in range(k_end):
        ev.append((k * dt + 0.006, 0, 'w', k * dt))
    for k in range(int(dur * 5)):
        tm = 0.013 + k * 0.2
        if gps_gap is None or not (gps_gap[0] <= tm < gps_gap[1]):
            rx = tm + fix_delay(tm) + float(jit[k])
            ev.append((rx, 1, 'f', tm))
            beat = 0.1003
            ev.append(((math.floor((rx + 0.005) / beat) + 1) * beat, 2, 'g', tm))
    for k in range(int(dur * 10)):
        ev.append((0.031 + k * 0.1, 3, 't', None))
    ev.sort()
    MONO[0] = 0.0
    d = Drive(nt, params, sysc(0.0), (pos(0.0) + 0.3, 0.0))
    noise = np.random.RandomState(3).normal(0.0, 0.008, int(dur * 5) + 1)
    truth = {}
    n_fix = 0
    for tau, _p, kind, tm in ev:
        d.tau = MONO[0] = tau
        if kind == 'w':
            d.wheel(sysc(tau), sysc(tm), pos(tm), 0.0, 0.0, 0.0)
        elif kind == 'f':
            n_fix += 1
            TORN[0] = 1 if torn_every and n_fix % torn_every == 0 else 0
            d.fix(sysc(tau), 7000.0 + tm + gps_off(tm))
            TORN[0] = 0
        elif kind == 'g':
            gx = pos(tm) + float(noise[int(round((tm - 0.013) / 0.2))])
            truth[gx] = pos(tm)
            d.gps(sysc(tau), 7000.0 + tm + gps_off(tm), gx, 0.0)
        else:
            d.tick(sysc(tau))
    err = [abs(truth[e[1][0]] - e[2][0]) for e in d.n._pose_buffer]
    return d, np.array(err)


def calls_of(d):
    return [(round(c[0], 4), c[1], round(c[2], 9)) for c in d.calls]


def run_pure(nt, check):
    f, new, at = nt.gps_stamp_offset_step, nt.gps_stamp_offset_new_state, nt.gps_stamp_offset_time
    cfg = dict(nt.GPS_STAMP_OFFSET_CFG, transport_s=FIX_DELAY)

    def feed(st, t0, t1, off, late=lambda t, k: 0.0, sys_step=0.0, gps_step=0.0, out=None):
        """Fixes of true time t0..t1 at 5 Hz; system clock = 5000 + t + off(t)
        + sys_step, GPS clock = 5000 + t + gps_step. out: (t, why, hold, time
        of the sample on the system clock, its error)."""
        k = 0
        t = t0
        while t < t1 - 1e-9:
            stamp = 5000.0 + t + gps_step
            true_sys = 5000.0 + t + off(t) + sys_step
            rx = true_sys + FIX_DELAY + late(t, k)
            st = f(st, rx, stamp, t + FIX_DELAY + late(t, k), cfg)
            ts = at(st['why'], st['offset'], st['hold'], stamp)
            if out is not None:
                out.append((t, st['why'], st['hold'], ts, None if ts is None else ts - true_sys,
                            st['rate_hold']))
            k += 1
            t = t0 + k * 0.2
        return st

    for off in (-0.5, 0.0, 0.5, -45.0):
        out = []
        st = feed(new(), 0.0, 30.0, lambda t: off, out=out)
        errs = [abs(o[4]) for o in out[4:]]
        check('constant %+.1f s: offset %+.4f s from the 5th sample on, time right to %.1e s'
              % (off, st['offset'], max(errs)),
              out[3][3] is None and out[4][1] == 'ok' and abs(st['offset'] - off) < 1e-6
              and max(errs) < 1e-6 and st['shifts'] == 0 and st['sys_steps'] == 0)
    out = []
    st = feed(new(), 0.0, 3600.0, lambda t: -0.5 - 0.093 * t / 3600.0, out=out)
    errs = [abs(o[4]) for o in out[4:]]
    check('drift -0.093 s/h for 1 h: time right to %.2f ms, nothing held, no step, no shift'
          % (1e3 * max(errs)), max(errs) < 0.001 and st['shifts'] == 0 and st['sys_steps'] == 0
          and not any(o[2] for o in out))
    out = []
    st = feed(new(), 0.0, 600.0, lambda t: -0.5 - 0.00015 * t, out=out)
    errs = [abs(o[4]) for o in out[4:]]
    check('drift 0.15 ms/s (6 times the measured one): followed, time right to %.1f ms'
          % (1e3 * max(errs)), max(errs) < 0.004 and st['shifts'] == 0 and st['rate_holds'] == 0)
    out = []
    st = feed(new(), 0.0, 600.0, lambda t: -0.5, late=lambda t, k: 0.5 * t / 600.0, out=out)
    errs = [abs(o[4]) for o in out[4:]]
    check('the delay creeps up 0.5 s in 10 min (0.83 ms/s): not followed as a drift - offset held '
          'to 0.2 ms/s (%.0f ms off at the end, not 500), rate_hold reported after %.0f s'
          % (1e3 * errs[-1], [o[0] for o in out if o[5]][0]),
          0.10 < errs[-1] < 0.125 and st['rate_holds'] == 1 and st['rate_hold']
          and st['shifts'] == 0 and abs(st['raw'] - st['offset']) > 0.35)
    out = []
    st = feed(new(), 0.0, 120.0, lambda t: -0.5,
              late=lambda t, k: 0.0 if (k % 40 == 0 or t < 20.0)
              else 0.1 + 0.2 * ((k * 7) % 10) / 10.0, out=out)
    errs = [abs(o[4]) for o in out[4:] if o[3] is not None]
    check('from 20 s on messages 0.1-0.3 s late, one in 8 s on time (window ages out): the last ones on time '
          'keep the offset - time right to %.1f ms, nothing taken over'
          % (1e3 * max(errs)), max(errs) < 1e-6 and st['shifts'] == 0 and st['rate_holds'] == 0
          and len(errs) == len(out) - 4 and len(st['win']) >= 5)
    for step in (45.0, -45.0, 0.3, -0.3):
        out = []
        st = feed(new(), 0.0, 30.0, lambda t: -0.5)
        st = feed(st, 30.0, 60.0, lambda t: -0.5, sys_step=step, out=out)
        bad = [o for o in out if o[3] is not None and abs(o[4]) > 1e-6]
        n_none = sum(1 for o in out if o[3] is None)
        check('system clock steps %+.1f s: offset %+.2f s at once (monotonic clock), %d samples '
              'held, none wrong' % (step, st['offset'], n_none),
              not bad and n_none == 5 and st['sys_steps'] == 1 and st['shifts'] == 0
              and abs(st['sys_step_s'] - step) < 1e-6 and abs(out[0][0] - 30.0) < 1e-9
              and abs(st['offset'] - (step - 0.5)) < 1e-6)
    for step in (0.5, -0.5):
        out = []
        st = feed(new(), 0.0, 30.0, lambda t: -0.5)
        st = feed(st, 30.0, 60.0, lambda t: -0.5, gps_step=step, out=out)
        wrong = [o for o in out if o[3] is not None and abs(o[4]) > 1e-6]
        n_wait = sum(1 for o in out if o[3] is None)
        if step > 0:        # receive - stamp drops: cannot be a delay
            check('GPS stamps step +0.5 s: 4 samples held, then the new offset, none wrong, 1 shift '
                  'reported', not wrong and n_wait == 4 and st['shifts'] == 1
                  and abs(st['shift_s'] + 0.5) < 1e-6)
        else:               # receive - stamp grows: looks like a delay
            check('GPS stamps step -0.5 s: looks like a delay - old offset for 10 s (%d samples '
                  '0.5 s off), then the new one, 1 shift reported' % len(wrong),
                  st['shifts'] == 1 and abs(st['shift_s'] - 0.5) < 1e-6 and n_wait == 0
                  and abs(out[-1][4]) < 1e-6 and 49 <= len(wrong) <= 52
                  and all(abs(o[4] - step) < 1e-6 for o in wrong))
    out = []
    st = feed(new(), 0.0, 30.0, lambda t: -0.5, late=lambda t, k: 0.3 if 100 <= k < 110 else 0.0,
              out=out)
    check('10 messages 0.3 s late: left out, offset and times do not move, nothing held',
          all(o[3] is not None and abs(o[4]) < 1e-6 for o in out[4:]) and st['shifts'] == 0
          and abs(st['offset'] + 0.5) < 1e-6)
    out = []
    st = feed(new(), 0.0, 60.0, lambda t: -0.5, late=lambda t, k: 0.5 if t >= 20.0 else 0.0, out=out)
    first_bad = [o[0] for o in out if o[3] is not None and abs(o[4]) > 1e-6]
    check('the delay grows by 0.5 s and stays: times right for 10 s more, then 0.5 s off - '
          'reported as a shift of %+.2f s (cannot be told from the stamps stepping)' % st['shift_s'],
          st['shifts'] == 1 and abs(st['shift_s'] - 0.5) < 1e-6 and 29.9 < first_bad[0] < 30.3)
    out = []
    st = feed(new(), 0.0, 60.0, lambda t: -0.5,
              late=lambda t, k: 0.5 if 20.0 <= t < 40.0 else 0.0, out=out)
    wrong = [o[0] for o in out if o[3] is not None and abs(o[4]) > 1e-6]
    check('... and goes after 20 s: wrong for %.1f s (30 s -> 40.8 s), 2 shifts reported'
          % (wrong[-1] - wrong[0] + 0.2), st['shifts'] == 2 and wrong[0] > 29.9 and wrong[-1] < 40.9
          and abs(st['offset'] + 0.5) < 1e-6)
    out = []
    st = feed(new(), 0.0, 30.0, lambda t: -0.5)
    st = feed(st, 90.0, 100.0, lambda t: -0.51, out=out)
    errs = [abs(o[4]) for o in out if o[3] is not None]
    check('GPS back after a 60 s gap, clock 10 ms further: the offset of before the gap holds, '
          'no waiting, time right to %.0f ms' % (1e3 * max(errs)),
          len(errs) == len(out) and max(errs) < 0.0101 and st['shifts'] == 0)
    out = []
    st = feed(new(), 0.0, 30.0, lambda t: -500.0, out=out)
    check('offset -500 s (above max_s 120): stamps as they are, why = over_max, measured %.1f'
          % st['raw'],
          st['why'] == 'over_max' and st['offset'] is None
          and all(o[3] == 5000.0 + o[0] for o in out[4:]))
    jit = np.random.RandomState(2).uniform(-0.04, 0.04, 2000)
    out = []
    st = feed(new(), 0.0, 300.0, lambda t: 2.0, late=lambda t, k: float(jit[k]), out=out)
    errs = [abs(o[4]) for o in out[20:] if o[3] is not None]
    check('receive time scattered +-0.04 s, clock 2 s ahead: time right to %.0f ms, no shift'
          % (1e3 * max(errs)), max(errs) < 0.012 and st['shifts'] == 0
          and len(errs) == len(out) - 20)
    st = f(new(), 100.0, 0.0, 1.0, cfg)
    check('stamp 0: left alone', not st['win'] and st['why'] == 'no_data')


def run_built(nt, check):
    ref = R.load_ref()
    cases = (
        ('clocks equal', {}),
        ('GPS stamps 0.5 s ahead (series 6)', {'gps_off': lambda t: 0.5}),
        ('GPS stamps 0.5 s behind', {'gps_off': lambda t: -0.5}),
    )
    res = {}
    for name, kw in cases:
        off, e_off = built(nt, {}, **kw)
        on, e_on = built(nt, ON, **kw)
        res[name] = (off, on, e_off, e_on)
        print('  %s:' % name)
        for tag, d, e in (('off', off, e_off), ('on ', on, e_on)):
            print('    %s  stored %4d / %4d GPS samples (%5.1f %%), fallback set_pose ticks %4d of %d, '
                  'anchor ticks %4d, pairing error max %.1f cm'
                  % (tag, d.stored, d.seen, 100.0 * d.stored / d.seen, d.n_fallback, d.ticks,
                     d.n_anchor, 100 * (e.max() if len(e) else float('nan'))))
    off, on, e_off, e_on = res['clocks equal']
    st = on.n._gso_st
    rng = [o for _, o in on.offs if o is not None]
    check('clocks equal, on: stored %d / %d, fallback ticks %d / %d, offset %+.1f .. %+.1f ms, '
          'no step, no shift' % (on.stored, off.stored, on.n_fallback, off.n_fallback,
                                 1e3 * min(rng), 1e3 * max(rng)),
          off.stored - 5 <= on.stored <= off.stored and on.n_fallback <= off.n_fallback + 10
          and st['shifts'] == 0 and st['sys_steps'] == 0 and max(abs(min(rng)), abs(max(rng))) < 0.008)
    check('clocks equal: pairing error on %.1f mm (off %.1f mm: the fix is 0.03 s old when stamped '
          'right)' % (1e3 * e_on.max(), 1e3 * e_off.max()), e_on.max() < 0.004 and e_off.max() < 1e-6)
    if ref is not None:
        r, _ = built(ref, {})
        check('clocks equal, off: set_pose calls EQUAL to the node before T36 (%d calls)'
              % len(off.calls), calls_of(off) == calls_of(r) and len(off.calls) > 0)
        r, _ = built(ref, {}, gps_off=lambda t: 0.5)
        o = res['GPS stamps 0.5 s ahead (series 6)'][0]
        check('stamps 0.5 s ahead, off: EQUAL to the node before T36 (%d calls)' % len(o.calls),
              calls_of(o) == calls_of(r) and len(o.calls) > 0)
    else:
        print('  (git 30eca80 not available - comparison with the node before T36 skipped)')
    off, on, e_off, e_on = res['GPS stamps 0.5 s ahead (series 6)']
    check('stamps ahead, off: nothing stored, every tick a fallback set_pose (%d of %d)'
          % (off.n_fallback, off.ticks), off.stored == 0 and off.n_fallback == off.ticks)
    check('stamps ahead, on: %.1f %% stored, fallback only while the buffer fills (%d ticks)'
          % (100.0 * on.stored / on.seen, on.n_fallback),
          on.stored >= on.seen - 6 and on.n_fallback < 60 and on.n_anchor > on.ticks - 60)
    check('stamps ahead, on: pairing error of the buffered samples %.1f mm, no shift'
          % (1e3 * e_on.max()), e_on.max() < 0.004 and on.n._gso_st['shifts'] == 0)
    off, on, e_off, e_on = res['GPS stamps 0.5 s behind']
    check('stamps behind, off: stored, but paired 0.5 s early - error up to %.0f cm at 0.5 m/s; '
          'on: %.1f mm' % (100 * e_off.max(), 1e3 * e_on.max()),
          e_off.max() > 0.2 and e_on.max() < 0.004 and on.stored >= on.seen - 6)

    for step in (45.0, -45.0):
        kw = {'sys_off': lambda t, s=step: s if t >= 60.0 else 0.0,
              'gps_off': lambda t: 0.5}
        on, e_on = built(nt, ON, **kw)
        late = [c for c in on.calls if c[1] == 'fallback' and c[3] > 72.0]
        big = max([c[2] for c in on.calls if c[3] > 60.0] or [0.0])
        on.n.publish_loc_status()
        j = json.loads(on.n.loc_status_pub.last.data)
        fb = sum(1 for t in on.fb_tau if t > 60.0)
        print('  system clock %+.0f s at 60 s: stored %d / %d, fallback ticks after it %d, biggest '
              'set_pose step after it %.1f cm, anchor_age_s %s'
              % (step, on.stored, on.seen, fb, 100 * big, j['anchor_age_s']))
        check('system clock %+.0f s in the drive: seen as a step of the system clock, buffer '
              'emptied, anchor back within 12 s' % step,
              not late and big < 0.0501 and e_on.max() < 0.004 and on.n._gso_st['sys_steps'] == 1
              and on.n._gso_st['shifts'] == 0 and on.stored > 0.85 * on.seen)
        check('... the lag of the anchor holds (fallback ticks after the step, no sample younger '
              'than the lag), anchor_age_s not negative, loc_status: %s / %d step'
              % (j['gps_stamp_state'], j['gps_stamp_sys_steps']),
              fb >= 20 and j['anchor_age_s'] is not None and 0.0 <= j['anchor_age_s'] < 5.0
              and j['gps_stamp_state'] == 'ok' and j['gps_stamp_sys_steps'] == 1
              and abs(j['gps_stamp_offset_s'] - (step - 0.5)) < 0.01)
    on, e_on = built(nt, ON, dur=300.0, gps_off=lambda t: 0.3 + 0.00015 * t)
    check('drift 0.15 ms/s: %.1f %% stored, pairing error %.1f mm, no rate_hold'
          % (100.0 * on.stored / on.seen, 1e3 * e_on.max()),
          on.stored >= on.seen - 6 and e_on.max() < 0.005 and on.n._gso_st['rate_holds'] == 0)
    on, e_on = built(nt, ON, gps_off=lambda t: 0.5, torn_every=10)
    st = on.n._gso_st
    check('every 10th fix: the two clocks read 0.08 s apart - dropped (%d), no step of the system '
          'clock, %.1f %% stored, pairing error %.1f mm'
          % (on.n._gso_torn, 100.0 * on.stored / on.seen, 1e3 * e_on.max()),
          on.n._gso_torn == 60 and st['sys_steps'] == 0 and st['shifts'] == 0
          and on.stored >= on.seen - 6 and e_on.max() < 0.004)
    on, e_on = built(nt, ON, dur=200.0, gps_off=lambda t: 0.5,
                     fix_delay=lambda t: FIX_DELAY + 0.5 * max(t - 20.0, 0.0) / 180.0)
    on.n.publish_loc_status()
    j = json.loads(on.n.loc_status_pub.last.data)
    check('/gnss/fix delay creeps up 0.5 s in 3 min: held (pairing error %.1f cm, not 25), '
          'loc_status %s, gps_stamp_rate_holds %d, measured %+.2f / in use %+.2f'
          % (100 * e_on.max(), j['gps_stamp_state'], j['gps_stamp_rate_holds'],
             j['gps_stamp_measured_s'], j['gps_stamp_offset_s']),
          e_on.max() < 0.03 and j['gps_stamp_state'] == 'rate_hold'
          and j['gps_stamp_rate_holds'] == 1 and j['gps_stamp_shifts'] == 0)
    on, e_on = built(nt, ON, gps_off=lambda t: 0.5, gps_gap=(40.0, 75.0))
    check('GPS silent 35 s: %d of %d stored after it, pairing error %.1f mm'
          % (on.stored, on.seen, 1e3 * e_on.max()),
          on.stored >= on.seen - 12 and e_on.max() < 0.004)
    on, e_on = built(nt, ON, gps_off=lambda t: 0.5,
                     fix_delay=lambda t: FIX_DELAY + (0.3 if 50.0 <= t < 56.0 else 0.0))
    check('/gnss/fix 0.3 s late for 6 s: offset does not move, pairing error %.1f mm, no shift'
          % (1e3 * e_on.max()), e_on.max() < 0.004 and on.n._gso_st['shifts'] == 0
          and on.stored >= on.seen - 8)
    on, e_on = built(nt, ON, gps_off=lambda t: 0.5,
                     fix_delay=lambda t: FIX_DELAY + (0.5 if t >= 50.0 else 0.0))
    on.n.publish_loc_status()
    j = json.loads(on.n.loc_status_pub.last.data)
    check('/gnss/fix 0.5 s late for good: followed after 10 s, pairing error then %.0f cm - '
          'loc_status gps_stamp_shifts %d, gps_stamp_shift_s %+.2f'
          % (100 * e_on.max(), j['gps_stamp_shifts'], j['gps_stamp_shift_s']),
          j['gps_stamp_shifts'] == 1 and abs(j['gps_stamp_shift_s'] - 0.5) < 0.02
          and 0.2 < e_on.max() < 0.27)
    off, _ = built(nt, {}, gps_off=lambda t: 500.0)
    on, _ = built(nt, ON, gps_off=lambda t: 500.0)
    check('stamps 500 s ahead (above max_s): on = off (%d fallback ticks, nothing stored)'
          % on.n_fallback,
          calls_of(on) == calls_of(off) and on.stored == 0 and on.n._gso_st['why'] == 'over_max')
    off, _ = built(nt, {})
    off.n.publish_loc_status()
    j0 = json.loads(off.n.loc_status_pub.last.data)
    check('off: loc_status carries no gps_stamp_* field',
          not any(k.startswith('gps_stamp') for k in j0))


def fix_times(bag, cache):
    """(receive time, UTC stamp) of /gnss/fix; from /gnss/navpvt when the fix
    is not recorded (the driver stamps the fix with that UTC)."""
    import calendar
    f = None
    if cache:
        st = os.stat(bag)
        f = os.path.join(cache, '%s.%d.t36fix.npz' % (os.path.basename(bag), st.st_size))
        if os.path.exists(f):
            return np.load(f)['fix']
    import rosbag
    rows, pvt = [], []
    with rosbag.Bag(bag) as b:
        for topic, m, t in b.read_messages(topics=['/gnss/fix', '/gnss/navpvt']):
            if topic == '/gnss/fix':
                rows.append((t.to_sec(), m.header.stamp.to_sec()))
            else:
                pvt.append((t.to_sec(), calendar.timegm((m.year, m.month, m.day, m.hour, m.min,
                                                         m.sec)) + m.nano * 1e-9))
    out = np.array(rows or pvt, dtype=float)
    if f:
        np.savez_compressed(f, fix=out)
    return out


def bag_numbers(nt, bag, cache):
    """A recorded drive through callback_gnss_fix + loop3sec, parameter off and on."""
    import nav_score as ns
    D = ns.load(bag, cache)
    fx = fix_times(bag, cache)
    g, ew, ekf = D['gps'], D['ekfw'], D['ekf']
    loc_t, owner = D['loc_t'], D['loc_owner']
    nav_yaw = np.interp(ew[:, 1], ekf[:, 1], np.unwrap(ekf[:, 5]))
    truth = float(np.median(fx[:, 0] - fx[:, 1]))        # system clock - GPS stamps (transport 0)
    dg = g[:, 0] - g[:, 1]
    print('%s: receive - stamp: /gnss/fix median %+.3f s (p5 %+.3f, p95 %+.3f) | /odometry/gps '
          'p5 %+.3f, median %+.3f, p95 %+.3f'
          % (os.path.basename(bag), truth, np.percentile(fx[:, 0] - fx[:, 1], 5),
             np.percentile(fx[:, 0] - fx[:, 1], 95), np.percentile(dg, 5), np.median(dg),
             np.percentile(dg, 95)))
    a, b = max(ew[0, 0], ekf[0, 0]), min(ew[-1, 0], g[-1, 0])
    ev = [(ew[i, 0], 0, 'w', i) for i in range(len(ew)) if a <= ew[i, 0] <= b]
    ev += [(fx[i, 0], 1, 'f', i) for i in range(len(fx)) if a <= fx[i, 0] <= b]
    ev += [(g[i, 0], 2, 'g', i) for i in range(len(g)) if a <= g[i, 0] <= b]
    ev += [(t, 3, 't', 0) for t in np.arange(a + 0.05, b, 0.1)]
    ev.sort()
    moving = np.abs(ew[:, 6]) > 0.03
    drive_min = float(np.diff(ew[:, 0])[moving[:-1]].sum()) / 60.0
    for tag, params in (('off', {}), ('on', {'gps_stamp_offset_track': True})):
        i0 = int(np.searchsorted(ekf[:, 0], a))
        d = Drive(nt, params, a, (ekf[i0, 2], ekf[i0, 3]))
        hist, along, gap = [], [], []
        for t, _p, kind, i in ev:
            d.tau = MONO[0] = t
            if kind == 'w':
                d.wheel(t, ew[i, 1], ew[i, 2], ew[i, 3], ew[i, 5], float(nav_yaw[i]))
                hist.append((ew[i, 1], d.xy[0], d.xy[1], float(nav_yaw[i]), ew[i, 6]))
            elif kind == 'f':
                d.fix(t, fx[i, 1])
            elif kind == 'g':
                d.gps(t, g[i, 1], g[i, 2], g[i, 3])
                ts = g[i, 1] + truth
                if d.owned and len(hist) > 60:
                    h = np.array(hist[-60:])
                    ex = np.interp(ts, h[:, 0], h[:, 1]) - g[i, 2]
                    ey = np.interp(ts, h[:, 0], h[:, 2]) - g[i, 3]
                    yw = np.interp(ts, h[:, 0], h[:, 3])
                    gap.append(math.hypot(ex, ey))
                    if abs(np.interp(ts, h[:, 0], h[:, 4])) > 0.1:
                        along.append((ex * math.cos(yw) + ey * math.sin(yw))
                                     * np.sign(np.interp(ts, h[:, 0], h[:, 4])))
            else:
                k = int(np.searchsorted(loc_t, t, side='right')) - 1
                d.owned = k >= 0 and owner[k] == 'gps'
                d.tick(t)
        st = np.array([c[2] for c in d.calls])
        mv = np.array([bool(np.interp(c[0], ew[:, 0], moving)) for c in d.calls], dtype=bool)
        fb = sum(1 for c in d.calls if c[1] == 'fallback')
        al = np.array(along) if along else np.array([float('nan')])
        print('  %-3s stored %4d / %4d (%5.1f %%) | GPS-owned ticks %5d: fallback %5d (%5.1f %%) | '
              'set_pose calls %4d (fallback %4d) | model steps driving > 1 cm %5.1f /min, > 3 cm '
              '%5.1f /min | |model - GPS| median %.1f cm, p95 %.1f cm | along the track (driving): '
              'median %+.1f cm, |.| median %.1f cm, p95 %.1f cm'
              % (tag, d.stored, d.seen, 100.0 * d.stored / max(d.seen, 1), d.ticks, d.n_fallback,
                 100.0 * d.n_fallback / max(d.ticks, 1), len(d.calls), fb,
                 (st[mv] > 0.01).sum() / drive_min, (st[mv] > 0.03).sum() / drive_min,
                 100 * np.median(gap), 100 * np.percentile(gap, 95),
                 100 * np.median(al), 100 * np.median(np.abs(al)), 100 * np.percentile(np.abs(al), 95)))
        if tag == 'on':
            s = d.n._gso_st
            o = np.array([v for _, v in d.offs if v is not None])
            held = sum(1 for _, v in d.offs if v is None)
            print('      offset: %+.3f .. %+.3f s, median %+.3f (whole-drive median of /gnss/fix '
                  '%+.3f, largest difference %.0f ms) | steps of the system clock %d, shifts %d, '
                  'fixes without an offset %d of %d'
                  % (o.min(), o.max(), np.median(o), truth, 1e3 * np.abs(o - truth).max(),
                     s['sys_steps'], s['shifts'], held, len(d.offs)))


def main():
    if '--bag' in sys.argv:
        nt = load('navi_transform_t36_f1')
        cache = sys.argv[sys.argv.index('--cache') + 1] if '--cache' in sys.argv else None
        i = sys.argv.index('--bag') + 1
        for bag in sys.argv[i:]:
            if bag.startswith('--'):
                break
            bag_numbers(nt, bag, cache)
        return 0
    fails = []

    def check(name, cond, detail=''):
        print('  %-76s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    nt = load('navi_transform_t36_f1')
    print('gps_stamp_offset_step:')
    run_pure(nt, check)
    print('built drive through loop3sec (120 s, 0.5 m/s with stops):')
    run_built(nt, check)
    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
