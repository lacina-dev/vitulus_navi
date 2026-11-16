#!/usr/bin/env python3
"""T36 F2 (~anchor_mode_report): /nav_tf/loc_status says in which mode the
position anchor runs (continuous / fallback_set_pose / none), which share of
the GPS samples reaches its buffer, the offset of the system clock against
the GPS stamps and whether the anchor starves. REPORT ONLY. nav_score.py
turns it into anchor_fallback_pct and the flag pos_metrics_invalid.

  - anchor_mode_tick / anchor_mode_clock / anchor_mode_fields (pure),
  - the node's own loop3sec and publish_loc_status on a built drive (wheel
    EKF 30 Hz, GPS 5 Hz, the nav EKF modelled from the set_pose calls):
    parameter off = no new field and set_pose calls EQUAL to the node before
    T36 (git 30eca80); on = set_pose calls EQUAL to off, and the fields with
    the clocks equal, with the GPS stamps 0.5 s ahead (series 6 of
    2026-11-16: nothing reaches the buffer) and after GPS lost the position,
  - nav_score.anchor_fallback, score() and --compare.

gps_clock_offset_s is measured on /gnss/fix (receive time - stamp, median,
minus transport_s). The built drive knows its transport (0.03 s) and passes
it; the default 0.0 is what 26 recorded drives give for the shift that lines
the GPS positions up with the path of the wheel EKF (-0.01 +- 0.01 s).

A recorded drive through the same code (numbers only; /gnss/fix is not
recorded, its receive time and stamp are those of /gnss/navpvt):
    test_t36_f2_anchor_mode_report.py --bag DRIVE.bag [--cache DIR]

Run:  python3 test_t36_f2_anchor_mode_report.py   (needs rospy - robot env)
"""
import io
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402
import nav_score as ns  # noqa: E402

import numpy as np  # noqa: E402
import rospy  # noqa: E402
from sensor_msgs.msg import NavSatFix  # noqa: E402

ON = {'anchor_mode_report': True, 'anchor_mode_report_transport_s': 0.03}
FIELDS = ('anchor_mode', 'anchor_fallback_share', 'anchor_buffered_share',
          'anchor_starving', 'gps_clock_offset_s')
SINK = [None]


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


def fix_msg(stamp):
    m = NavSatFix()
    m.header.stamp = rospy.Time.from_sec(stamp)
    m.status.status = 2
    m.latitude, m.longitude = 50.0, 14.0
    m.position_covariance = [1e-4, 0, 0, 0, 1e-4, 0, 0, 0, 1e-4]
    return m


class Drive(object):
    """loop3sec + publish_loc_status (1 Hz) of the node with a modelled nav
    EKF: its position is the last set_pose plus the wheel-EKF delta since."""

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
        self.calls = []            # (t, x, y) of every set_pose
        self.loc = []              # (t, loc_status dict) once a second
        self._next_loc = t_sys + 1.0
        self._stamp, self._nav_yaw = t_sys, 0.0

    def _sink(self, req):
        p = req.pose.pose.pose.position
        self.calls.append((round(R.CLOCK[0], 4), round(p.x, 9), round(p.y, 9)))
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

    def gps(self, t_sys, stamp, x, y):
        R.CLOCK[0] = t_sys
        self.n.callback_rtk_odom(R.odom_msg(stamp, x, y, 0.0, 0.0, 0.0, 0.0))

    def tick(self, t_sys):
        R.CLOCK[0] = t_sys
        SINK[0] = self._sink
        self.n.loop3sec()
        SINK[0] = None
        if t_sys >= self._next_loc:
            self._next_loc += 1.0
            self.n.publish_loc_status()
            self.loc.append((t_sys, json.loads(self.n.loc_status_pub.last.data)))


def profile(tau):
    """Speed of the built drive [m/s]: 20 s periods with a stop."""
    p = tau % 20.0
    if p < 3.0 or p >= 17.0:
        return 0.0
    if p < 5.0:
        return 0.25 * (p - 3.0)
    if p < 15.0:
        return 0.5
    return 0.5 - 0.25 * (p - 15.0)


def built(nt, params, dur=60.0, gps_off=0.0, lost_at=None):
    """A straight built drive, GPS stamps = system clock + gps_off; /gnss/fix
    received 0.03 s and /odometry/gps 0.08 s after the measurement, 8 mm of
    fixed pseudo-noise; the nav EKF starts 0.3 m off. lost_at: from then on
    GPS does not own the position."""
    install()
    dt = 1.0 / 30.0
    k_end = int(dur / dt)
    s = np.concatenate([[0.0], np.cumsum([profile(k * dt) * dt for k in range(k_end)])])
    pos = lambda tau: float(np.interp(tau, np.arange(k_end + 1) * dt, s))
    ev = [(k * dt + 0.006, 0, 'w', k * dt) for k in range(k_end)]
    ev += [(0.013 + k * 0.2 + 0.03, 1, 'f', 0.013 + k * 0.2) for k in range(int(dur * 5))]
    ev += [(0.013 + k * 0.2 + 0.08, 2, 'g', 0.013 + k * 0.2) for k in range(int(dur * 5))]
    ev += [(0.031 + k * 0.1, 3, 't', None) for k in range(int(dur * 10))]
    ev.sort()
    noise = np.random.RandomState(3).normal(0.0, 0.008, int(dur * 5) + 1)
    d = Drive(nt, params, 7000.0, (pos(0.0) + 0.3, 0.0))
    for tau, _p, kind, tm in ev:
        if kind == 'w':
            d.wheel(7000.0 + tau, 7000.0 + tm, pos(tm), 0.0, 0.0, 0.0)
        elif kind == 'f':
            d.fix(7000.0 + tau, 7000.0 + tm + gps_off)
        elif kind == 'g':
            d.gps(7000.0 + tau, 7000.0 + tm + gps_off,
                  pos(tm) + float(noise[int(round((tm - 0.013) / 0.2))]), 0.0)
        else:
            d.owned = lost_at is None or tau < lost_at
            d.tick(7000.0 + tau)
    return d


def run_pure(nt, check):
    new, tick, clock, fields = (nt.anchor_mode_new_state, nt.anchor_mode_tick,
                                nt.anchor_mode_clock, nt.anchor_mode_fields)
    st = new()
    f = fields(st, 100.0)
    check('nothing yet: mode none, shares and offset None, not starving',
          f == {'anchor_mode': 'none', 'anchor_fallback_share': None,
                'anchor_buffered_share': None, 'anchor_starving': False,
                'gps_clock_offset_s': None})
    for k in range(100):                       # 10 s, every sample buffered at its 2nd tick
        t = 100.0 + 0.1 * k
        stamp = 50.0 + 0.2 * (k // 2)
        tick(st, t, k >= 30, stamp, stamp if k % 2 else stamp - 0.2)
    f = fields(st, 109.95)
    check('3 s of fallback, then continuous: mode continuous, fallback share 0.30, buffered 1.0',
          f['anchor_mode'] == 'continuous' and f['anchor_fallback_share'] == 0.30
          and f['anchor_buffered_share'] == 1.0 and not f['anchor_starving'])
    f = fields(st, 110.6)
    check('no tick for 0.6 s: mode none (GPS does not own), not starving',
          f['anchor_mode'] == 'none' and not f['anchor_starving'])
    st = new()
    for k in range(100):
        tick(st, 100.0 + 0.1 * k, False, 50.0 + 0.2 * (k // 2), 0.0)
    f = fields(st, 109.95)
    check('nothing buffered: fallback_set_pose, fallback share 1.0, buffered 0.0, starving',
          f['anchor_mode'] == 'fallback_set_pose' and f['anchor_fallback_share'] == 1.0
          and f['anchor_buffered_share'] == 0.0 and f['anchor_starving'])
    st = new()
    for k in range(100):                       # 2 of 5 samples reach the buffer
        stamp = 50.0 + 0.2 * (k // 2)
        tick(st, 100.0 + 0.1 * k, True, stamp, stamp if (k // 2) % 5 < 2 else 0.0)
    f = fields(st, 109.95)
    check('2 of 5 samples buffered: share 0.40, starving (limit 0.5)',
          f['anchor_buffered_share'] == 0.40 and f['anchor_starving'])
    st = new()
    for k in range(8):
        tick(st, 100.0 + 0.1 * k, False, 50.0 + 0.2 * (k // 2), 0.0)
    check('under 10 GPS samples in the window: not starving yet',
          not fields(st, 100.75)['anchor_starving'])
    st = new()
    for k in range(30):
        clock(st, 100.0 + k, -0.42 + (0.2 if k % 7 == 0 else 0.0))
    f = fields(st, 129.5)
    check('receive - stamp -0.42 s with outliers: gps_clock_offset_s = -0.42 (median, transport 0)',
          f['gps_clock_offset_s'] == -0.42)
    f = fields(st, 129.5, dict(nt.ANCHOR_MODE_CFG, transport_s=0.08))
    check('... with transport_s 0.08: -0.5', f['gps_clock_offset_s'] == -0.5)
    check('clock samples older than 30 s are dropped: None after a minute',
          fields(st, 200.0)['gps_clock_offset_s'] is None)
    for k in range(5):
        tick(st, 50.0 + 0.1 * k, True, 9.0, 9.0)    # the clock went back
    f = fields(st, 50.45)
    check('system clock back: old rows dropped, the new ticks count',
          f['anchor_mode'] == 'continuous' and f['anchor_fallback_share'] == 0.0)


def run_built(nt, check):
    ref = R.load_ref()
    off = built(nt, {})
    on = built(nt, ON)
    check('off: loc_status has none of the fields',
          not any(k in j for _, j in off.loc for k in FIELDS) and len(off.loc) > 50)
    if ref is not None:
        r = built(ref, {})
        check('off: set_pose calls EQUAL to the node before T36 (%d calls)' % len(off.calls),
              off.calls == r.calls and len(off.calls) > 20)
    else:
        print('  (git 30eca80 not available - comparison with the node before T36 skipped)')
    check('on: set_pose calls EQUAL to off (report only)', on.calls == off.calls)
    j = on.loc[-1][1]
    print('    clocks equal, after 60 s: %s' % {k: j.get(k) for k in FIELDS})
    check('clocks equal: continuous, buffered share >= 0.98, not starving, offset 0',
          j['anchor_mode'] == 'continuous' and j['anchor_buffered_share'] >= 0.98
          and j['anchor_fallback_share'] == 0.0 and not j['anchor_starving']
          and abs(j['gps_clock_offset_s']) <= 0.001)
    j = on.loc[1][1]
    check('... 2 s after GPS took over: fallback_set_pose (the buffer fills), not starving',
          j['anchor_mode'] == 'fallback_set_pose' and not j['anchor_starving']
          and j['anchor_buffered_share'] >= 0.9)
    off5 = built(nt, {}, gps_off=0.5)
    on5 = built(nt, ON, gps_off=0.5)
    check('stamps 0.5 s ahead: set_pose calls EQUAL to off (%d calls)' % len(on5.calls),
          on5.calls == off5.calls and len(on5.calls) > 300)
    j = on5.loc[-1][1]
    print('    stamps 0.5 s ahead:        %s' % {k: j.get(k) for k in FIELDS})
    check('stamps 0.5 s ahead: fallback_set_pose, shares 1.0 / 0.0, starving, offset -0.5',
          j['anchor_mode'] == 'fallback_set_pose' and j['anchor_fallback_share'] == 1.0
          and j['anchor_buffered_share'] == 0.0 and j['anchor_starving']
          and abs(j['gps_clock_offset_s'] + 0.5) <= 0.001)
    first = [t - 7000.0 for t, x in on5.loc if x['anchor_starving']][0]
    check('... starving raised %.0f s after GPS took over (10 samples)' % first, first <= 3.0)
    lost = built(nt, ON, gps_off=0.5, lost_at=40.0)
    j = lost.loc[-1][1]
    check('GPS lost the position 20 s ago: mode none, not starving, the offset still measured',
          j['anchor_mode'] == 'none' and not j['anchor_starving']
          and j['anchor_fallback_share'] is None and abs(j['gps_clock_offset_s'] + 0.5) <= 0.001)


def run_score(check):
    owner = np.array(['dock'] * 20 + ['gps'] * 80)
    age = np.array([np.nan] * 25 + [0.2] * 75)
    gps = np.column_stack([np.arange(100.0), np.arange(100.0) - 0.08])
    M, bad, why = ns.anchor_fallback(owner, None, age, gps)
    check('old recording, anchor_age_s null for 5 of 80 owner=gps samples: 6.25 %, valid',
          abs(M['anchor_fallback_pct'] - 6.25) < 1e-9 and bad is False
          and abs(M['gps_rx_minus_stamp_s'] - 0.08) < 1e-9)
    M, bad, why = ns.anchor_fallback(owner, None, np.full(100, np.nan), gps)
    check('old recording, anchor_age_s always null: 100 %, INVALID', M['anchor_fallback_pct'] == 100.0
          and bad is True and 'nouzovem' in why)
    tg = 0.05 + 0.2 * np.arange(500)
    tw = np.arange(3000) / 30.0
    ekfw = np.column_stack([tw + 0.006, tw])
    lead = np.column_stack([tg, tg + 0.42])
    own5, age5 = np.array(['gps'] * 100), np.full(100, 0.2)
    M, bad, why = ns.anchor_fallback(own5, np.array([''] * 100), age5, lead, None, ekfw)
    check('old recording, GPS stamps 0.42 s ahead of the receive time: none can be buffered '
          '(%.0f %%), INVALID' % M['gps_buffered_est_pct'],
          bad is True and 'hladovela' in why and M['gps_buffered_est_pct'] == 0.0
          and abs(M['gps_rx_minus_stamp_s'] + 0.42) < 1e-9)
    res = {}
    for ld in (0.08, 0.11, 0.14):
        g2 = np.column_stack([tg, tg + ld])
        res[ld] = ns.anchor_fallback(own5, None, age5, g2, None, ekfw)
    check('... 0.08 / 0.11 / 0.14 s ahead: %.0f / %.0f / %.0f %% buffered - the share decides, '
          'not the offset: %s / %s / %s'
          % tuple([res[k][0]['gps_buffered_est_pct'] for k in (0.08, 0.11, 0.14)]
                  + [res[k][1] for k in (0.08, 0.11, 0.14)]),
          res[0.08][1] is False and res[0.08][0]['gps_buffered_est_pct'] > 90.0
          and res[0.14][1] is True and res[0.14][0]['gps_buffered_est_pct'] < 30.0
          and res[0.11][1] == (res[0.11][0]['gps_buffered_est_pct'] < 50.0))
    M, bad, why = ns.anchor_fallback(own5, None, age5, np.column_stack([tg, tg - 0.3]), None, ekfw)
    check('... 0.3 s behind: all buffered (%.1f %%), valid' % M['gps_buffered_est_pct'],
          bad is False and M['gps_buffered_est_pct'] > 99.0)
    mode = np.array(['none'] * 20 + ['fallback_set_pose'] * 30 + ['continuous'] * 50)
    M, bad, why = ns.anchor_fallback(owner, mode, np.full(100, np.nan), lead)
    check('anchor_mode in the recording decides: 30 of 80 = 37.5 %, INVALID',
          abs(M['anchor_fallback_pct'] - 37.5) < 1e-9 and bad is True)
    mode = np.array(['none'] * 20 + ['fallback_set_pose'] * 4 + ['continuous'] * 76)
    M, bad, why = ns.anchor_fallback(owner, mode, np.full(100, np.nan), lead, None, ekfw)
    check('... 4 of 80 = 5 %: valid, even with the stamps 0.42 s ahead (the node corrected them)',
          abs(M['anchor_fallback_pct'] - 5.0) < 1e-9 and bad is False)
    starv = np.array([False] * 20 + [True] * 60 + [False] * 20)
    M, bad, why = ns.anchor_fallback(owner, mode, np.full(100, np.nan), lead, starv)
    check('... but anchor_starving for 60 of 80 samples: 75 %, INVALID',
          abs(M['anchor_starving_pct'] - 75.0) < 1e-9 and bad is True and 'hladovi' in why)
    M, bad, why = ns.anchor_fallback(None, None, None, None)
    check('no loc_status and no /odometry/gps: None (cannot tell)',
          bad is None and M['anchor_fallback_pct'] is None)
    t = 1000.0 + np.arange(600) / 30.0
    x = 0.3 * (t - 1000.0)
    col = lambda a: np.full(600, a, dtype=float)
    od = np.column_stack([t, t, x, col(0), col(0), col(0), col(0.3), col(0), col(0)])
    tf = np.column_stack([t, t, x, col(0), col(0), col(0)])
    D = {'wheel': od, 'ekf': od, 'odom': od, 'imu': np.column_stack([t, t, col(0), col(0)]),
         'mo': np.column_stack([t, t, col(0), col(0), col(0), col(0)]), 'ob': tf, 'counts': {},
         'loc_t': 1000.0 + np.arange(20.0), 'loc_owner': np.array(['gps'] * 20),
         'loc_anchor_age': np.full(20, np.nan), 'loc_heading_age': np.full(20, 0.1)}
    A = ns.score(D)
    D2 = dict(D, loc_anchor_age=np.full(20, 0.1))
    B = ns.score(D2)
    check('score(): pos_metrics_invalid True / False, anchor_fallback_pct 100 / 0',
          A['pos_metrics_invalid'] is True and B['pos_metrics_invalid'] is False
          and A['metrics']['anchor_fallback_pct'] == 100.0
          and B['metrics']['anchor_fallback_pct'] == 0.0)
    out = io.StringIO()
    ns.print_table(ns._jsonable(A), out)
    check('table: the warning line', 'POZOR: polohove metriky teto jizdy NEPLATI' in out.getvalue())
    out = io.StringIO()
    ns.compare(ns._jsonable(A), ns._jsonable(B), out)
    txt = out.getvalue()
    check('--compare: warns which drive is invalid and that the two do not compare',
          'jizdy "pred" NEPLATI' in txt and 'nejsou v poloze srovnatelne' in txt
          and 'jizdy "po" NEPLATI' not in txt)
    out = io.StringIO()
    old = {k: v for k, v in ns._jsonable(B).items() if not k.startswith('pos_metrics')}
    ns.compare(old, ns._jsonable(B), out)
    check('--compare with a JSON of before F2 (no flag): no warning', 'POZOR' not in out.getvalue())


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
    """A recorded drive through loop3sec + publish_loc_status with the report on."""
    D = ns.load(bag, cache)
    fx = fix_times(bag, cache)
    g, ew, ekf = D['gps'], D['ekfw'], D['ekf']
    loc_t, owner = D['loc_t'], D['loc_owner']
    nav_yaw = np.interp(ew[:, 1], ekf[:, 1], np.unwrap(ekf[:, 5]))
    a, b = max(ew[0, 0], ekf[0, 0]), min(ew[-1, 0], g[-1, 0])
    ev = [(ew[i, 0], 0, 'w', i) for i in range(len(ew)) if a <= ew[i, 0] <= b]
    ev += [(fx[i, 0], 1, 'f', i) for i in range(len(fx)) if a <= fx[i, 0] <= b]
    ev += [(g[i, 0], 2, 'g', i) for i in range(len(g)) if a <= g[i, 0] <= b]
    ev += [(t, 3, 't', 0) for t in np.arange(a + 0.05, b, 0.1)]
    ev.sort()
    install()
    i0 = int(np.searchsorted(ekf[:, 0], a))
    d = Drive(nt, {'anchor_mode_report': True}, a, (ekf[i0, 2], ekf[i0, 3]))
    for t, _p, kind, i in ev:
        if kind == 'w':
            d.wheel(t, ew[i, 1], ew[i, 2], ew[i, 3], ew[i, 5], float(nav_yaw[i]))
        elif kind == 'f':
            d.fix(t, fx[i, 1])
        elif kind == 'g':
            d.gps(t, g[i, 1], g[i, 2], g[i, 3])
        else:
            k = int(np.searchsorted(loc_t, t, side='right')) - 1
            d.owned = k >= 0 and owner[k] == 'gps'
            d.tick(t)
    js = [j for _, j in d.loc if j['anchor_mode'] != 'none']
    n = max(len(js), 1)
    offs = [j['gps_clock_offset_s'] for _, j in d.loc if j['gps_clock_offset_s'] is not None]
    sh = [j['anchor_buffered_share'] for j in js if j['anchor_buffered_share'] is not None]
    R_ = ns.score(D)
    print('%s: loc_status with GPS owning %d s: continuous %.0f %%, fallback_set_pose %.0f %%, '
          'starving %.0f %%, buffered share median %.2f, gps_clock_offset_s %+.3f .. %+.3f '
          '(/gnss/fix median %+.3f) | nav_score: anchor_fallback_pct %s, gps_buffered_est_pct %s, '
          'gps_rx_minus_stamp_s %+.3f, pos_metrics_invalid %s (%s)'
          % (os.path.basename(bag), len(js),
             100.0 * sum(j['anchor_mode'] == 'continuous' for j in js) / n,
             100.0 * sum(j['anchor_mode'] == 'fallback_set_pose' for j in js) / n,
             100.0 * sum(j['anchor_starving'] for j in js) / n,
             float(np.median(sh)) if sh else float('nan'),
             min(offs) if offs else float('nan'), max(offs) if offs else float('nan'),
             float(np.median(fx[:, 0] - fx[:, 1])),
             ns._fmt(R_['metrics']['anchor_fallback_pct']),
             ns._fmt(R_['metrics']['gps_buffered_est_pct']), R_['metrics']['gps_rx_minus_stamp_s'],
             R_['pos_metrics_invalid'], R_['pos_metrics_invalid_why']))


def main():
    if '--bag' in sys.argv:
        install()
        nt = R.load('navi_transform_t36_f2')
        cache = sys.argv[sys.argv.index('--cache') + 1] if '--cache' in sys.argv else None
        for bag in sys.argv[sys.argv.index('--bag') + 1:]:
            if bag.startswith('--'):
                break
            bag_numbers(nt, bag, cache)
        return 0
    fails = []

    def check(name, cond, detail=''):
        print('  %-76s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    install()
    nt = R.load('navi_transform_t36_f2')
    print('anchor_mode_* (pure):')
    run_pure(nt, check)
    print('built drive through loop3sec and publish_loc_status (60 s):')
    run_built(nt, check)
    print('nav_score:')
    run_score(check)
    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
