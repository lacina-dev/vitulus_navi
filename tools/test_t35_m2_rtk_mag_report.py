#!/usr/bin/env python3
"""T35 M2 (~rtk_mag_report): /nav_tf/loc_status shows the difference between
the RTK heading the node is about to take and the magnetometer heading, and a
flag + counter "the RTK heading disagrees with the magnetometer". REPORT ONLY.

On the 36 dock approaches of 2026-11-15/16 the RTK heading in front of the
garage passes every quality gate while being several degrees off; the
magnetometer field is disturbed at the same place, so the comparison rests on
the last `ok` magnetometer offset carried by the gyro (~rtk_mag_report_hold_s).

Part 1: the pure functions (rtk_mag_offset_diff, rtk_mag_report_step).
Part 2: the node built by its real __init__ offline (see _t36_replay.py):
parameter off = no subscriber, no field, same heading state; on = the fields,
and still the same heading state.
Run:  python3 test_t35_m2_rtk_mag_report.py   (needs rospy - robot env)
"""
import json
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

import rospy  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402

D = math.radians
FIXED = 0x100 | (2 << 3) | 0x7        # relPosHeadingValid + carrSoln fixed
FLOAT = 0x100 | (1 << 3) | 0x7
KEYS = ('rtk_mag_diff_deg', 'rtk_mag_age_s', 'rtk_mag_valid', 'rtk_mag_ref_age_s',
        'rtk_mag_over_s', 'rtk_mag_disagree', 'rtk_mag_disagree_count',
        'rtk_mag_disagree_age_s')
ON = {'rtk_mag_report': True}


def mag_msg(stamp, yaw, ok=True):
    m = Imu()
    m.header.stamp = rospy.Time.from_sec(stamp)
    m.orientation.z, m.orientation.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)
    cov = [1000.0, 0.0, 0.0, 0.0, 1000.0, 0.0, 0.0, 0.0, D(4.0) ** 2 if ok else 1000.0]
    m.orientation_covariance = cov
    return m


class Feed(object):
    """100 Hz IMU, 10 Hz magnetometer heading, 5 Hz RTK heading, 1 Hz
    loc_status. Headings in the RTK convention; `rtk_err` / `mag_err` (deg)
    are added to the true heading, `rate` turns the robot (rad/s)."""

    def __init__(self, nt, params=None, t=9000.0):
        R.CLOCK[0] = t
        self.n = n = R.make_node(nt, params)
        n.init = False
        n.fix_usable = n.heading_fix_usable = True
        n.callback_relposned(types.SimpleNamespace(flags=FIXED))
        self.yaw = 0.3
        self.k = 0
        self.loc = None

    def go(self, secs, rtk_err=0.0, mag_err=0.0, mag_ok=True, rate=0.0, flags=FIXED,
           mag=True, rtk=True, acc_deg=1.4):
        n = self.n
        for _ in range(int(round(secs * 100))):
            R.CLOCK[0] += 0.01
            self.k += 1
            self.yaw = R.wrap(self.yaw + rate * 0.01)
            t = R.CLOCK[0]
            n.callback_imu(R.imu_msg(t, R.wrap(self.yaw + 1.0), rate))     # IMU yaw: own origin
            if mag and self.k % 10 == 3:
                # the magnetometer sample is 9 ms old when it arrives
                y = R.wrap(self.yaw - rate * 0.009 + D(mag_err))
                if getattr(n, '_rmr_enable', False):
                    n._rmr_cb_mag(mag_msg(t - 0.009, y, mag_ok))
            if rtk and self.k % 20 == 7:
                n.callback_relposned(types.SimpleNamespace(flags=flags))
                n.callback_navheading(R.navheading_msg(t, R.wrap(self.yaw + D(rtk_err)), acc_deg))
            if self.k % 100 == 0:
                n.publish_loc_status()
                self.loc = json.loads(n.loc_status_pub.last.data)
        return self.loc

    def heading_state(self):
        n = self.n
        return (n.diff_yaw, n.map_odom_yaw, n.init, n._heading_good_after_acq,
                tuple(n._ha_buf), n._ha_last_applied_t)


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-74s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    nt = R.load('navi_transform_t35_m2')

    print('pure: difference of the offsets')
    f = nt.rtk_mag_offset_diff
    check('rtk 5 deg left of mag -> +5', abs(f(D(35), D(30)) - 5) < 1e-9)
    check('across +-180: 178 and -178 -> -4', abs(f(D(178), D(-178)) + 4) < 1e-9)
    check('offsets a full turn apart -> 0', abs(f(D(10) + 2 * math.pi, D(10))) < 1e-9)

    print('pure: disagree = time above the limit within the window')
    new, step = nt.rtk_mag_report_new_state, nt.rtk_mag_report_step
    cfg = {'disagree_deg': 6.0, 'disagree_s': 0.3, 'window_s': 5.0}

    def run(st, t0, diffs, dt=0.2):
        out, edges = None, []
        for k, d in enumerate(diffs):
            st, out = step(st, t0 + k * dt, d, cfg)
            if out['edge']:
                edges.append((round(k * dt, 1), out['edge']))
        return st, out, edges
    st, out, edges = run(new(), 100.0, [2.0, -3.0, 5.9, -5.9] * 10)
    check('differences up to 5.9 deg for 8 s -> never', not out['disagree'] and not edges
          and st['count'] == 0 and out['over_s'] == 0.0)
    st, out, edges = run(new(), 100.0, [1.0] * 5 + [7.0] + [1.0] * 40)
    check('one sample at 7 deg -> not yet (0.2 s of 0.3)', not edges and st['count'] == 0)
    st, out, edges = run(new(), 100.0, [1.0] * 5 + [7.0, 1.0, 1.0, -8.0] + [1.0] * 40)
    check('two samples above (+7, -8) within 5 s -> raised at the second, cleared 5 s on',
          edges == [(1.6, 'raised'), (6.2, 'cleared')] and st['count'] == 1, str(edges))
    st2, out, edges = run(st, 120.0, [9.0, 9.0, 9.0])
    check('a second episode -> count 2, last_true kept', st2['count'] == 2 and out['disagree']
          and abs(st2['last_true'] - 120.4) < 1e-9)
    st3, out = step(st2, 121.0, None, cfg)
    check('no comparison (None) adds nothing, the flag holds while the window does',
          out['disagree'] and len(st3['win']) == len(st2['win']))
    st3, out = step(st3, 126.0, None, cfg)
    check('... and ages out with the window', not out['disagree'] and out['edge'] == 'cleared'
          and st3['win'] == () and st3['count'] == 2)
    st, out, edges = run(new(), 100.0, [9.0, 9.0], dt=3.0)
    check('samples 3 s apart count 0.2 s each (not the gap)', abs(out['over_s'] - 0.4) < 1e-9)
    st, out = step(st, 50.0, 9.0, cfg)
    check('clock back -> window restarted', len(st['win']) == 1 and not out['disagree'])
    st0 = new()
    step(st0, 1.0, 9.0, cfg)
    check('the state passed in is not modified', st0 == new())
    st, out = step(new(), 1.0, float('nan'), cfg)
    check('non-finite difference -> no comparison', st['win'] == ())
    st, out = step(new(), 1.0, 9.0, None)
    check('defaults: 6 deg, 0.3 s, 5 s window, 30 s hold',
          [nt.RTK_MAG_REPORT_DEFAULTS[k] for k in ('disagree_deg', 'disagree_s', 'window_s', 'hold_s')]
          == [6.0, 0.3, 5.0, 30.0] and out['over_s'] == 0.2)

    print('node: parameter off (default)')
    subs = []
    real_sub = rospy.Subscriber
    rospy.Subscriber = lambda *a, **k: subs.append(a[0]) or types.SimpleNamespace()
    warns = []
    real_warn = rospy.logwarn
    rospy.logwarn = lambda *a, **k: warns.append(a[0] % a[1:] if len(a) > 1 else a[0])
    try:
        off = Feed(nt)
        n_subs_off = len(subs)
        off.go(1.0, rtk=False, mag=False)
        loc = off.go(11.0, rtk_err=9.0)
        check('no rtk_mag_* field in loc_status', not [k for k in loc if k.startswith('rtk_mag')])
        check('no state of M2 on the node',
              [a for a in vars(off.n) if a.startswith('_rmr_')] == ['_rmr_enable']
              and off.n._rmr_enable is False)

        print('node: ~rtk_mag_report true')
        del subs[:]
        on = Feed(nt, ON)
        check('one more subscriber (%s), nothing else' % subs.count('/bno085/mag_heading'),
              len(subs) == n_subs_off + 1 and subs.count('/bno085/mag_heading') == 1)
        loc = on.go(1.0, rtk=False, mag=False)
        check('before any message: all fields there, null / false',
              [loc.get(k, 0) for k in KEYS] == [None, None, False, None, 0.0, False, 0, None],
              str([loc.get(k, 0) for k in KEYS]))
        base = {k: v for k, v in loc.items() if k not in KEYS}
        loc = on.go(11.0, rtk_err=9.0)
        check('RTK 9 deg off the magnetometer -> diff +9.0, valid, disagree, count 1',
              loc['rtk_mag_diff_deg'] == 9.0 and loc['rtk_mag_valid'] is True
              and loc['rtk_mag_disagree'] is True and loc['rtk_mag_disagree_count'] == 1
              and loc['rtk_mag_ref_age_s'] < 0.2 and loc['rtk_mag_disagree_age_s'] < 0.3,
              json.dumps({k: loc[k] for k in KEYS}))
        mw = [w for w in warns if 'rtk_mag' in w]
        check('exactly one warning, with the numbers',
              len(mw) == 1 and '+9.0 deg' in mw[0] and 'report only' in mw[0])
        check('the same heading state as with the parameter off (report only)',
              on.heading_state() == off.heading_state())
        off.go(1.0, rtk=False, mag=False)
        check('the other loc_status fields are the ones the node has without M2',
              sorted(base) == sorted(off.loc))
        loc = on.go(8.0, rtk_err=1.5)
        check('agreement again -> diff 1.5, flag cleared, count stays 1',
              loc['rtk_mag_diff_deg'] == 1.5 and loc['rtk_mag_disagree'] is False
              and loc['rtk_mag_disagree_count'] == 1 and loc['rtk_mag_over_s'] == 0.0)
        loc = on.go(6.0, rtk_err=1.5, rate=0.9)
        check('turning 0.9 rad/s (messages 9 ms and 10 ms old) -> still 1.5 +- 0.3',
              abs(loc['rtk_mag_diff_deg'] - 1.5) <= 0.3 and not loc['rtk_mag_disagree'],
              str(loc['rtk_mag_diff_deg']))
        loc = on.go(10.0, rtk_err=8.0, mag_err=40.0, mag_ok=False)
        check('magnetometer not ok (1e3 covariance): its last ok offset is used -> 8.0',
              loc['rtk_mag_diff_deg'] == 8.0 and loc['rtk_mag_valid'] is True
              and 9.0 < loc['rtk_mag_ref_age_s'] < 11.0 and loc['rtk_mag_disagree_count'] == 2)
        loc = on.go(27.0, rtk_err=8.0, mag_err=40.0, mag_ok=False)
        check('... for 30 s only: then no comparison (valid false, flag cleared)',
              loc['rtk_mag_valid'] is False and loc['rtk_mag_disagree'] is False
              and loc['rtk_mag_ref_age_s'] > 30.0 and loc['rtk_mag_age_s'] > 1.0)
        loc = on.go(3.0, rtk_err=0.5)
        check('magnetometer ok again -> compared again', loc['rtk_mag_valid'] is True
              and loc['rtk_mag_diff_deg'] == 0.5)
        c0 = loc['rtk_mag_disagree_count']
        loc = on.go(6.0, rtk_err=20.0, flags=FLOAT)
        check('RTK heading FLOAT (the node refuses it) -> not compared, no flag',
              loc['rtk_mag_valid'] is False and loc['rtk_mag_disagree_count'] == c0)
        loc = on.go(6.0, rtk_err=20.0, acc_deg=0.9)
        check('variance below the band -> not compared', loc['rtk_mag_valid'] is False
              and loc['rtk_mag_disagree_count'] == c0)
        on.n.heading_fix_usable = False
        loc = on.go(3.0, rtk_err=20.0)
        on.n.heading_fix_usable = True
        check('heading fix not usable -> not compared', loc['rtk_mag_valid'] is False)
        loc = on.go(3.0, rtk_err=-7.0)
        check('usable again, -7 deg -> diff -7.0, count + 1', loc['rtk_mag_diff_deg'] == -7.0
              and loc['rtk_mag_disagree_count'] == c0 + 1)

        print('node: robustness')
        thr = []
        real_thr = rospy.logwarn_throttle
        rospy.logwarn_throttle = lambda period, msg, *a: thr.append(msg)
        try:
            m = mag_msg(R.CLOCK[0], 0.0)
            m.orientation.z = m.orientation.w = float('nan')
            ref = on.n._rmr_ref
            on.n._rmr_cb_mag(m)
            check('non-finite magnetometer message is ignored', on.n._rmr_ref == ref)
            on.n._rmr_lock = None                   # breaks the step and the fields
            hs = on.heading_state()
            on.go(1.0, rtk_err=0.0)
            check('failing report -> logged (throttled), RTK heading still taken',
                  len(thr) >= 1 and 'rtk_mag' in thr[0] and on.heading_state() != hs)
            check('... loc_status still published, fields null / false',
                  'owner' in on.loc and on.loc['rtk_mag_diff_deg'] is None
                  and on.loc['rtk_mag_disagree'] is False)
        finally:
            rospy.logwarn_throttle = real_thr
        bad = 0
        for extra in ({}, ON):
            fd = Feed(nt, extra)
            fd.n.imu_msg = Imu()                    # no IMU message yet (stamp 0)
            fd.n.callback_navheading(R.navheading_msg(R.CLOCK[0], 0.2, 1.4))
            fd.n.publish_loc_status()
            try:
                json.loads(fd.n.loc_status_pub.last.data,
                           parse_constant=lambda c: (_ for _ in ()).throw(ValueError(c)))
            except ValueError:
                bad += 1
        check('before the first IMU message: strict JSON, no comparison', bad == 0)

        print('node: the switch in __init__')
        import inspect
        src = inspect.getsource(nt.NavTfNode.__init__)
        check('__init__ calls _rmr_setup() and nothing else of M2',
              src.count('self._rmr_setup()') == 1 and src.count('_rmr_') == 1)
        asked = []
        real_param = rospy.get_param
        rospy.get_param = lambda name, default=None: asked.append(name) or default
        try:
            m = object.__new__(nt.NavTfNode)
            m._rmr_setup()
        finally:
            rospy.get_param = real_param
        check('off -> one parameter read, one attribute',
              asked == ['~rtk_mag_report'] and list(vars(m)) == ['_rmr_enable'])
    finally:
        rospy.Subscriber, rospy.logwarn = real_sub, real_warn

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
