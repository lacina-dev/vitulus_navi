#!/usr/bin/env python3
"""T36 N1 (~heading_hold_near_dock): while the robot DOCKS and is within 3.5 m
of the DOCKED waypoint, no RTK heading sample is taken; the gyro carries the
heading. "Docks" = /dock_smach/dock_status 2 (`docking`: the docking program
of dock_smach runs, from its start until the charging contacts report 0). In
every other dock state - 0 docked, 1 undocking (stays 1 after the way out), 3
undocked (mowing near the dock), 4 failed, 5 unknown -, with the state stale
or missing and with the DOCKED waypoint unknown the node is as without the
parameter. No magnetometer.

  - heading_hold_step (pure): start, the ways a hold ends, the fuse (90 s /
    8 m), hysteresis of the zone,
  - the node, off: no state, the RTK heading path untouched,
  - the node, on: a wrong RTK heading while docking in the zone is not taken;
    everywhere else the heading state is EQUAL to off; loc_status fields.

Run:  python3 test_t36_n1_heading_hold_near_dock.py   (needs rospy - robot env)
"""
import json
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402
import test_t35_m2_rtk_mag_report as M2  # noqa: E402

import rospy  # noqa: E402

D = math.radians
ON = {'heading_hold_near_dock': True}
KEYS = ('heading_hold', 'heading_hold_s', 'heading_hold_m', 'heading_hold_end',
        'heading_hold_count', 'heading_hold_fuse_count', 'heading_hold_skipped')


class Feed(M2.Feed):
    """M2's feed (100 Hz IMU, 5 Hz RTK heading, 1 Hz loc_status) with the
    dock: distance from the DOCKED waypoint, the dock state, driving speed."""

    def __init__(self, nt, params=None, dist=2.0, dock=2, v=0.0):
        M2.Feed.__init__(self, nt, params)
        self.dist, self.dock, self.v, self.stale = dist, dock, v, False
        if getattr(self.n, '_hhd_enable', False):
            self.n._hhd_dock_dist = lambda: self.dist

    def go(self, secs, rtk_err=0.0, rate=0.0, rtk=True, acc_deg=1.4, **_):
        n = self.n
        for _k in range(int(round(secs * 100))):
            R.CLOCK[0] += 0.01
            self.k += 1
            self.yaw = R.wrap(self.yaw + rate * 0.01)
            t = R.CLOCK[0]
            n._dock_status_val = self.dock
            if not self.stale:
                n._dock_status_rx_t = t
            n.ekf_wheel_odom_msg.pose.pose.position.x += self.v * 0.01
            n.callback_imu(R.imu_msg(t, R.wrap(self.yaw + 1.0), rate))
            if rtk and self.k % 20 == 7:
                n.callback_relposned(types.SimpleNamespace(flags=M2.FIXED))
                n.callback_navheading(R.navheading_msg(t, R.wrap(self.yaw + D(rtk_err)), acc_deg))
            if self.k % 100 == 0:
                n.publish_loc_status()
                self.loc = json.loads(n.loc_status_pub.last.data)
        return self.loc


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-72s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    nt = R.load('navi_transform_t36_n1')
    step, new = nt.heading_hold_step, nt.heading_hold_new_state

    print('heading_hold_step:')
    st0 = new()
    st, h = step(st0, 10.0, 2, 3.0, (0.0, 0.0))
    check('docking (2), 3.0 m from the dock: hold starts; the state passed in is not modified',
          h and st['hold'] and st['count'] == 1 and st0 == new())
    for status, label in ((0, 'docked'), (1, 'undocking'), (3, 'undocked'), (4, 'failed'),
                          (5, 'unknown'), (None, 'stale / missing')):
        check('dock state %s (%s) at 2 m: no hold' % (status, label),
              step(new(), 10.0, status, 2.0, (0.0, 0.0))[1] is False)
    check('docking, 3.6 m: no hold; DOCKED waypoint unknown (None / nan): no hold',
          not step(new(), 10.0, 2, 3.6, None)[1] and not step(new(), 10.0, 2, None, None)[1]
          and not step(new(), 10.0, 2, float('nan'), None)[1])
    st, h = step(st, 11.0, 2, 3.8, (0.3, 0.0))
    check('running hold, robot swings out to 3.8 m (limit 3.5 + 0.5): hold goes on', h)
    st2, h = step(st, 12.0, 2, 4.1, (0.6, 0.0))
    check('... 4.1 m: over, end left_zone', not h and st2['end'] == 'left_zone'
          and st2['last_s'] == 2.0 and abs(st2['last_m'] - 0.3) < 1e-9)
    st2, h = step(st, 12.0, 0, 0.0, (0.6, 0.0))
    check('charging contacts (state 0): over, end docked', not h and st2['end'] == 'docked')
    st2, h = step(st, 12.0, 4, 1.0, (0.6, 0.0))
    check('docking failed (state 4): over, end dock_state', not h and st2['end'] == 'dock_state')
    st2, h = step(st, 12.0, 2, None, (0.6, 0.0))
    check('DOCKED waypoint lost: over, end dock_unknown', not h and st2['end'] == 'dock_unknown')
    st, t = new(), 0.0
    held = 0
    while t < 200.0:
        t += 1.0
        st, h = step(st, t, 2, 2.0, (0.0, 0.0))
        held += h
    check('fuse by time: standing in the zone, the hold ends after 90 s and stays off',
          89 <= held <= 92 and st['end'] == 'fuse_time' and st['spent'] and st['fuse_count'] == 1
          and st['count'] == 1)
    st2, h = step(st, 201.0, 2, 4.5, (0.0, 0.0))
    st2, h2 = step(st2, 202.0, 2, 3.0, (0.0, 0.0))
    check('... leaving the zone re-arms it: the next entry holds again',
          not h and h2 and st2['count'] == 2 and not st2['spent'])
    st, x, held = new(), 0.0, 0
    for k in range(300):
        x += 0.05
        st, h = step(st, 0.2 * k, 2, 2.0, (x, 0.0))
        held += h
    check('fuse by distance: 8 m driven inside the zone ends the hold (%.1f m)' % st['last_m'],
          st['end'] == 'fuse_dist' and 8.0 < st['last_m'] < 8.2 and st['fuse_count'] == 1)
    st, h = step(new(), 0.0, 2, 2.0, (0.0, 0.0))
    st, h = step(st, 1.0, 2, 2.0, (50.0, 0.0))
    check('a 50 m jump of the odometry is not driving', h and st['path'] == 0.0)
    st, h = step(new(), 0.0, 2, 2.0, (0.0, 0.0), {'radius_m': 1.5})
    check('cfg radius_m 1.5: 2 m is outside', not h)

    print('node:')
    off = Feed(nt)
    check('off (default): no state, nothing in loc_status',
          [a for a in vars(off.n) if a.startswith('_hhd_')] == ['_hhd_enable']
          and off.n._hhd_enable is False)
    src = open(R.NAVI_TF).read()
    check('the gate is a plain branch at the head of offset_navheading_imu (no method swap)',
          'offset_navheading_imu' not in vars(Feed(nt, ON).n)
          and src.count("if getattr(self, '_hhd_enable', False) and self._hhd_holds(msg):") == 1)

    def pair(seq, **kw):
        out = []
        for params in (None, ON):
            f = Feed(nt, params, **kw)
            f.go(8.0)
            f.n0 = f.n.diff_yaw
            for s in seq:
                s = dict(s)
                for key in ('dist', 'dock', 'v', 'stale'):
                    if key in s:
                        setattr(f, key, s.pop(key))
                f.go(**s)
            out.append(f)
        return out

    def moved(f):
        return math.degrees(R.wrap(f.n.diff_yaw - f.n0))

    bad = [dict(secs=6.0, rtk_err=9.0)]
    off, on = pair(bad, dist=5.0)
    check('5 m from the dock, docking: EQUAL to off (heading follows RTK, %.1f deg)' % moved(off),
          on.heading_state() == off.heading_state() and on.loc['heading_hold'] is False
          and on.loc['heading_hold_skipped'] == 0)
    off, on = pair([dict(secs=1.0, dist=2.0)] + bad, dist=5.0)
    check('entering the zone while docking: RTK 9 deg wrong not taken (off %.1f, on %.2f deg)'
          % (moved(off), moved(on)), moved(off) > 3.0 and abs(moved(on)) < 0.05
          and on.loc['heading_hold'] is True and on.loc['heading_hold_count'] == 1
          and on.loc['heading_hold_skipped'] >= 30 and 6.0 <= on.loc['heading_hold_s'] <= 7.5,
          str({k: on.loc[k] for k in KEYS}))
    for status, label in ((1, 'undocking / after the way out'), (3, 'undocked: mowing by the dock'),
                          (0, 'docked'), (4, 'failed'), (5, 'unknown')):
        off, on = pair(bad, dist=2.0, dock=status)
        check('2 m from the dock, dock state %d (%s): EQUAL to off' % (status, label),
              on.heading_state() == off.heading_state() and on.loc['heading_hold'] is False
              and on.loc['heading_hold_count'] == 0)
    res = []
    for params in (None, ON):
        f = Feed(nt, params, dist=2.0, dock=2)
        f.stale = True
        f.n._dock_status_rx_t = R.CLOCK[0] - 10.0      # the last dock state is 10 s old
        f.go(8.0)
        f.go(6.0, rtk_err=9.0)
        res.append(f)
    check('dock state `docking` but 10+ s old (stale limit 5 s): EQUAL to off',
          res[1].heading_state() == res[0].heading_state()
          and res[1].loc['heading_hold'] is False and res[1].loc['heading_hold_count'] == 0)
    f = Feed(nt, ON)
    f.n._hhd_dock_dist = lambda: None
    f.go(8.0)
    f.n0 = f.n.diff_yaw
    f.go(6.0, rtk_err=9.0)
    off, _ = pair(bad)
    check('DOCKED waypoint unknown: EQUAL to off',
          f.heading_state() == off.heading_state() and f.loc['heading_hold'] is False)
    off, on = pair([dict(secs=1.0, dist=2.0), dict(secs=5.0, rtk_err=9.0),
                    dict(secs=1.0, dock=0, dist=0.0, rtk=False)], dist=5.0)
    check('docked: hold over, end docked, duration and path kept in loc_status',
          on.loc['heading_hold'] is False and on.loc['heading_hold_end'] == 'docked'
          and 5.5 <= on.loc['heading_hold_s'] <= 7.0 and abs(moved(on)) < 0.05)
    off, on = pair([dict(secs=1.0, dist=2.0, v=0.2), dict(secs=50.0, rtk_err=9.0)], dist=5.0)
    check('fuse by distance (0.2 m/s for 50 s): hold ends at 8 m, then RTK is taken again',
          on.loc['heading_hold'] is False and on.loc['heading_hold_end'] == 'fuse_dist'
          and on.loc['heading_hold_fuse_count'] == 1 and 8.0 <= on.loc['heading_hold_m'] <= 8.3
          and moved(on) > 3.0, str({k: on.loc[k] for k in KEYS}))
    off, on = pair([dict(secs=1.0, dist=2.0), dict(secs=100.0, rtk_err=9.0)], dist=5.0)
    check('fuse by time (standing 100 s in the zone): ends after 90 s, RTK taken again',
          on.loc['heading_hold_end'] == 'fuse_time' and 89.0 <= on.loc['heading_hold_s'] <= 92.0
          and moved(on) > 3.0)
    off, on = pair([dict(secs=3.0)], dist=5.0)
    check('loc_status: the seven fields, every other field equal to off',
          all(k in on.loc for k in KEYS)
          and sorted(k for k in on.loc if k not in KEYS) == sorted(off.loc))
    n = R.make_node(nt, ON)
    n._dock_anchor_pose = (3.0, 4.0, 0.0)
    n.map_odom_transform.transform.translation.x = 1.0
    n.ekf_wheel_nav_odom_msg.pose.pose.position.x = 2.0
    check('distance from the DOCKED waypoint: 4.0 m; None while it is not known',
          abs(n._hhd_dock_dist() - 4.0) < 1e-9
          and (setattr(n, '_dock_anchor_pose', None) or n._hhd_dock_dist() is None))
    f = Feed(nt, ON, dist=2.0)
    f.go(8.0)
    f.n._hhd_st = None                                  # a fault inside
    b = f.n.diff_yaw
    f.go(6.0, rtk_err=9.0)
    check('a fault inside: the RTK heading is taken as without the parameter',
          abs(math.degrees(R.wrap(f.n.diff_yaw - b))) > 3.0)

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
