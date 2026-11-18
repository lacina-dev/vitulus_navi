#!/usr/bin/env python3
"""T36 P3 (~dock_offset_report; first name ~dock_anchor_offset_report): /nav_tf/loc_status and the log say how
far the DOCKED waypoint is from where the GPS has the docked robot, from every
undock (pose the dock left against the pose after the GPS takeover) and every
docking (pose the robot arrived with against DOCKED). REPORT ONLY: nothing is
written to the map, to map->odom or to the EKF.

  - dock_offset_frame / _undock_sample / _dock_sample / _add / _fields (pure),
  - the node (real __init__, offline): a waypoint 5.5 cm short along the robot
    axis and turned 1.7 deg, three rounds of release -> takeover -> georef
    settled -> docked edge with a lateral drift of the approach: the estimate
    is the built offset, the lateral part is flagged as not an offset; a
    takeover that never settles, a robot that drove on, an absurd sample;
    parameter off = no field, no state; on = map->odom, map_odom_yaw and the
    set_pose calls untouched.
Only this parameter is switched on.

Run:  python3 test_t36_p3_dock_offset_report.py   (needs rospy - robot env)
"""
import json
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402
import test_t36_f2_anchor_mode_report as F2  # noqa: E402

ON = {'dock_offset_report': True}
D = math.radians
ANCHOR = (5.0, -8.0, D(-15.7))          # DOCKED as saved
OFF_X, OFF_YAW = 0.055, -1.7            # the truth against it (robot axis, deg)


def rot(x, y, a):
    return x * math.cos(a) - y * math.sin(a), x * math.sin(a) + y * math.cos(a)


class Sim(object):
    """Feeds the node the two EKF poses and its own map->odom state."""

    def __init__(self, nt, params):
        F2.install()
        R.CLOCK[0] = 1000.0
        self.calls = []
        F2.SINK[0] = lambda req: self.calls.append(req)
        n = R.make_node(nt, params)
        n._dock_anchor_pose = ANCHOR
        n._dock_anchor_fetch_t = 1000.0
        n._map_odom_placed = True
        self.n = n
        self.wheel = [0.0, 0.0, 0.3]
        self.set(ANCHOR, 0.0)

    def set(self, map_pose, map_odom_yaw, mo_xy=(1.0, 2.0)):
        """Node state so that the robot's map pose is map_pose."""
        n = self.n
        n.map_odom_transform.transform.translation.x = mo_xy[0]
        n.map_odom_transform.transform.translation.y = mo_xy[1]
        n.map_odom_yaw = map_odom_yaw
        ox, oy = rot(map_pose[0] - mo_xy[0], map_pose[1] - mo_xy[1], -map_odom_yaw)
        n.callback_ekf_wheel_nav_odom(R.odom_msg(R.CLOCK[0], ox, oy, 0.0,
                                                 map_pose[2] - map_odom_yaw, 0.0, 0.0))
        n.callback_ekf_wheel_odom(R.odom_msg(R.CLOCK[0], self.wheel[0], self.wheel[1], 0.0,
                                             self.wheel[2], 0.0, 0.0))

    def t(self, dt):
        R.CLOCK[0] += dt

    def status(self, v):
        self.n.callback_dock_status(types.SimpleNamespace(data=v))

    def round(self, drift_y=0.03, settle=True, drive_on=0.0, absurd=False, target=None):
        """Docked with the pose = DOCKED -> release -> drive out 3 m -> GPS
        takeover -> heading hand-over and georef (the pose becomes the truth)
        -> back and the docked edge (arrived pose = truth + lateral drift)."""
        n = self.n
        self.status(0)
        n._dock_hold_active = True
        self.set(ANCHOR, D(12.0))
        self.t(5.0)
        self.status(1)                          # undock
        n.dock_anchor_hold()                    # the hold lets go
        # 3 m straight out: dock pose + odometry (held heading = DOCKED yaw)
        self.wheel[0] += 3.0 * math.cos(self.wheel[2])
        self.wheel[1] += 3.0 * math.sin(self.wheel[2])
        fx, fy = rot(3.0, 0.0, ANCHOR[2])
        held = (ANCHOR[0] + fx, ANCHOR[1] + fy, ANCHOR[2])
        self.t(40.0)
        self.set(held, D(12.0))
        n._anchor_owned = False
        n._dor_takeover()                       # loop3sec: resume tick
        # the truth: waypoint offset + the same 3 m along the TRUE heading
        ty = ANCHOR[2] + D(OFF_YAW)
        ox, oy = rot(OFF_X + (0.9 if absurd else 0.0), 0.0, ANCHOR[2])
        gx, gy = rot(3.0, 0.0, ty)
        true = (ANCHOR[0] + ox + gx, ANCHOR[1] + oy + gy, ty)
        if drive_on:
            self.wheel[0] += drive_on * math.cos(self.wheel[2])
            self.wheel[1] += drive_on * math.sin(self.wheel[2])
            mx, my = rot(drive_on, 0.0, ty)
            true = (true[0] + mx, true[1] + my, ty)
        self.t(2.0)
        n._anchor_owned = True
        self.set(true, D(4.0))                  # hand-over still running
        n._dor_tick()
        if settle:
            self.t(3.0)
            self.set(true, 0.0)
            n._georef_last_t = R.CLOCK[0]
            n._georef_target = None
            for _ in range(35 if target != 'far' else 700):    # 3.5 s of 10 Hz ticks
                self.t(0.1)
                if target is not None:
                    # navi_man's map_coords set the georef target again and again:
                    # 'near' = where map->odom is (georef done), 'far' = 5 cm off
                    m = n.map_odom_transform.transform.translation
                    n._georef_target = (m.x + (0.003 if target == 'near' else 0.05), m.y, 0.0)
                n._dor_tick()
            n._georef_target = None
        else:
            for _ in range(700):
                self.t(0.1)
                n._dor_tick()
        # back in the dock: the truth of the docked robot + drift of the approach
        self.status(2)
        dx, dy = rot(OFF_X, drift_y, ANCHOR[2])
        self.t(60.0)
        self.set((ANCHOR[0] + dx, ANCHOR[1] + dy, ANCHOR[2] + D(OFF_YAW)), 0.0)
        self.status(0)
        n.publish_loc_status()
        return json.loads(n.loc_status_pub.last.data)


def run_pure(nt, check):
    fr, us, ds, new, add, fields = (nt.dock_offset_frame, nt.dock_offset_undock_sample,
                                    nt.dock_offset_dock_sample, nt.dock_offset_new_state,
                                    nt.dock_offset_add, nt.dock_offset_fields)
    x, y = fr(0.0, 0.1, D(90.0))
    check('frame: map +y with DOCKED heading 90 deg = +x of the waypoint',
          abs(x - 0.1) < 1e-12 and abs(y) < 1e-12)
    s = us(D(20.0), (1.0, 1.0, D(50.0)), (0.0, 0.0, D(30.0)),
           (1.06, 1.0, D(48.0)), (0.0, 0.0, D(30.0)), 0.0)
    check('undock, robot standing: 6 cm, heading taken back 2 deg, moved 0',
          abs(s[0] - 0.06) < 1e-9 and abs(s[1]) < 1e-9 and abs(s[2] + 2.0) < 1e-9 and s[3] == 0.0)
    th = D(18.0)
    s = us(D(20.0), (1.0, 1.0, D(50.0)), (0.0, 0.0, D(30.0)),
           (1.06 + math.cos(th + D(30.0)), 1.0 + math.sin(th + D(30.0)), D(48.0)),
           (math.cos(D(30.0)), math.sin(D(30.0)), D(30.0)), 0.0)
    check('undock, robot drove 1 m meanwhile: the same 6 cm (wheel path taken out)',
          abs(s[0] - 0.06) < 1e-9 and abs(s[1]) < 1e-9 and abs(s[3] - 1.0) < 1e-9)
    s = ds((4.9, -8.0, D(-17.0)), (5.0, -8.0, D(-15.0)))
    check('dock: DOCKED - arrived pose in the DOCKED frame, yaw +2 deg',
          abs(s[0] - 0.1 * math.cos(D(15.0))) < 1e-9 and abs(s[1] - 0.1 * math.sin(D(15.0))) < 1e-9
          and abs(s[2] - 2.0) < 1e-9)
    st = new()
    f = fields(st)
    check('nothing yet: counts 0, everything None',
          f['dock_offset_n_undock'] == 0 and f['dock_offset_est_x_m'] is None
          and f['dock_offset_undock'] is None and f['dock_offset_last'] is None)
    for v in ((0.05, -0.02, -1.5), (0.06, -0.04, -1.9), (0.30, -0.03, -1.6)):
        add(st, 'und', v)
    for v in ((-0.05, -0.07), (-0.06, -0.05)):
        add(st, 'dock', (v[0], v[1], 1.8))
    f = fields(st)
    check('3 undocks, 2 dockings: medians reported, no estimate yet (needs 3 at each end)',
          f['dock_offset_undock'] == [0.06, -0.03, -1.6] and f['dock_offset_dock'] == [-0.055, -0.06, 1.8]
          and f['dock_offset_est_x_m'] is None)
    add(st, 'dock', (-0.07, -0.06, 2.0))
    f = fields(st)
    check('3 + 3: estimate = half the difference of the medians (an outlier does not move it)',
          f['dock_offset_est_x_m'] == 0.06 and f['dock_offset_est_yaw_deg'] == -1.7
          and f['dock_offset_y_offset_like'] is False, str(f['dock_offset_est_x_m']))
    check('sample over 0.5 m or 10 deg: rejected, counted',
          not add(st, 'und', (0.6, 0.0, 0.0)) and not add(st, 'dock', (0.0, 0.0, 11.0))
          and fields(st)['dock_offset_rejected'] == 2 and len(st['und']) == 3)
    for k in range(40):
        add(st, 'und', (0.01, 0.0, 0.0))
    check('at most 30 samples kept per end', len(st['und']) == 30)


def run_node(nt, check):
    off = Sim(nt, {})
    j = off.round()
    check('off: no dock_offset field, no state',
          not any(k.startswith('dock_offset') for k in j) and not hasattr(off.n, '_dor_st'))
    on = Sim(nt, ON)
    js = [on.round(drift_y=0.03 + 0.01 * k) for k in range(3)]
    j = js[-1]
    print('    after 3 rounds: undock %s, dock %s, estimate x %s m, yaw %s deg' % (
        j['dock_offset_undock'], j['dock_offset_dock'], j['dock_offset_est_x_m'],
        j['dock_offset_est_yaw_deg']))
    check('after 1 round: one sample at each end, no estimate',
          js[0]['dock_offset_n_undock'] == 1 and js[0]['dock_offset_n_dock'] == 1
          and js[0]['dock_offset_est_x_m'] is None)
    check('3 rounds: estimate = the built offset (%.3f m, %.1f deg)' % (OFF_X, OFF_YAW),
          abs(j['dock_offset_est_x_m'] - OFF_X) <= 0.002
          and abs(j['dock_offset_est_yaw_deg'] - OFF_YAW) <= 0.02)
    check('undock end +offset, dock end -offset',
          abs(j['dock_offset_undock'][0] - OFF_X) <= 0.002 and abs(j['dock_offset_dock'][0] + OFF_X) <= 0.002
          and abs(j['dock_offset_undock'][2] - OFF_YAW) <= 0.02 and abs(j['dock_offset_dock'][2] + OFF_YAW) <= 0.02)
    check('the lateral part (heading error x 3 m out, drift in) is flagged as not an offset',
          j['dock_offset_y_offset_like'] is False and j['dock_offset_undock'][1] < 0
          and j['dock_offset_dock'][1] < 0)
    check('on: no set_pose sent, map->odom and map_odom_yaw are what the test set',
          not on.calls and on.n.map_odom_yaw == 0.0
          and on.n.map_odom_transform.transform.translation.x == 1.0
          and on.n.map_odom_transform.transform.translation.y == 2.0)
    n0 = j['dock_offset_n_undock']
    j = on.round(settle=False)
    check('takeover that never settles (hand-over stuck 70 s): no undock sample, rejected + 1',
          j['dock_offset_n_undock'] == n0 and j['dock_offset_rejected'] == 1)
    j = on.round(drive_on=2.0)
    check('robot drove 2 m between takeover and settle: no undock sample, rejected + 1',
          j['dock_offset_n_undock'] == n0 and j['dock_offset_rejected'] == 2)
    j = on.round(drive_on=0.8)
    check('robot drove 0.8 m meanwhile: sample taken, estimate unchanged',
          j['dock_offset_n_undock'] == n0 + 1 and abs(j['dock_offset_est_x_m'] - OFF_X) <= 0.002)
    j = on.round(absurd=True)
    check('GPS 0.95 m from DOCKED at the undock: not counted (bounds), rejected + 1',
          j['dock_offset_n_undock'] == n0 + 1 and j['dock_offset_rejected'] == 3)
    j = on.round(target='near')
    check('georef target set again every tick, 3 mm from map->odom (live map_coords): sample taken',
          j['dock_offset_n_undock'] == n0 + 2 and abs(j['dock_offset_est_x_m'] - OFF_X) <= 0.002)
    j = on.round(target='far')
    check('georef still 5 cm from its target for 70 s: no undock sample, rejected + 1',
          j['dock_offset_n_undock'] == n0 + 2 and j['dock_offset_rejected'] == 4)
    old = Sim(nt, {'dock_anchor_offset_report': True})
    check('the first parameter name still switches the report on',
          'dock_offset_n_undock' in old.round())
    on.n._dock_anchor_pose = None
    nd = j['dock_offset_n_dock']
    j = on.round()
    check('DOCKED waypoint unknown: nothing measured',
          j['dock_offset_n_dock'] == nd and j['dock_offset_n_undock'] == n0 + 2)


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-76s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    F2.install()
    nt = R.load('navi_transform_t36_p3')
    print('dock_offset_* (pure):')
    run_pure(nt, check)
    print('node, built rounds (release -> takeover -> settled -> docked edge):')
    run_node(nt, check)
    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
