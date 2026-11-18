#!/usr/bin/env python3
"""T36 Q1 (~takeover_wait_still): after an undock GPS takes the position over
only once the robot stands.

dock_smach drives the robot from the dock to the UNDOCK waypoint and waits for
the GPS there. When the RTK heading came before the robot arrived, GPS took
the position over in the middle of that drive and the map pose travelled
40-110 cm instead of 15-27 cm (night series 2026-11-18). With the switch the
takeover waits - during the undock program only - until the wheels have stood
still for 0.5 s, at most 15 s.

  - takeover_wait_still_step / _fields (pure),
  - the node (real __init__, offline): _gps_stable() while the robot drives
    out, after it stops, the cap, a robot that already stands, status 3 / 2
    (mission drive, docking: never held), stale wheel odometry, the next
    undock; parameter off = _gps_stable() as on main, no field, no state.
Only this parameter is switched on.

Run:  python3 test_t36_q1_takeover_wait_still.py   (needs rospy - robot env)
"""
import json
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

ON = {'takeover_wait_still': True}
T = 5000.0


class Sim(object):
    def __init__(self, nt, params):
        R.CLOCK[0] = T
        n = R.make_node(nt, params)
        n._gps_proven = True
        n._gps_good_since = T - 60.0
        n._heading_good_after_acq = True
        self.n = n
        self.v = 0.0

    def at(self, t, v=None, status=None, wheel_age=0.01, gps=True):
        R.CLOCK[0] = T + t
        if v is not None:
            self.v = v
        if status is not None:
            self.n.callback_dock_status(types.SimpleNamespace(data=status))
            if status != 0 and gps:
                # the undock edge re-arms the GPS probation; here the GPS has proved
                # itself in the first metre and is ready from now on
                self.n._gps_proven = True
                self.n._gps_good_since = T - 60.0
                self.n._heading_good_after_acq = True
        self.n.callback_wheel_odom(R.odom_msg(T + t - wheel_age, 0.0, 0.0, 0.0, 0.0, self.v, 0.0))
        return self.n._gps_stable()

    def loc(self):
        self.n.publish_loc_status()
        return json.loads(self.n.loc_status_pub.last.data)


def run_pure(nt, check):
    new, step, fields = (nt.takeover_wait_still_new_state, nt.takeover_wait_still_step,
                         nt.takeover_wait_still_fields)
    st = new()
    check('mission drive (status 3), robot moving, GPS ready: never held',
          not step(st, 10.0, 3, True, 0.25, 0.0, 0.01) and st['n'] == 0)
    check('undock program starts, GPS not ready yet: nothing held',
          not step(st, 20.0, 1, False, 0.25, 0.0, 0.01) and st['armed'])
    check('GPS ready while the robot drives: held',
          step(st, 30.0, 1, True, 0.25, 0.0, 0.01) and st['event'] == 'hold'
          and fields(st)['takeover_wait_active'])
    st['event'] = ''
    check('robot stops: still held for the first 0.4 s at rest',
          step(st, 33.0, 1, True, 0.0, 0.0, 0.01) and step(st, 33.4, 1, True, 0.01, 0.0, 0.01))
    check('at rest 0.5 s: released, the wait (3.5 s) is reported',
          not step(st, 33.5, 1, True, 0.0, 0.0, 0.01) and st['event'] == 'still'
          and st['last_s'] == 3.5 and not fields(st)['takeover_wait_active'])
    check('the same undock, robot drives again: not held a second time',
          not step(st, 36.0, 1, True, 0.25, 0.0, 0.01) and st['n'] == 1)
    st = new()
    step(st, 0.0, 1, True, 0.25, 0.0, 0.01)
    step(st, 2.0, 1, True, 0.0, 0.0, 0.01)
    check('a short stop (0.3 s) and on: the rest clock starts anew',
          step(st, 2.3, 1, True, 0.25, 0.0, 0.01) and step(st, 4.0, 1, True, 0.0, 0.0, 0.01)
          and step(st, 4.4, 1, True, 0.0, 0.0, 0.01) and not step(st, 4.5, 1, True, 0.0, 0.0, 0.01))
    st = new()
    step(st, 0.0, 1, True, 0.25, 0.0, 0.01)
    check('robot never stops: held 14.9 s, released at 15 s, timeout counted',
          step(st, 14.9, 1, True, 0.25, 0.0, 0.01) and not step(st, 15.0, 1, True, 0.25, 0.0, 0.01)
          and st['timeouts'] == 1 and st['event'] == 'timeout' and st['last_s'] == 15.0)
    st = new()
    step(st, 0.0, 1, False, 0.0, 0.0, 0.01)
    check('robot stands when GPS gets ready: no hold at all',
          not step(st, 5.0, 1, True, 0.0, 0.0, 0.01) and st['n'] == 0 and st['last_s'] is None)
    st = new()
    step(st, 0.0, 1, True, 0.25, 0.0, 0.01)
    check('turning on the spot (w 0.3 rad/s) is not standing',
          step(st, 1.0, 1, True, 0.0, 0.3, 0.01) and step(st, 2.0, 1, True, 0.0, 0.3, 0.01))
    check('undock program ends (status 3) while held: released at once',
          not step(st, 3.0, 3, True, 0.25, 0.0, 0.01) and st['event'] == 'left' and st['last_s'] == 3.0)
    st = new()
    step(st, 0.0, 1, True, 0.25, 0.0, 0.01)
    check('wheel odometry 2 s old (speed not known): released, nothing held',
          not step(st, 1.0, 1, True, 0.25, 0.0, 2.0) and not st['holding'])
    st = new()
    check('wheel stamp 8 ms ahead of the clock: speed still known, held',
          step(st, 0.0, 1, True, 0.25, 0.0, -0.008) and step(st, 0.0, 1, True, 0.25, 0.0, -0.008)
          and st['holding'])
    st = new()
    step(st, 0.0, 1, True, 0.25, 0.0, 0.01)
    check('GPS not ready again while held: stays held, the clock keeps running',
          step(st, 1.0, 1, False, 0.25, 0.0, 0.01) and step(st, 2.0, 1, True, 0.25, 0.0, 0.01)
          and not step(st, 15.0, 1, True, 0.25, 0.0, 0.01))
    st = new()
    step(st, 0.0, 1, True, 0.0, 0.0, 0.01)
    step(st, 0.6, 1, True, 0.0, 0.0, 0.01)
    step(st, 100.0, 3, True, 0.3, 0.0, 0.01)
    check('next undock (status 1 again after 3): armed again',
          step(st, 300.0, 1, True, 0.25, 0.0, 0.01) and st['armed'] and st['holding'])


def run_node(nt, check):
    off = Sim(nt, {})
    r = [off.at(0.0, 0.25, 1), off.at(1.0), off.at(2.0, 0.0), off.at(3.0)]
    check('off: _gps_stable() True all along (as on main), no field, no state',
          r == [True] * 4 and not hasattr(off.n, '_tws_st')
          and not any(k.startswith('takeover_wait') for k in off.loc()))
    on = Sim(nt, ON)
    check('on, docked: False (as before)', not on.at(-5.0, 0.0, 0))
    r = [on.at(0.0, 0.06, 1), on.at(5.0, 0.25), on.at(9.0, 0.25)]
    check('on, driving out of the dock with GPS ready: _gps_stable() False', r == [False] * 3)
    j = on.loc()
    check('on: loc_status owner is not gps, takeover_wait_active', j['owner'] != 'gps'
          and j['takeover_wait_active'] and j['takeover_wait_n'] == 1)
    r = [on.at(10.0, 0.0), on.at(10.4), on.at(10.5), on.at(11.0)]
    check('on: robot stops - False for 0.5 s, then True', r == [False, False, True, True])
    j = on.loc()
    check('on: owner gps, the wait of 10.5 s reported, no timeout',
          j['owner'] == 'gps' and not j['takeover_wait_active'] and j['takeover_wait_last_s'] == 10.5
          and j['takeover_wait_timeouts'] == 0)
    check('on: mission drive after the undock (status 3, moving): True',
          on.at(20.0, 0.3, 3) and on.at(21.0))
    check('on: docking (status 2, moving): True', on.at(200.0, 0.2, 2))
    on.at(260.0, 0.0, 0)
    r = [on.at(300.0, 0.25, 1), on.at(310.0), on.at(314.9), on.at(315.0), on.at(316.0)]
    check('on: next undock, robot never stops - held 15 s, then True; timeout counted',
          r == [False, False, False, True, True] and on.loc()['takeover_wait_timeouts'] == 1)
    cap = Sim(nt, dict(ON, takeover_wait_max_s=3.0, takeover_wait_still_s=1.0))
    r = [cap.at(0.0, 0.25, 1), cap.at(2.9), cap.at(3.0)]
    check('parameters are taken (cap 3 s)', r == [False, False, True])
    st2 = Sim(nt, ON)
    st2.n._gps_good_since = None            # GPS not ready during the drive out
    st2.at(0.0, 0.25, 1, gps=False)
    r0 = [st2.at(5.0, 0.25), st2.at(20.0, 0.0), st2.at(25.0)]
    st2.n._gps_good_since = T - 60.0        # RTK comes 5 s after the robot stopped at UNDOCK
    r = [st2.at(25.1), st2.at(26.0)]
    check('on: GPS gets ready when the robot already stands at UNDOCK: no hold, True at once',
          r0 == [False] * 3 and r == [True, True] and st2.loc()['takeover_wait_n'] == 0)


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-76s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    R.install_stubs()
    nt = R.load('navi_transform_t36_q1')
    print('takeover_wait_still_* (pure):')
    run_pure(nt, check)
    print('node, _gps_stable() and loc_status:')
    run_node(nt, check)
    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
