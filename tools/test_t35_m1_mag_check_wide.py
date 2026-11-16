#!/usr/bin/env python3
"""T35 M1 (~mag_check_wide): the magnetometer heading watchdog compares during
the undock and the docking manoeuvre too. Before, dock_status 0 / 1 / 2 all
meant `dock`; on the 18 drives of 2026-11-15/16 the status was 1 or 2 for 89 %
of the time off the dock and the watchdog compared 7 % of it. With the
parameter only the robot standing in the dock (dock_status 0), the dock hold
and an unknown dock state are `dock`; the settle time runs from leaving the
dock. REPORT ONLY as the watchdog itself.

Part 1: the pure decision (mag_check_in_dock). Part 2: the node methods on the
stub node of test_t35_mag_check.py, parameter off (as before) and on.
Run:  python3 test_t35_m1_mag_check_wide.py   (needs rospy - robot env)
"""
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_t35_mag_check as T  # noqa: E402

import rospy  # noqa: E402


def main():
    nt = T.load('navi_transform_t35_m1', T.NAVI_TF)
    fails = []

    def check(name, cond):
        print('  %-72s %s' % (name, 'OK' if cond else 'FAIL'))
        if not cond:
            fails.append(name)

    print('pure: what is `dock`')
    f = nt.mag_check_in_dock
    check('off: dock_status 0 / 1 / 2 -> dock, 3 / 4 -> not (as before)',
          [f(v, False, False) for v in (0, 1, 2, 3, 4)] == [True, True, True, False, False])
    check('wide: only dock_status 0 -> dock',
          [f(v, False, False, True) for v in (0, 1, 2, 3, 4)] == [True, False, False, False, False])
    check('dock state unknown -> dock, off and wide',
          f(3, True, False) and f(3, True, False, True) and f(None, True, False, True))
    check('dock hold active -> dock, off and wide',
          f(1, False, True) and f(1, False, True, True) and f(3, False, True, True))
    check('wide, dock_status None but not flagged unknown -> not dock (caller flags None)',
          f(None, False, False, True) is False)

    real_gt, real_sub, real_param = rospy.get_time, rospy.Subscriber, rospy.get_param
    real_logs = rospy.logwarn_throttle, rospy.logwarn, rospy.loginfo
    rospy.get_time = lambda: T.CLOCK[0]
    rospy.Subscriber = lambda *a, **k: types.SimpleNamespace()
    rospy.logwarn_throttle = rospy.logwarn = rospy.loginfo = lambda *a, **k: None
    asked = []

    def params(values):
        def get(name, default=None):
            asked.append(name)
            return values.get(name, default)
        return get
    try:
        print('node: parameter off (default) - as before')
        rospy.get_param = params({})
        n = T.make_node(nt, True)
        check('~mag_check_wide is read, default False',
              '~mag_check_wide' in asked and n._mag_check_wide is False)
        T.drive(n, 12, nav=20, mag=24)
        n._dock_status_val = 1
        st = T.drive(n, 12, nav=20, mag=80)
        check('undocking (dock_status 1) -> dock', st['mag_check'] == 'dock')
        n._dock_status_val = 2
        st = T.drive(n, 12, nav=20, mag=80)
        check('docking (dock_status 2) -> dock', st['mag_check'] == 'dock')
        n._dock_status_val = 3
        n._dock_free_since = T.CLOCK[0]
        st = T.drive(n, 7, nav=20, mag=24)
        check('7 s after the end of the manoeuvre -> settle', st['mag_check'] == 'settle')
        st = T.drive(n, 3, nav=20, mag=24)
        check('10 s after -> compared', st['mag_check'] == 'ok')

        print('node: ~mag_check_wide true')
        rospy.get_param = params({'~mag_check_wide': True})
        n = T.make_node(nt, True)
        check('parameter taken', n._mag_check_wide is True)
        n._dock_status_val = 0
        st = T.drive(n, 12, nav=20, mag=80)
        check('standing in the dock (dock_status 0) -> dock, no diff',
              st['mag_check'] == 'dock' and st['mag_diff_deg'] is None)
        n._dock_status_val = 1
        st = T.drive(n, 7, nav=20, mag=24)
        check('7 s after leaving the dock (dock_status 1) -> settle', st['mag_check'] == 'settle')
        st = T.drive(n, 3, nav=20, mag=24)
        check('10 s after -> compared during the undock (diff 4.0)',
              st['mag_check'] == 'ok' and st['mag_diff_deg'] == 4.0)
        n._dock_status_val = 3
        n._dock_free_since = T.CLOCK[0]          # what callback_dock_status does on 1 -> 3
        st = T.drive(n, 2, nav=20, mag=24)
        check('end of the manoeuvre (1 -> 3) does not start a new settle',
              st['mag_check'] == 'ok')
        n._dock_status_val = 2
        st = T.drive(n, 3, nav=20, mag=24)
        check('way back (dock_status 2) -> compared', st['mag_check'] == 'ok')
        frozen = (n.diff_yaw, n.map_odom_yaw, n.init, n._heading_good_after_acq,
                  n._gps_proven, n._dock_hold_active)
        st = T.drive(n, 4, nav=20, mag=55)
        check('35 deg apart for 4 s while docking -> shown, not yet disagree',
              st['mag_diff_deg'] == 35.0 and st['mag_disagree'] is False)
        st = T.drive(n, 3, nav=20, mag=55)
        check('... for 7 s -> mag_disagree true', st['mag_disagree'] is True)
        check('heading state untouched (report only)',
              frozen == (n.diff_yaw, n.map_odom_yaw, n.init, n._heading_good_after_acq,
                         n._gps_proven, n._dock_hold_active))
        st = T.drive(n, 2, nav=20, mag=55, state='disturbed', cov=1e3)
        check('magnetometer disturbed near the dock -> not_ok, not compared',
              st['mag_check'] == 'not_ok' and st['mag_diff_deg'] is None)
        n._dock_status_val = 0
        st = T.drive(n, 1, nav=20, mag=55)
        check('docked -> dock, disagree dropped',
              st['mag_check'] == 'dock' and st['mag_disagree'] is False)
        n._dock_status_val = 1
        n._dock_hold_active = True
        st = T.drive(n, 12, nav=20, mag=55)
        check('dock hold still active after the edge -> dock', st['mag_check'] == 'dock')
        n._dock_hold_active = False
        st = T.drive(n, 7, nav=20, mag=24)
        check('settle runs from the moment the hold let go', st['mag_check'] == 'settle')
        st = T.drive(n, 3, nav=20, mag=24)
        check('... then compared', st['mag_check'] == 'ok')
        st = T.drive(n, 20, nav=20, mag=24, dock_feed=False)
        check('dock feed stale -> dock (unknown state, as before)', st['mag_check'] == 'dock')
        n2 = T.make_node(nt, True)
        n2._dock_status_val = None
        n2._dock_status_rx_t = 0.0
        st = T.drive(n2, 12, nav=20, mag=80, dock_feed=False)
        check('dock_status never received -> dock', st['mag_check'] == 'dock')

        print('node: the switch')
        del asked[:]
        rospy.get_param = params({})
        m = object.__new__(nt.NavTfNode)
        m._mag_check_setup()
        check('watchdog off -> ~mag_check_wide not even read, no state',
              asked == ['~mag_check_enable'] and list(vars(m)) == ['_mag_check_enable'])
    finally:
        rospy.get_time, rospy.Subscriber, rospy.get_param = real_gt, real_sub, real_param
        rospy.logwarn_throttle, rospy.logwarn, rospy.loginfo = real_logs

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
