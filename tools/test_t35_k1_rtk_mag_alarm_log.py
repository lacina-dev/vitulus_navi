#!/usr/bin/env python3
"""T35 K1 (part of M2, ~rtk_mag_report): an alarm of M2 leaves a trace. The
node's warnings reach neither /rosout nor its log file (roslaunch's logging
setup inside the node; only INFO gets to the stdout log) and loc_status showed
an alarm only while it lasted. Now every alarm is one INFO line and stays in
loc_status (rtk_mag_alarms: unix time, rtk - mag when raised, the largest
|rtk - mag| while it lasted; the last 5).

Run:  python3 test_t35_k1_rtk_mag_alarm_log.py   (needs rospy - robot env)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402
import test_t35_m2_rtk_mag_report as M2  # noqa: E402

import rospy  # noqa: E402


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-72s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    nt = R.load('navi_transform_t35_k1a')
    infos = []
    rospy.loginfo = lambda *a, **k: infos.append(a[0] % a[1:] if len(a) > 1 else a[0])

    off = M2.Feed(nt)
    loc = off.go(12.0, rtk_err=9.0)
    check('M2 off: no rtk_mag_alarms, no line', 'rtk_mag_alarms' not in loc
          and not [i for i in infos if 'ALARM' in i])

    on = M2.Feed(nt, M2.ON)
    loc = on.go(3.0)
    check('M2 on, headings agree: rtk_mag_alarms empty', loc['rtk_mag_alarms'] == [])
    t_up = R.CLOCK[0]
    on.go(2.0, rtk_err=7.0)
    loc = on.go(4.0, rtk_err=11.0)
    a = loc['rtk_mag_alarms']
    check('RTK 7 then 11 deg off: one alarm, raised at +7.0, largest 11.0, its time',
          len(a) == 1 and a[0]['diff_deg'] == 7.0 and a[0]['max_deg'] == 11.0
          and t_up < a[0]['t'] < t_up + 1.0 and loc['rtk_mag_disagree'] is True, str(a))
    lines = [i for i in infos if 'ALARM' in i]
    check('one INFO line with the number and the size',
          len(lines) == 1 and 'ALARM #1' in lines[0] and '+7.0 deg' in lines[0], str(lines))
    loc = on.go(8.0, rtk_err=0.5)
    check('alarm over: the flag is down, the alarm stays listed, one line says so',
          loc['rtk_mag_disagree'] is False and loc['rtk_mag_alarms'] == a
          and len([i for i in infos if 'alarm #1 over' in i and '11.0' in i]) == 1)
    for k in range(6):
        on.go(2.0, rtk_err=-8.0 - k)
        loc = on.go(8.0, rtk_err=0.0)
    a = loc['rtk_mag_alarms']
    check('six more alarms: the last 5 kept, in order, signed',
          len(a) == 5 and [x['diff_deg'] for x in a] == [-9.0, -10.0, -11.0, -12.0, -13.0]
          and loc['rtk_mag_disagree_count'] == 7
          and len([i for i in infos if 'ALARM #' in i]) == 7, str(a))
    check('heading state untouched by the log (report only)',
          on.n._rmr_enable is True and isinstance(on.n.diff_yaw, float))

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
