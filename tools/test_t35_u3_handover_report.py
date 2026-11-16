#!/usr/bin/env python3
"""T35 U3 (~handover_report): /nav_tf/loc_status says how much of the GNSS
heading hand-over is left - map_odom_yaw_deg and heading_handover_done. After
the undock the dock heading sits in map_odom_yaw and is walked to 0 about the
robot; the position of the antenna hardly shows it (1-2 cm), so a consumer
that waits for the take-over to finish (dock_smach ~undock_waits_adaptive)
needs the number itself. REPORT ONLY.

Part 1: the pure fields. Part 2: publish_loc_status of the stub node of
test_t35_mag_check.py, parameter off (no field) and on; slew_yaw_level walks
map_odom_yaw to 0 and the report follows.
Run:  python3 test_t35_u3_handover_report.py   (needs rospy - robot env)
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import test_t35_mag_check as T  # noqa: E402

import rospy  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402

KEYS = ('map_odom_yaw_deg', 'heading_handover_done')


def main():
    nt = T.load('navi_transform_t35_u3', T.NAVI_TF)
    fails = []

    def check(name, cond):
        print('  %-72s %s' % (name, 'OK' if cond else 'FAIL'))
        if not cond:
            fails.append(name)

    print('pure: handover_report_fields')
    f = nt.handover_report_fields
    check('0 -> 0.0 deg, done', f(0.0) == {'map_odom_yaw_deg': 0.0, 'heading_handover_done': True})
    check('-15.7 deg (fresh start in the dock) -> -15.7, not done',
          f(math.radians(-15.7)) == {'map_odom_yaw_deg': -15.7, 'heading_handover_done': False})
    check('0.004 deg left -> shown as 0.0 but NOT done (the georef waits for exactly 0)',
          f(math.radians(0.004)) == {'map_odom_yaw_deg': 0.0, 'heading_handover_done': False})
    check('nan / None / bool -> None, not done',
          all(f(v) == {'map_odom_yaw_deg': None, 'heading_handover_done': False}
              for v in (float('nan'), float('inf'), None, True)))

    real_gt, real_param = rospy.get_time, rospy.get_param
    real_logs = rospy.logwarn_throttle, rospy.logwarn, rospy.loginfo
    rospy.get_time = lambda: T.CLOCK[0]
    rospy.logwarn_throttle = rospy.logwarn = rospy.loginfo = lambda *a, **k: None
    try:
        def status(n):
            del n.published[:]
            n.publish_loc_status()
            return json.loads(n.published[-1])

        print('node: parameter off (default) - as before')
        n = T.make_node(nt, False)
        n.map_odom_yaw = math.radians(6.1)
        st = status(n)
        check('loc_status published, none of the fields', 'owner' in st
              and not [k for k in KEYS if k in st])
        base_keys = sorted(st)

        print('node: ~handover_report true')
        n._hor_enable = True
        st = status(n)
        check('hand-over in progress: 6.1 deg, not done',
              st['map_odom_yaw_deg'] == 6.1 and st['heading_handover_done'] is False)
        check('nothing else in loc_status changed',
              sorted(k for k in st if k not in KEYS) == base_keys)
        frozen = (n.diff_yaw, n.map_odom_yaw, n.init, n._gps_proven)
        status(n)
        check('report only: heading state untouched',
              frozen == (n.diff_yaw, n.map_odom_yaw, n.init, n._gps_proven))
        # the node's own levelling walks it to 0 (T7d2), the report follows
        n._yaw_level = True
        n._heading_slew_dps = 5.0
        n.map_odom_transform = TransformStamped()
        n._odom_base_xy = lambda: (1.0, 2.0)
        seen = []
        for i in range(30 * 60):
            T.CLOCK[0] += 1.0 / 30.0
            n.slew_yaw_level()
            if i % 30 == 0:
                seen.append(status(n))
            if n.map_odom_yaw == 0.0:
                break
        st = status(n)
        check('levelling (5 deg/s, 0.2 deg per cycle): 6.1 deg gone in ~1 s',
              n.map_odom_yaw == 0.0 and 0.9 <= (i + 1) / 30.0 <= 1.5)
        check('never done before 0',
              len(seen) >= 1 and not any(x['heading_handover_done'] for x in seen))
        check('at 0: 0.0 deg and done',
              st['map_odom_yaw_deg'] == 0.0 and st['heading_handover_done'] is True)

        print('node: the switch')
        src = open(T.NAVI_TF).read()
        check("one parameter read: ~handover_report, default False",
              src.count("rospy.get_param('~handover_report', False)") == 1)
        check('fields added only behind _hor_enable',
              src.count('handover_report_fields(self.map_odom_yaw)') == 1
              and "if getattr(self, '_hor_enable', False):       # T35 U3" in src)
        m = object.__new__(nt.NavTfNode)
        check('node built without __init__ (older tests): off',
              getattr(m, '_hor_enable', False) is False)
    finally:
        rospy.get_time, rospy.get_param = real_gt, real_param
        rospy.logwarn_throttle, rospy.logwarn, rospy.loginfo = real_logs

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d\n  ' % len(fails) + '\n  '.join(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
