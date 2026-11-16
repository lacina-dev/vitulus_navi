#!/usr/bin/env python3
"""T36 H2 (~anchor_freeze_report): /nav_tf/loc_status says when the robot
drives blind - on a heading anchor or a position anchor that has not advanced
for more than after_s, or on dead reckoning - away from the dock. REPORT ONLY.

What loc_status already carried: anchor_age_s / anchor_frozen (frozen > 5 s
with owner gps, moving or not), heading_anchor_age_s / heading_anchor_frozen
(only while heading samples keep failing the window - not when they stop
coming), owner. The new fields add what cannot be read from one message: the
distance driven since the anchor advanced, the time on dead reckoning, the
dock as the place where all of it is normal, one flag with the reason.

  - anchor_freeze_step (pure): each reason, the distance condition, the dock
    (also unknown), absolute owners, the GPS position written directly
    (bootstrap / emergency set_pose - not blind), a heading never verified, a
    robot that stopped, the episode counter,
  - the node's publish_loc_status: parameter off = no new field; on = the
    fields, every other field equal to off.

Run:  python3 test_t36_h2_anchor_freeze_report.py   (needs rospy - robot env)
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

ON = {'anchor_freeze_report': True}
FIELDS = ('loc_blind', 'loc_blind_near_dock', 'loc_blind_reason', 'loc_blind_s',
          'loc_blind_m', 'loc_blind_dock_m', 'loc_blind_count')


def drive(nt, steps, cfg=None, v=0.3, far=20.0):
    """steps: list of (seconds, owner, heading_age0, anchor_age0, kw) - each
    second one step; an age given as ('grow', a0) grows 1 s per step, a number
    stays. kw: v (m/s), in_dock, dock (distance, m), direct (GPS written
    directly). Returns the fields of every step."""
    st = nt.anchor_freeze_new_state()
    out, t, x = [], 1000.0, 0.0

    def age(a, k):
        if isinstance(a, tuple):
            return a[1] + k
        return a
    for secs, owner, h, p, kw in steps:
        for k in range(int(secs)):
            t += 1.0
            x += kw.get('v', v) * 1.0
            out.append(nt.anchor_freeze_step(
                st, t, (x, 0.0), owner, age(h, k), age(p, k), kw.get('in_dock', False),
                kw.get('dock', far), kw.get('direct', False), cfg))
    return out


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-72s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    nt = R.load('navi_transform_t36_h2')
    G = ('grow', 0.2)

    print('anchor_freeze_step:')
    o = drive(nt, [(40, 'gps', 0.2, 0.1, {})])
    check('GPS owns, both anchors advance: never blind',
          not any(r['loc_blind'] or r['loc_blind_near_dock'] for r in o)
          and o[-1] == {'loc_blind': False, 'loc_blind_near_dock': False, 'loc_blind_reason': '',
                        'loc_blind_s': 0.0, 'loc_blind_m': 0.0, 'loc_blind_dock_m': 20.0,
                        'loc_blind_count': 0})
    o = drive(nt, [(5, 'gps', 0.2, 0.1, {}), (30, 'gps', G, 0.1, {})])
    first = [i for i, r in enumerate(o) if r['loc_blind']][0] - 5
    check('heading anchor frozen while driving: blind once it is older than 10 s',
          first == 10 and o[-1]['loc_blind_reason'] == 'heading_anchor'
          and o[-1]['loc_blind_s'] == 29.2 and abs(o[-1]['loc_blind_m'] - 8.7) < 0.01
          and o[-1]['loc_blind_count'] == 1, 'after %d s, %s' % (first, o[-1]))
    o = drive(nt, [(5, 'gps', 0.2, 0.1, {}), (30, 'gps', 0.2, G, {})])
    check('position anchor frozen while driving (owner gps): position_anchor',
          o[-1]['loc_blind'] and o[-1]['loc_blind_reason'] == 'position_anchor'
          and not o[5 + 9]['loc_blind'] and o[5 + 10]['loc_blind'])
    o = drive(nt, [(5, 'gps', 0.2, 0.1, {}), (30, 'gps', G, G, {})])
    check('both: reason heading_anchor+position_anchor',
          o[-1]['loc_blind_reason'] == 'heading_anchor+position_anchor')
    o = drive(nt, [(5, 'gps', 0.2, 0.1, {}), (30, 'dr', 0.2, None, {})])
    check('owner dr while driving: dead_reckoning after 10 s without a position source',
          o[-1]['loc_blind_reason'] == 'dead_reckoning' and not o[5 + 8]['loc_blind']
          and o[5 + 9]['loc_blind'] and o[5 + 9]['loc_blind_s'] == 10.1
          and o[-1]['loc_blind_s'] == 30.1)
    o = drive(nt, [(5, 'gps', 0.2, 0.1, {}), (60, 'gps', G, G, {'v': 0.0})])
    check('anchors frozen 60 s but the robot stands: not blind',
          not any(r['loc_blind'] for r in o))
    o = drive(nt, [(5, 'gps', 0.2, 0.1, {}), (30, 'gps', G, 0.1, {'v': 0.0}),
                   (3, 'gps', ('grow', 30.2), 0.1, {})])
    check('... and blind after the first 0.5 m driven on it',
          not o[5 + 30]['loc_blind'] and o[5 + 31]['loc_blind'] and o[-1]['loc_blind_s'] > 30)
    o = drive(nt, [(5, 'gps', 0.2, 0.1, {}), (20, 'gps', G, 0.1, {}), (5, 'gps', 0.2, 0.1, {}),
                   (20, 'gps', G, 0.1, {})])
    check('the anchor advances: flag off, distance from zero, a second episode counted',
          o[24]['loc_blind'] and not o[25]['loc_blind'] and o[25]['loc_blind_m'] == 0.0
          and not o[30 + 9]['loc_blind'] and o[-1]['loc_blind'] and o[-1]['loc_blind_count'] == 2)
    o = drive(nt, [(5, 'gps', 0.2, 0.1, {}), (30, 'dr', G, None, {'dock': 3.0})])
    check('the same within 6 m of the dock: loc_blind_near_dock, not loc_blind, not counted',
          o[-1]['loc_blind_near_dock'] and not any(r['loc_blind'] for r in o)
          and o[-1]['loc_blind_reason'] == 'heading_anchor+dead_reckoning'
          and o[-1]['loc_blind_count'] == 0)
    o = drive(nt, [(5, 'dock', None, None, {'in_dock': True, 'v': 0.0, 'dock': 0.0}),
                   (25, 'dr', ('grow', 80.0), None, {'dock': 3.0}),
                   (30, 'dr', ('grow', 105.0), None, {'dock': 9.0})])
    check('undock on dr, still no GPS past 6 m: near_dock, then blind (dead_reckoning)',
          o[29]['loc_blind_near_dock'] and not o[29]['loc_blind'] and o[30]['loc_blind']
          and 'dead_reckoning' in o[-1]['loc_blind_reason'] and o[-1]['loc_blind_count'] == 1)
    o = drive(nt, [(40, 'dock', ('grow', 50.0), ('grow', 50.0), {'in_dock': True})])
    check('standing in the dock: nothing, whatever the ages',
          not any(r['loc_blind'] or r['loc_blind_near_dock'] for r in o))
    o = drive(nt, [(40, 'tracker', ('grow', 50.0), ('grow', 50.0), {})])
    check('lidar tracker owns the pose (its own absolute pose): not blind',
          not any(r['loc_blind'] for r in o))
    o = drive(nt, [(5, 'gps', 0.2, 0.1, {}), (30, 'gps', 0.2, G, {'dock': None})])
    check('dock position unknown: only loc_blind_near_dock (may be at the dock), not counted',
          not any(r['loc_blind'] for r in o) and o[-1]['loc_blind_near_dock']
          and o[-1]['loc_blind_dock_m'] is None and o[-1]['loc_blind_count'] == 0)
    o = drive(nt, [(5, 'gps', 0.2, 0.1, {}), (30, 'gps', 0.2, G, {'direct': True})])
    check('anchor not advancing but the GPS position is written directly: not blind',
          not any(r['loc_blind'] or r['loc_blind_near_dock'] for r in o))
    o = drive(nt, [(5, 'gps', 0.2, 0.1, {}), (30, 'gps', 0.2, G, {'direct': True}),
                   (15, 'gps', 0.2, ('grow', 30.2), {})])
    check('... back on the anchor, still not advancing: position_anchor after 0.5 m',
          not o[35]['loc_blind'] and o[37]['loc_blind']
          and o[-1]['loc_blind_reason'] == 'position_anchor')
    o = drive(nt, [(40, 'gps', None, None, {})])
    first = [i for i, r in enumerate(o) if r['loc_blind']]
    check('no GNSS heading verified since the start, driving away from the dock: '
          'heading_unverified after 10 s', first and first[0] == 11
          and o[-1]['loc_blind_reason'] == 'heading_unverified' and o[-1]['loc_blind_s'] == 39.0,
          str(first[:1]))
    o = drive(nt, [(40, 'gps', None, None, {'dock': 3.0})])
    check('... the same within 6 m of the dock: near_dock only',
          not any(r['loc_blind'] for r in o) and o[-1]['loc_blind_near_dock'])
    o = drive(nt, [(5, 'gps', 0.2, 0.1, {}), (20, 'gps', G, 0.1, {}),
                   (10, 'gps', ('grow', 20.2), 0.1, {'v': 0.0}),
                   (3, 'gps', ('grow', 30.2), 0.1, {})])
    check('the robot stops: loc_blind goes off after 3 s standing, back with the first step',
          o[24]['loc_blind'] and o[26]['loc_blind'] and not o[27]['loc_blind']
          and not o[34]['loc_blind'] and o[35]['loc_blind']
          and o[34]['loc_blind_reason'] == 'heading_anchor' and o[-1]['loc_blind_count'] == 1)
    o = drive(nt, [(5, 'gps', 0.2, 0.1, {}), (30, 'gps', G, 0.1, {})],
              cfg={'after_s': 20.0, 'min_dist_m': 3.0})
    first = [i for i, r in enumerate(o) if r['loc_blind']][0] - 5
    check('cfg after_s 20, min_dist_m 3: blind after 20 s', first == 20, 'after %d s' % first)
    st = nt.anchor_freeze_new_state()
    nt.anchor_freeze_step(st, 10.0, (0.0, 0.0), 'gps', 0.2, 0.1, False, 20.0)
    for k in range(1, 30):
        r = nt.anchor_freeze_step(st, 10.0 + k, (50.0 * k, 0.0), 'gps', 0.2 + k, 0.1, False, 20.0)
    check('a pose step of 50 m per second (reset of the odometry) is not distance',
          not r['loc_blind'] and r['loc_blind_m'] == 0.0)

    print('node, publish_loc_status:')
    R.CLOCK[0] = 7000.0
    n_off, n_on = R.make_node(nt, None), R.make_node(nt, ON)
    for n in (n_off, n_on):
        n.publish_loc_status()
    j_off = json.loads(n_off.loc_status_pub.last.data)
    j_on = json.loads(n_on.loc_status_pub.last.data)
    check('parameter off (default): no loc_blind field',
          n_off._afr_enable is False and not [k for k in j_off if k.startswith('loc_blind')])
    check('on: the six fields, every other field equal to off',
          all(k in j_on for k in FIELDS) and {k: v for k, v in j_on.items() if k not in FIELDS} == j_off,
          str({k: j_on.get(k) for k in FIELDS}))
    # drive 30 s on GPS with the heading anchor applied at the start only
    n_on._ha_last_applied_t = 7000.0
    n_on._gps_stable = lambda: True
    n_on._anchor_last_advance_t = 7000.0
    n_on._dock_anchor_pose = (100.0, 0.0, 0.0)
    for k in range(1, 31):
        R.CLOCK[0] = 7000.0 + k
        n_on.ekf_wheel_odom_msg.pose.pose.position.x = 0.3 * k
        n_on.ekf_wheel_nav_odom_msg.pose.pose.position.x = 0.3 * k
        n_on._anchor_last_advance_t = 7000.0 + k - 0.1
        n_on.publish_loc_status()
    j = json.loads(n_on.loc_status_pub.last.data)
    check('heading anchor 30 s old while driving 9 m: loc_blind, reason heading_anchor',
          j['loc_blind'] is True and j['loc_blind_reason'] == 'heading_anchor'
          and j['loc_blind_s'] == 30.0 and abs(j['loc_blind_m'] - 8.7) < 0.31
          and j['loc_blind_count'] == 1 and j['loc_blind_dock_m'] == 91.0
          and j['heading_anchor_age_s'] == 30.0
          and j['heading_anchor_frozen'] is False,
          str({k: j.get(k) for k in FIELDS}))
    n_on._dock_anchor_pose = (10.0, 0.0, 0.0)          # robot at x = 9: 1 m from the dock
    R.CLOCK[0] = 7031.0
    n_on.publish_loc_status()
    j = json.loads(n_on.loc_status_pub.last.data)
    check('... 1 m from the DOCKED waypoint: loc_blind_near_dock instead',
          j['loc_blind'] is False and j['loc_blind_near_dock'] is True)
    # position anchor 20 s old with owner gps: emergency set_pose (nothing in the
    # anchor's buffer) is not blind, a frozen verified anchor is
    n_p = R.make_node(nt, ON)
    n_p._gps_stable = lambda: True
    n_p._dock_anchor_pose = (100.0, 0.0, 0.0)
    R.CLOCK[0] = 7100.0
    n_p._ha_last_applied_t = 7100.0
    n_p._anchor_last_advance_t = 7100.0
    seen = []
    for k in range(1, 41):
        R.CLOCK[0] = 7100.0 + k
        n_p._ha_last_applied_t = 7100.0 + k
        n_p.ekf_wheel_odom_msg.pose.pose.position.x = 0.3 * k
        if k == 21:                # the buffer fills: a lag-deep entry, a verified anchor
            entry = (7100.0 + k - 5.0, (0.0, 0.0), (0.0, 0.0), 0.0, 0.0)
            n_p._pose_buffer.append(entry)
            n_p._anchor_verified = entry
        n_p.publish_loc_status()
        seen.append(json.loads(n_p.loc_status_pub.last.data))
    check('node: anchor 20 s old, buffer empty (GPS written directly): not blind',
          seen[19]['anchor_age_s'] == 20.0 and seen[19]['anchor_frozen'] is True
          and not any(j['loc_blind'] for j in seen[:20]))
    check('node: ... with a verified anchor that does not advance: position_anchor',
          seen[-1]['loc_blind'] is True and seen[-1]['loc_blind_reason'] == 'position_anchor'
          and seen[-1]['loc_blind_s'] == 40.0, str({k: seen[-1].get(k) for k in FIELDS}))
    check('node: the state is guarded by a lock (loc_status goes out from two loops)',
          hasattr(n_p._afr_lock, 'acquire') and not n_p._afr_lock.locked())
    n_on._afr_st = None                                 # a fault in the report
    n_on.publish_loc_status()
    j = json.loads(n_on.loc_status_pub.last.data)
    check('a fault inside the report: loc_status still goes out, without the fields',
          'owner' in j and 'loc_blind' not in j)

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
