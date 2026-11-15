#!/usr/bin/env python3
"""T36 baseline: with every parameter of the T36 batch at its default (off) the
node must behave exactly as main before the batch (30eca80).

Four windows of the recorded drive magtest3 (2026-11-14) are replayed through
the node's own callbacks and its real loop30hz (see _t36_replay.py), once with
nodes/navi_transform of this checkout and once with `git show
30eca80:nodes/navi_transform`. Everything the node sends and keeps must be
EQUAL, value by value: the broadcast map->odom and odom->base_link, the pose
and twist of /odometry/odom, the yaw of /gnss_heading/nav_fused, diff_yaw,
map_odom_yaw and the times a heading sample was applied. No numbers are frozen
in this file - the reference is the old code itself.

Run:  python3 test_t36_baseline.py   (needs rospy - robot env)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

import numpy as np  # noqa: E402

# the parameters of the batch, at "off"; a parameter the checkout does not have
# (its change reverted / not there yet) is simply never read
OFF = {'odom_twist_from_wheels': False, 'heading_imu_interp': False,
       'handover_follow_ekf': False, 'dock_engage_slew_mps': 0.0}

# name, t0, t1, pin_until (recorded map->odom until then), dock anchor from t
WINDOWS = (('GNSS heading hand-over at UNDOCK', 85.0, 112.0, 94.5, None),
           ('driving on GPS, reversing', 315.0, 333.0, 1e9, None),
           ('driving on GPS, turning', 531.0, 559.0, 1e9, None),
           ('docking, first write of the dock anchor', 794.0, 822.0, 799.0, 821.0))


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-66s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    data = R.load_data()
    new = R.load('navi_transform_t36')
    ref = R.load_ref()
    if ref is None:
        print('git / commit %s not available - nothing to compare against' % R.MAIN_REF)
        return 1
    for name, t0, t1, pin, anchor_t in WINDOWS:
        print('%s (%.0f-%.0f s):' % (name, t0, t1))
        anchor = R.dock_anchor_from(data, anchor_t) if anchor_t else None
        kw = dict(pin_until=pin, dock_anchor=anchor)
        tr_ref, n_ref = R.replay(ref, data, t0, t1, **kw)
        for label, params in (('defaults', None), ('explicitly off', OFF)):
            tr, n = R.replay(new, data, t0, t1, params=params, **kw)
            diff = [k for k in sorted(tr_ref) if not np.array_equal(tr_ref[k], tr[k])]
            check('%s: all %d traces equal to %s (%d ticks)'
                  % (label, len(tr_ref), R.MAIN_REF, len(tr['t'])), not diff,
                  ('differs: %s' % ', '.join(diff)) if diff else '')
            mt_r, mt = n_ref.map_odom_transform.transform.translation, \
                n.map_odom_transform.transform.translation
            check('%s: final node state equal' % label,
                  (mt_r.x, mt_r.y, n_ref.map_odom_yaw, n_ref.diff_yaw, n_ref._dock_hold_active)
                  == (mt.x, mt.y, n.map_odom_yaw, n.diff_yaw, n._dock_hold_active))
        # the replay must really exercise the code (not compare two idle runs)
        if 'hand-over' in name:
            check('window exercises the hand-over (map_odom_yaw %.1f deg -> 0, %d samples applied)'
                  % (np.degrees(tr_ref['map_odom_yaw'][0]), len(tr_ref['hdg_t'])),
                  abs(tr_ref['map_odom_yaw'][0]) > R.D(10) and tr_ref['map_odom_yaw'][-1] == 0.0
                  and len(tr_ref['hdg_t']) > 20)
        elif 'docking' in name:
            x, y, yaw = R.map_base(tr_ref)
            step = np.hypot(np.diff(x), np.diff(y)).max()
            check('window exercises the dock hold (engaged, first write moves %.1f cm)'
                  % (100 * step), n_ref._dock_hold_active and step > 0.05)
        else:
            check('window exercises driving (max |v| %.2f m/s, %d heading samples applied)'
                  % (np.abs(tr_ref['od_v']).max(), len(tr_ref['hdg_t'])),
                  np.abs(tr_ref['od_v']).max() > 0.2 and len(tr_ref['hdg_t']) > 5)
    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
