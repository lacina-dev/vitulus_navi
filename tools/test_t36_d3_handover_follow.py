#!/usr/bin/env python3
"""T36 D3 (~handover_follow_ekf): during the GNSS heading hand-over the
broadcast map->odom is turned about the robot by the part of diff_yaw the nav
EKF has not taken yet, so the heading of map->base does not sawtooth by the
0.5 deg step of every heading sample.

Replays the hand-over of the recorded drive magtest3 (2026-11-14, robot
standing after UNDOCK) through the node (see _t36_replay.py). The EKF yaw is
modelled as the fused heading delayed by 70 / 300 / 700 ms (the range seen in
the recording), because the replayed samples do not fall on the recorded times:
  - off: the heading of map->base steps 0.5-1.0 deg per 30 Hz tick,
  - on: at most 0.1 deg per tick, and the swing is the real change of the
    heading sum only,
  - on: diff_yaw, map_odom_yaw, map_odom_transform, odom->base, /odometry/odom
    are EQUAL to off tick by tick, and so is the robot's position in the map,
  - after the hand-over the broadcast is exactly the node's own map->odom again.
Then built cases: samples stop mid-way, dock hold, EKF not following, turning
robot, no IMU, an exception inside. Only this parameter is switched on.

Run:  python3 test_t36_d3_handover_follow.py   (needs rospy - robot env)
"""
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

import numpy as np  # noqa: E402

ON = {'handover_follow_ekf': True}
D = math.radians
BROADCAST = ('mo_x', 'mo_y', 'mo_yaw')
OB = (2.1, -4.24)


def steps(tr, a, b):
    _x, _y, yaw = R.map_base(tr)
    m = R.sel(tr, a, b)
    return np.abs(np.degrees(np.diff(yaw[m]))), float(np.degrees(np.ptp(yaw[m])))


def sim(nt, params, seconds=12.0, delay=0.3, w=0.0, samples_until=1e9, noise=D(0.2),
        mo0=D(-10.0), setup=None, imu_until=1e9, ekf_follows=True, off_target=0.0):
    """Small closed loop: IMU 100 Hz (turning at w), EKF 30 Hz with yaw = imu +
    diff_yaw as it was `delay` ago, heading samples 4.4 Hz from 1 s (straight to
    _apply_heading_target, target = the consistent heading sum + noise +
    off_target), loop30hz after every EKF message. Returns rows (t, broadcast
    map->odom x, y, yaw, node map->odom x, y, yaw, ekf yaw, diff_yaw) and the node."""
    t0 = 9000.0
    R.CLOCK[0] = t0
    n = R.make_node(nt, params)
    n.init = False
    n.diff_yaw = D(40.0)
    n.map_odom_yaw = mo0
    n._map_odom_placed = True
    n.map_odom_transform.transform.translation.x = 3.0
    n.map_odom_transform.transform.translation.y = -4.0
    n.map_odom_transform.transform.translation.z = 0.25
    if setup is not None:
        setup(n)
    target = n.diff_yaw + mo0 + off_target
    hist = [(0.0, n.diff_yaw)]
    rows = []
    next_s = 1.0
    ekf_yaw = 0.3 + n.diff_yaw
    for k in range(int(seconds * 300)):
        t = k / 300.0
        R.CLOCK[0] = t0 + t
        imu_yaw = 0.3 + w * t
        if k % 3 == 0 and t < imu_until:
            n.callback_imu(R.imu_msg(t0 + t, R.wrap(imu_yaw), w))
            hist.append((t, n.diff_yaw))
        if t >= next_s and t < samples_until:
            n._apply_heading_target(target + noise * math.sin(7.0 * t))
            next_s += 1.0 / 4.4
        if k % 10 == 0:
            j = len(hist) - 1
            while j > 0 and hist[j][0] > t - delay:
                j -= 1
            if ekf_follows:
                ekf_yaw = R.wrap(0.3 + w * (t - 0.005) + hist[j][1])
            n.callback_ekf_wheel_nav_odom(R.odom_msg(t0 + t, OB[0], OB[1], -0.25, ekf_yaw, 0.0, w))
            n.loop30hz()
            sent = n.tf2_TransformBroadcaster.last
            tm = [x for x in sent if x.child_frame_id == 'odom'][0].transform
            it = n.map_odom_transform.transform.translation
            rows.append((t, tm.translation.x, tm.translation.y, R.quat_yaw(tm.rotation),
                         it.x, it.y, n.map_odom_yaw, ekf_yaw, n.diff_yaw, tm.translation.z))
    return np.array(rows), n


def base_xy(x, y, yaw):
    return (x + np.cos(yaw) * OB[0] - np.sin(yaw) * OB[1],
            y + np.sin(yaw) * OB[0] + np.cos(yaw) * OB[1])


def heading_steps(rows, w=0.0, whole=False):
    """Per-tick change of the map->base heading (the robot's own turning
    removed) until 0.1 s after map_odom_yaw reached 0 - after that the GNSS
    corrections go straight to diff_yaw as before, hand-over or not."""
    yaw = np.unwrap(rows[:, 3] + rows[:, 7])
    st = np.abs(np.degrees(np.diff(yaw) - w * np.diff(rows[:, 0])))
    zero = np.flatnonzero(rows[:, 6] == 0.0)
    if whole or len(zero) == 0:
        return st
    return st[rows[1:, 0] <= rows[zero[0], 0] + 0.1]


def same(a, b):
    return a.shape == b.shape and np.abs(a - b).max() < 1e-12


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-72s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    data = R.load_data()
    nt = R.load('navi_transform_t36_d3')

    for delay in (0.07, 0.3, 0.7):
        print('hand-over at UNDOCK replayed, EKF yaw %.0f ms behind the fused heading:' % (1000 * delay))
        off, n_off = R.replay(nt, data, 85.0, 112.0, pin_until=94.5, ekf_delay=delay)
        on, n_on = R.replay(nt, data, 85.0, 112.0, params=ON, pin_until=94.5, ekf_delay=delay)
        first = off['hdg_t'][off['hdg_t'] >= 94.5][0]
        done = off['t'][np.flatnonzero(off['map_odom_yaw'] == 0.0)[0]]
        s_off, p_off = steps(off, first - 0.5, done)
        s_on, p_on = steps(on, first - 0.5, done)
        check('heading of map->base, largest step per tick: %.2f -> %.3f deg (expected <= 0.1)'
              % (s_off.max(), s_on.max()), s_off.max() >= 0.45 and s_on.max() <= 0.1)
        check('... ticks stepping over 0.1 deg: %d -> %d' % ((s_off > 0.1).sum(), (s_on > 0.1).sum()),
              (s_off > 0.1).sum() > 30 and (s_on > 0.1).sum() == 0)
        check('... swing over the hand-over: %.2f -> %.2f deg' % (p_off, p_on),
              p_on < 0.5 and p_on < p_off)
        diff = [k for k in sorted(off) if k not in BROADCAST and not np.array_equal(off[k], on[k])]
        check('node state and everything else it sends equal to off', not diff,
              ('differs: %s' % ', '.join(diff)) if diff else '')
        a, b = n_off.map_odom_transform.transform.translation, n_on.map_odom_transform.transform.translation
        check('final map_odom_transform / map_odom_yaw / diff_yaw equal to off',
              (a.x, a.y, n_off.map_odom_yaw, n_off.diff_yaw) == (b.x, b.y, n_on.map_odom_yaw, n_on.diff_yaw))
        x0, y0, _ = R.map_base(off)
        x1, y1, _ = R.map_base(on)
        dev = np.abs(np.degrees(np.arctan2(np.sin(on['mo_yaw'] - off['mo_yaw']),
                                           np.cos(on['mo_yaw'] - off['mo_yaw']))))
        check('robot position in the map equal to off (max %.1e m), broadcast turned <= %.2f deg'
              % (np.hypot(x1 - x0, y1 - y0).max(), dev.max()),
              np.hypot(x1 - x0, y1 - y0).max() < 1e-9 and 0.3 < dev.max() <= 5.0)
        late = on['t'] > done + 1.5
        check('1.5 s after the hand-over the broadcast is the node\'s map->odom again',
              late.sum() > 10 and all(np.array_equal(off[k][late], on[k][late]) for k in BROADCAST)
              and n_on._hf_c == 0.0 and n_on._hf_s is None)

    print('recorded EKF yaw (not synchronous with the replayed samples): still only the broadcast')
    off, _ = R.replay(nt, data, 85.0, 112.0, pin_until=94.5)
    on, _ = R.replay(nt, data, 85.0, 112.0, params=ON, pin_until=94.5)
    diff = [k for k in sorted(off) if k not in BROADCAST and not np.array_equal(off[k], on[k])]
    x0, y0, _ = R.map_base(off)
    x1, y1, _ = R.map_base(on)
    check('node state equal to off, robot position equal', not diff
          and np.hypot(x1 - x0, y1 - y0).max() < 1e-9)

    print('built hand-over of 10 deg (EKF 300 ms behind):')
    r_off, _ = sim(nt, None)
    r_on, n = sim(nt, ON)
    check('off steps %.2f deg per tick, on %.3f deg' % (heading_steps(r_off).max(), heading_steps(r_on).max()),
          heading_steps(r_off).max() > 0.45 and heading_steps(r_on).max() <= 0.1)
    check('node map->odom / diff_yaw equal to off', np.array_equal(r_off[:, 4:7], r_on[:, 4:7])
          and np.array_equal(r_off[:, 8], r_on[:, 8]))
    bx, by = base_xy(r_on[:, 1], r_on[:, 2], r_on[:, 3])
    ix, iy = base_xy(r_on[:, 4], r_on[:, 5], r_on[:, 6])
    check('robot position from the broadcast = from the node state (max %.1e m)'
          % np.hypot(bx - ix, by - iy).max(), np.hypot(bx - ix, by - iy).max() < 1e-9)
    check('map->odom z untouched', np.array_equal(r_on[:, 9], r_off[:, 9]))
    check('hand-over finished: map_odom_yaw 0 and broadcast = node state',
          n.map_odom_yaw == 0.0 and same(r_on[-30:, 1:4], r_on[-30:, 4:7]) and n._hf_c == 0.0)

    print('dock heading 6 deg off the GNSS one (the heading sum really moves):')
    r_off, _ = sim(nt, None, seconds=16.0, off_target=D(6.0))
    r_on, n = sim(nt, ON, seconds=16.0, off_target=D(6.0))
    check('off steps %.2f deg per tick, on %.3f deg' % (heading_steps(r_off).max(), heading_steps(r_on).max()),
          heading_steps(r_off).max() > 0.45 and heading_steps(r_on).max() <= 0.1 + 1e-6)
    end_off = R.wrap(r_off[-1, 3] + r_off[-1, 7])
    end_on = R.wrap(r_on[-1, 3] + r_on[-1, 7])
    check('ends on the same map heading as off', abs(R.wrap(end_on - end_off)) < 1e-9
          and n.map_odom_yaw == 0.0)

    print('robot turning 0.5 rad/s during the hand-over:')
    r_off, _ = sim(nt, None, w=0.5)
    r_on, n = sim(nt, ON, w=0.5)
    check('heading steps beyond the turning itself: off %.2f, on %.3f deg per tick'
          % (heading_steps(r_off, 0.5).max(), heading_steps(r_on, 0.5).max()),
          heading_steps(r_on, 0.5).max() <= 0.1)
    bx, by = base_xy(r_on[:, 1], r_on[:, 2], r_on[:, 3])
    ix, iy = base_xy(r_on[:, 4], r_on[:, 5], r_on[:, 6])
    check('robot position kept', np.hypot(bx - ix, by - iy).max() < 1e-9)

    print('heading samples stop mid-way (stalled hand-over):')
    r_on, n = sim(nt, ON, samples_until=3.0, seconds=8.0)
    dev = np.abs(np.degrees(R.wrap(0) + np.arctan2(np.sin(r_on[:, 3] - r_on[:, 6]),
                                                   np.cos(r_on[:, 3] - r_on[:, 6]))))
    t = r_on[:, 0]
    check('map_odom_yaw left at %.1f deg; broadcast = node state from %.1f s on'
          % (math.degrees(n.map_odom_yaw), t[np.flatnonzero(dev > 0)[-1]] + 1 / 30.0),
          n.map_odom_yaw != 0.0 and (dev[t > 5.5] == 0.0).all() and n._hf_c == 0.0)
    check('... no step on the way back (max %.3f deg per tick)' % heading_steps(r_on, whole=True).max(),
          heading_steps(r_on, whole=True).max() <= 0.1)

    print('dock holds the pose:')

    def docked(n):
        n._dock_status_val = 0
        n._docked = True
        n._docked_since = R.CLOCK[0] - 10.0
        n._dock_status_rx_t = R.CLOCK[0] + 2.0
        n._dock_anchor_pose = (5.0, -8.0, 0.4)
        n._dock_anchor_fetch_t = R.CLOCK[0]
        sent = [None]
        real = n.transform_odom_to_base_link

        def odom_base(stamp=None):
            sent[0] = real(stamp)
            return sent[0]
        n.transform_odom_to_base_link = odom_base
        n.get_base_link_transform = lambda stamp=None: sent[0]
    r_on, n = sim(nt, ON, setup=docked, seconds=4.0)
    check('dock hold engaged, broadcast = node state all the time',
          n._dock_hold_active and same(r_on[:, 1:4], r_on[:, 4:7]) and n._hf_c == 0.0)

    print('EKF does not follow the fused heading at all:')
    r_on, n = sim(nt, ON, ekf_follows=False)
    dev = np.abs(np.degrees(np.arctan2(np.sin(r_on[:, 3] - r_on[:, 6]), np.cos(r_on[:, 3] - r_on[:, 6]))))
    check('broadcast turned at most handover_follow_max_deg (%.2f deg), steps as off at most (%.2f deg)'
          % (dev.max(), heading_steps(r_on, whole=True).max()),
          4.9 < dev.max() <= 5.0 + 1e-9 and heading_steps(r_on, whole=True).max() <= 0.5 + 1e-6)
    check('... and back on the node state after the hand-over', dev[-1] == 0.0 and n.map_odom_yaw == 0.0)

    print('IMU silent from 3 s:')
    r_on, n = sim(nt, ON, imu_until=3.0, seconds=8.0)
    dev = np.abs(np.degrees(np.arctan2(np.sin(r_on[:, 3] - r_on[:, 6]), np.cos(r_on[:, 3] - r_on[:, 6]))))
    r_off, _ = sim(nt, None, imu_until=3.0, seconds=8.0)
    check('back on the node state within 2 s, steps %.2f deg per tick (off: %.2f)'
          % (heading_steps(r_on, whole=True).max(), heading_steps(r_off, whole=True).max()),
          (dev[r_on[:, 0] > 5.0] == 0.0).all()
          and heading_steps(r_on, whole=True).max() <= heading_steps(r_off, whole=True).max() + 0.05)

    print('switch and failure:')

    def must_not_run(n):
        def boom(*a, **k):
            raise AssertionError('handover_follow called with the parameter off')
        n.handover_follow = boom
    r, n = sim(nt, None, setup=must_not_run, seconds=3.0)
    check('off (default): handover_follow never runs, no hand-over time kept',
          n._hf_enable is False and n._hf_t_sample is None and len(r) > 80)

    def broken(n):
        def boom(*a, **k):
            raise RuntimeError('boom')
        n.handover_follow = boom
    r, n = sim(nt, ON, setup=broken, seconds=3.0)
    check('an exception inside: both transforms still sent, map->odom = node state',
          len(r) > 80 and same(r[:, 1:4], r[:, 4:7]))

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
