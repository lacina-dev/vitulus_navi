#!/usr/bin/env python3
"""T36 D4 (~dock_engage_slew_mps): the first write of the dock anchor after
docking is walked at a bounded speed instead of moving the robot's map pose by
the whole collected error in one 30 Hz tick.

On the robot that first write is the pulse of dock_localization_seed
(/nav_tf/set_map_pose_dock -> callback_set_map_pose(source='dock'), 0.07-1.26 s
after the docked edge); the dock hold engages 2 s after the edge. The replay
(_t36_replay.py) has no seed node, so SeedSim below sends its pulse in the
recorded order: at the moment the recorded map pose jumped.

Replays the docking of the recorded drive magtest3 (2026-11-14) through the
node (the DOCKED waypoint is the pose the recorded hold kept) with the
parameter 0 and 0.10 m/s:
  - real order, seed pulse 1.26 s after the edge, then the hold:
    0: the pulse moves the robot 23.8 cm and 2.0 deg in one tick,
    0.10: at most 0.4 cm per tick and 0.17 deg on top of the EKF's own yaw
    noise (0.3 deg in all), the hold takes the walk over
    without a step, repeated pulses change nothing, and after the walk every
    value equals the run with 0; with 0 every value equals main before T36,
  - without the seed (the hold alone writes first): the same,
then built cases: fresh start (map->odom not placed), error over 1 m / 15 deg
(all immediate, as before), the seed's pulse before / during / after the walk,
undock during the walk and a dock pulse after it, another writer, stale dock
feed, map change, the creep sanity bound.
Only this parameter is switched on.

Run:  python3 test_t36_d4_dock_engage.py   (needs rospy - robot env)
"""
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

import numpy as np  # noqa: E402
from geometry_msgs.msg import PoseStamped, TransformStamped  # noqa: E402

ON = {'dock_engage_slew_mps': 0.10}
D = math.radians
ANCHOR = (5.0, -8.0, 0.4)
OB = (2.1, -4.24, -2.0)


def pose_steps(tr):
    x, y, yaw = R.map_base(tr)
    return np.hypot(np.diff(x), np.diff(y)), np.abs(np.degrees(np.diff(yaw))), x, y, yaw


def pose_msg(a):
    p = PoseStamped()
    p.pose.position.x, p.pose.position.y = a[0], a[1]
    R._yaw_quat(p.pose.orientation, a[2])
    return p


class SeedSim(object):
    """dock_localization_seed for the replay (hook): its one pulse per docked
    episode - the DOCKED waypoint on the dock channel - `delay` s after the
    docked edge, and further pulses `more` s after the first one (the node's
    re-anchor / retry pulses)."""
    def __init__(self, anchor, delay, more=()):
        self.msg = pose_msg(anchor)
        self.delay, self.more = delay, tuple(more)
        self.prev = None
        self.due = []
        self.edge = None
        self.sent = []

    def __call__(self, n, t, kind):
        if kind == 'dock':
            if n._dock_status_val == 0 and self.prev not in (None, 0):
                self.edge = t
                self.due = [t + self.delay] + [t + self.delay + m for m in self.more]
            elif n._dock_status_val != 0:
                self.due = []
            self.prev = n._dock_status_val
        while self.due and t >= self.due[0]:
            self.due.pop(0)
            self.sent.append(t)
            n.callback_set_map_pose(self.msg, source='dock')


def recorded_jump(data, a, b):
    """(time, metres) of the largest one-tick step of the recorded map pose in a..b."""
    mo, ob = data['mo'], data['ob']
    t = ob[:, 0] - data['T0']
    m = np.flatnonzero((t >= a) & (t <= b))
    c, s = np.cos(mo[m, 5]), np.sin(mo[m, 5])
    x = mo[m, 2] + c * ob[m, 2] - s * ob[m, 3]
    y = mo[m, 3] + s * ob[m, 2] + c * ob[m, 3]
    st = np.hypot(np.diff(x), np.diff(y))
    i = int(np.argmax(st))
    return t[m][i + 1], st[i]


def docked_edge(data, a, b):
    dk = data['dock']
    t = dk[:, 0] - data['T0']
    i = np.flatnonzero((dk[1:, 1] == 0) & (dk[:-1, 1] != 0) & (t[1:] >= a) & (t[1:] <= b))
    return t[i[0] + 1]


def real_order(nt, data, a, b, check, more=(0.3, 0.8, 1.3, 1.9, 2.4, 4.0, 6.0), ref=None):
    """Docking in a..b replayed with the seed's pulse at the recorded moment.
    Returns the numbers for the report."""
    edge = docked_edge(data, a, b)
    t_jump, jump = recorded_jump(data, edge - 0.5, edge + 4.0)
    delay = t_jump - edge - 0.02          # sent just before the tick that showed it
    anchor = R.dock_anchor_from(data, edge + 8.0)
    kw = dict(pin_until=edge - 0.1, dock_anchor=anchor)
    s_off, s_on, s_rep = SeedSim(anchor, delay), SeedSim(anchor, delay), SeedSim(anchor, delay, more)
    off, n_off = R.replay(nt, data, a, b, hook=s_off, **kw)
    on, n_on = R.replay(nt, data, a, b, params=ON, hook=s_on, **kw)
    rep, n_rep = R.replay(nt, data, a, b, params=ON, hook=s_rep, **kw)
    d_off, a_off, _, _, _ = pose_steps(off)
    d_on, a_on, x, y, yaw = pose_steps(on)
    i0 = int(np.argmax(d_off)) + 1
    check('recorded: pulse %.2f s after the docked edge moved the pose %.1f cm; replayed with 0: '
          '%.1f cm / %.2f deg at +%.2f s' % (t_jump - edge, 100 * jump, 100 * d_off.max(),
                                              a_off.max(), off['t'][i0] - edge),
          abs(d_off.max() - jump) < 0.02 and abs(off['t'][i0] - t_jump) < 0.05
          and off['t'][i0] - edge < 1.9 and len(s_off.sent) == 1)
    # heading: the hold (and the walk) place the robot from odom->base of the
    # previous tick, so the EKF's tick-to-tick yaw noise shows in the map
    # heading with 0 as well (hold_a); the walk adds its own <= 0.17 deg
    hold_a = a_off[i0 + 1:].max()
    ob_a = np.abs(np.degrees(np.diff(np.unwrap(on['ob_yaw']))))
    own = a_on[1:] - ob_a[1:] - ob_a[:-1]
    wk = slice(i0, i0 + int(30 * (10.0 * jump + 1.0)))
    check('0.10: largest step per tick %.2f cm (<= 0.4); heading %.2f deg, during the walk %.2f deg, '
          'of that the walk itself <= %.2f deg (<= 0.17)'
          % (100 * d_on.max(), a_on.max(), a_on[wk].max(), max(own.max(), 0.0)),
          d_on.max() <= 0.0041 and own.max() <= 0.17 + 1e-6
          and a_on.max() <= max(hold_a, a_on[wk].max()) + 1e-9 and a_on[wk].max() <= 0.45,
          'engaged hold alone with 0: %.2f deg' % hold_a)
    differs = np.flatnonzero((on['mo_x'] != off['mo_x']) | (on['mo_y'] != off['mo_y'])
                             | (on['mo_yaw'] != off['mo_yaw']))
    t_hold = edge + n_on._dock_anchor_debounce_s
    check('differs from 0 only from the pulse (+%.2f s) to the end of the walk (+%.2f s); '
          'hold engaged at +%.2f s' % (on['t'][differs[0]] - edge, on['t'][differs[-1]] - edge,
                                         t_hold - edge),
          differs[0] == i0 and on['t'][differs[-1]] - on['t'][i0] <= 10.0 * jump + 1.0)
    w = (on['t'] >= t_hold - 0.2) & (on['t'] <= t_hold + 0.2)
    check('no step where the hold takes over (max %.2f cm in +-0.2 s)'
          % (100 * d_on[w[1:]].max()), d_on[w[1:]].max() <= 0.0041)
    after = np.arange(len(on['t'])) > differs[-1]
    ticks = [k for k in sorted(off) if np.shape(off[k]) == np.shape(off['t'])]
    same = [k for k in ticks if not np.array_equal(off[k][after], on[k][after])]
    check('after the walk every value equal to 0 again (%d ticks)' % after.sum(),
          after.sum() > 100 and not same, ', '.join(same))
    other = [k for k in sorted(off) if k not in ('mo_x', 'mo_y', 'mo_yaw', 'map_odom_yaw')
             and not np.array_equal(off[k], on[k])]
    check('odom->base, /odometry/odom, diff_yaw untouched all the time', not other, ', '.join(other))
    ws = slice(i0 - 1, differs[-1] + 2)
    to_anchor = np.hypot(x[ws] - anchor[0], y[ws] - anchor[1])
    check('distance to the DOCKED waypoint shrinks all the way (%.1f cm -> %.2f cm)'
          % (100 * to_anchor[0], 100 * to_anchor[-1]),
          (to_anchor[10:] < to_anchor[:-10]).all() and to_anchor[-1] < 0.005)
    check('hold engaged, walk over, no refusal',
          n_on._dock_hold_active and n_on._dock_engage_off is None
          and n_on._dock_hold_refusals == 0 and n_off._dock_hold_refusals == 0)
    diff = [k for k in sorted(on) if not np.array_equal(on[k], rep[k])]
    check('%d more pulses during and after the walk: not one value changes' % len(more),
          len(s_rep.sent) == 1 + len(more) and not diff and n_rep._dock_engage_off is None,
          ', '.join(diff))
    if ref is not None:
        s_ref = SeedSim(anchor, delay, more)
        s_off2 = SeedSim(anchor, delay, more)
        r, _ = R.replay(ref, data, a, b, hook=s_ref, **kw)
        o, _ = R.replay(nt, data, a, b, hook=s_off2, **kw)
        diff = [k for k in sorted(r) if not np.array_equal(r[k], o[k])]
        check('parameter 0 with the same pulses: every value equal to main %s' % R.MAIN_REF,
              not diff and len(s_ref.sent) == len(s_off2.sent) == 1 + len(more), ', '.join(diff))
    return dict(edge=edge, pulse=t_jump - edge, rec_cm=100 * jump, off_cm=100 * d_off.max(),
                off_deg=a_off.max(), on_cm=100 * d_on.max(), on_deg=a_on[wk].max(),
                walk_s=on['t'][differs[-1]] - on['t'][i0])


def docked_node(nt, params, err=(0.2, 0.1, D(2.0)), placed=True, t0=8000.0, docked_for=10.0):
    """A node whose map pose is `err` (x, y, yaw) off the DOCKED waypoint, dock
    status 0 for `docked_for` s (the hold engages after 2 s). Returns (node,
    tick, pose): tick() runs dock_anchor_hold after 1/30 s and returns the
    robot's map pose."""
    R.CLOCK[0] = t0
    n = R.make_node(nt, params)
    ob = TransformStamped()
    ob.transform.translation.x, ob.transform.translation.y = OB[0], OB[1]
    R._yaw_quat(ob.transform.rotation, OB[2])
    n.get_base_link_transform = lambda stamp=None: ob
    tx, ty, m = nt.dock_hold_map_odom(
        (ANCHOR[0] + err[0], ANCHOR[1] + err[1], ANCHOR[2] + err[2]), OB)
    n.map_odom_transform.transform.translation.x = tx
    n.map_odom_transform.transform.translation.y = ty
    n.map_odom_yaw = m
    n._map_odom_placed = placed
    n._dock_anchor_pose = ANCHOR
    n.callback_dock_status(types.SimpleNamespace(data=3))
    n.callback_dock_status(types.SimpleNamespace(data=0))
    n._docked_since = t0 - docked_for

    def pose():
        t = n.map_odom_transform.transform.translation
        c, s = math.cos(n.map_odom_yaw), math.sin(n.map_odom_yaw)
        return (t.x + c * OB[0] - s * OB[1], t.y + s * OB[0] + c * OB[1],
                R.wrap(n.map_odom_yaw + OB[2]))

    def tick():
        R.CLOCK[0] += 1.0 / 30.0
        n._dock_status_rx_t = R.CLOCK[0]
        n.dock_anchor_hold()
        return pose()
    return n, tick, pose


def off_anchor(p):
    return math.hypot(p[0] - ANCHOR[0], p[1] - ANCHOR[1]), abs(R.wrap(p[2] - ANCHOR[2]))


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-72s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    data = R.load_data()
    nt = R.load('navi_transform_t36_d4')

    ref = R.load_ref('navi_transform_ref_d4')
    if ref is None:
        print('  (main %s not available from git: comparison with it skipped)' % R.MAIN_REF)
    print('docking replayed in the real order, seed pulse then hold (794-822 s):')
    real_order(nt, data, 794.0, 822.0, check, ref=ref)

    print('docking replayed without the seed, the hold writes first (794-822 s):')
    anchor = R.dock_anchor_from(data, 821.0)
    kw = dict(pin_until=799.0, dock_anchor=anchor)
    off, n_off = R.replay(nt, data, 794.0, 822.0, **kw)
    on, n_on = R.replay(nt, data, 794.0, 822.0, params=ON, **kw)
    d_off, a_off, _, _, _ = pose_steps(off)
    d_on, a_on, x, y, yaw = pose_steps(on)
    i0 = int(np.argmax(d_off)) + 1                    # the tick of the first write
    # the hold itself lets the EKF's tick-to-tick yaw noise through (it works
    # with odom->base of the previous tick): that is there with 0 as well
    hold_a = a_off[i0 + 1:].max()
    check('robot map pose, largest step per tick: %.1f cm -> %.2f cm (expected <= 0.4)'
          % (100 * d_off.max(), 100 * d_on.max()), d_off.max() > 0.2 and d_on.max() <= 0.0041)
    check('... heading: %.2f deg -> %.2f deg (the engaged hold alone: %.2f deg, p95 %.2f)'
          % (a_off.max(), a_on.max(), hold_a, np.percentile(a_off[i0 + 120:], 95)),
          a_off.max() > 1.5 and a_on.max() <= max(hold_a, 0.2) + 1e-9)
    check('... during the walk p95 %.2f cm / %.2f deg per tick'
          % (100 * np.percentile(d_on[i0:i0 + 70], 95), np.percentile(a_on[i0:i0 + 70], 95)),
          np.percentile(d_on[i0:i0 + 70], 95) <= 0.004 and np.percentile(a_on[i0:i0 + 70], 95) <= 0.2)
    differs = np.flatnonzero((on['mo_x'] != off['mo_x']) | (on['mo_y'] != off['mo_y'])
                             | (on['mo_yaw'] != off['mo_yaw']))
    walk_s = on['t'][differs[-1]] - on['t'][i0] if len(differs) else 0.0
    check('before the first write everything equal to 0', len(differs) > 0 and differs[0] == i0)
    check('walk of %.1f cm takes %.1f s (0.10 m/s; a late tick gets no bigger step)'
          % (100 * d_off.max(), walk_s), 2.2 <= walk_s <= 3.5)
    after = np.arange(len(on['t'])) > differs[-1]
    ticks = [k for k in sorted(off) if np.shape(off[k]) == np.shape(off['t'])]
    same = [k for k in ticks if not np.array_equal(off[k][after], on[k][after])]
    check('after the walk every value equal to 0 again (%d ticks)' % after.sum(),
          after.sum() > 100 and not same, ('differs: %s' % ', '.join(same)) if same else '')
    other = [k for k in sorted(off) if k not in ('mo_x', 'mo_y', 'mo_yaw', 'map_odom_yaw')
             and not np.array_equal(off[k], on[k])]
    check('odom->base, /odometry/odom, diff_yaw untouched all the time', not other, ', '.join(other))
    w = slice(i0 - 1, differs[-1] + 2)
    to_anchor = np.hypot(x[w] - anchor[0], y[w] - anchor[1])
    check('distance to the DOCKED waypoint shrinks all the way (%.1f cm -> %.2f cm)'
          % (100 * to_anchor[0], 100 * to_anchor[-1]),
          (to_anchor[10:] < to_anchor[:-10]).all() and to_anchor[-1] < 0.005)
    check('dock hold engaged and owning the pose at the end',
          n_on._dock_hold_active and n_on._dock_engage_off is None
          and n_on._dock_hold_refusals == 0 and n_off._dock_hold_refusals == 0)

    print('built: pose 22 cm / 2 deg off the DOCKED waypoint, map->odom placed:')
    n, tick, pose = docked_node(nt, ON)
    start = pose()
    path = [start] + [tick() for _ in range(120)]
    dd = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])]
    da = [abs(R.wrap(b[2] - a[2])) for a, b in zip(path, path[1:])]
    check('first tick moves %.2f cm / %.2f deg (not the whole error)'
          % (100 * dd[0], math.degrees(da[0])), dd[0] <= 0.0034 and da[0] <= D(0.17))
    check('every tick <= 0.34 cm and <= 0.17 deg', max(dd) <= 0.0034 and max(da) <= D(0.17))
    done = next(i for i, p in enumerate(path) if off_anchor(p)[0] < 1e-9 and off_anchor(p)[1] < 1e-9)
    check('on the waypoint exactly after %.1f s' % (done / 30.0), 60 <= done <= 72)
    check('hold active from the first tick, flags as with the immediate write',
          n._dock_hold_active and n.init is False and n._map_pose_target is None
          and n._georef_target is None and n._yaw_level is False and n._map_odom_placed)
    cross = [abs((p[0] - start[0]) * (ANCHOR[1] - start[1]) - (p[1] - start[1]) * (ANCHOR[0] - start[0]))
             / math.hypot(ANCHOR[0] - start[0], ANCHOR[1] - start[1]) for p in path]
    check('straight line to the waypoint (max %.1e m aside)' % max(cross), max(cross) < 1e-9)

    print('immediate as before:')
    n, tick, pose = docked_node(nt, None)
    p = tick()
    check('parameter 0 (default): whole error in the first tick', off_anchor(p)[0] < 1e-9
          and n._dock_engage_slew_mps == 0.0)
    n, tick, pose = docked_node(nt, ON, placed=False)
    p = tick()
    check('fresh start (map->odom not placed yet): immediate', off_anchor(p)[0] < 1e-9
          and off_anchor(p)[1] < 1e-9 and n._dock_engage_off is None)
    n, tick, pose = docked_node(nt, ON, err=(1.2, 0.0, 0.0))
    p = tick()
    check('1.2 m off (> dock_engage_slew_max_m): immediate', off_anchor(p)[0] < 1e-9)
    n, tick, pose = docked_node(nt, ON, err=(0.1, 0.0, D(20.0)))
    p = tick()
    check('20 deg off (> dock_engage_slew_max_deg): immediate', off_anchor(p)[0] < 1e-9
          and off_anchor(p)[1] < 1e-9)
    n, tick, pose = docked_node(nt, ON, err=(0.0, 0.0, D(10.0)))
    path = [tick() for _ in range(90)]
    da = [abs(R.wrap(b[2] - a[2])) for a, b in zip(path, path[1:])]
    check('10 deg off, position right: heading walked at 5 deg/s, done in 2 s',
          max(da) <= D(0.17) and off_anchor(path[62])[1] < 1e-9 and off_anchor(path[30])[1] > D(4.0))

    print('dock seed pulse (/nav_tf/set_map_pose_dock, instant write of the same waypoint):')
    seed = PoseStamped()
    seed.pose.position.x, seed.pose.position.y = ANCHOR[0], ANCHOR[1]
    R._yaw_quat(seed.pose.orientation, ANCHOR[2])
    n, tick, pose = docked_node(nt, ON)
    for _ in range(20):
        p = tick()
    n.callback_set_map_pose(seed, source='dock')
    check('during the walk: ignored, the pose does not jump', pose() == p)
    p2 = tick()
    check('... and the walk goes on by one step', 0.0 < math.hypot(p2[0] - p[0], p2[1] - p[1]) <= 0.0034)
    for _ in range(80):
        tick()
    tr = n.map_odom_transform.transform.translation
    tr.x += 0.03
    n.callback_set_map_pose(seed, source='dock')
    check('after the walk: applied at once, as before', n._dock_engage_off is None
          and off_anchor(pose())[0] < 1e-9)
    n, tick, pose = docked_node(nt, None)
    n.callback_set_map_pose(seed, source='dock')
    check('parameter 0: applied at once, as before', off_anchor(pose())[0] < 1e-9
          and off_anchor(pose())[1] < 1e-9)

    print('undock during the walk, creep bound:')
    n, tick, pose = docked_node(nt, ON)
    for _ in range(20):
        p = tick()
    left = off_anchor(p)[0]
    n.callback_dock_status(types.SimpleNamespace(data=1))
    p2 = tick()
    check('undock after 0.67 s: hold released, pose stays where the walk got (%.1f cm short)'
          % (100 * left), not n._dock_hold_active and p2 == p and 0.1 < left < 0.2)
    n.callback_dock_status(types.SimpleNamespace(data=0))
    n._docked_since = R.CLOCK[0] - 10.0
    path = [tick() for _ in range(60)]
    check('docked again: a new walk from there, on the waypoint after %.1f cm'
          % (100 * left), n._dock_hold_active and off_anchor(path[-1])[0] < 1e-9
          and math.hypot(path[0][0] - p[0], path[0][1] - p[1]) <= 0.0034)
    tr = n.map_odom_transform.transform.translation
    tr.x += 5.0                                   # something threw map->odom 5 m away
    before = (tr.x, tr.y, n.map_odom_yaw)
    tick()
    check('engaged hold still refuses a metres-scale correction (sanity bound)',
          (tr.x, tr.y, n.map_odom_yaw) == before and n._dock_hold_refusals == 1)
    n, tick, pose = docked_node(nt, ON)
    for _ in range(10):
        tick()
    tr = n.map_odom_transform.transform.translation
    tr.x += 5.0
    before = (tr.x, tr.y, n.map_odom_yaw)
    tick()
    check('... during the walk too', (tr.x, tr.y, n.map_odom_yaw) == before
          and n._dock_hold_refusals == 1)

    print('built: seed pulse before the hold engages (docked 0.1 s, hold after 2 s):')
    seed = pose_msg(ANCHOR)
    n, tick, pose = docked_node(nt, ON, docked_for=0.1)
    start = pose()
    n.callback_set_map_pose(seed, source='dock')
    check('the pulse itself writes nothing, a walk is started', pose() == start
          and n._dock_engage_off is not None and not n._dock_hold_active)
    path = [start]
    engaged_at = None
    for i in range(120):
        if i in (10, 25, 40):
            n.callback_set_map_pose(seed, source='dock')       # repeated pulses
        path.append(tick())
        if engaged_at is None and n._dock_hold_active:
            engaged_at = i
        if i == 5:
            early = (n._dock_hold_active, n._tracker_pose_suppressed(), n.init,
                     n._map_pose_target, n._georef_target, n._yaw_level,
                     R.CLOCK[0] - n._map_pose_last_accept['dock'])
    dd = [math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])]
    da = [abs(R.wrap(b[2] - a[2])) for a, b in zip(path, path[1:])]
    check('every tick <= 0.34 cm and <= 0.17 deg, the first one %.2f cm' % (100 * dd[0]),
          0.0 < dd[0] <= 0.0034 and max(dd) <= 0.0034 and max(da) <= D(0.17))
    check('hold engages at tick %s in the middle of the walk, no step there' % engaged_at,
          engaged_at is not None and 50 <= engaged_at <= 60 and off_anchor(path[engaged_at])[0] > 0.01
          and dd[engaged_at] <= 0.0034)
    done = next(i for i, p in enumerate(path) if off_anchor(p)[0] < 1e-9 and off_anchor(p)[1] < 1e-9)
    check('on the waypoint exactly after %.1f s, walk over' % (done / 30.0),
          60 <= done <= 72 and n._dock_engage_off is None)
    cross = [abs((p[0] - start[0]) * (ANCHOR[1] - start[1]) - (p[1] - start[1]) * (ANCHOR[0] - start[0]))
             / math.hypot(ANCHOR[0] - start[0], ANCHOR[1] - start[1]) for p in path]
    check('straight line to the waypoint (max %.1e m aside)' % max(cross), max(cross) < 1e-9)
    check('before the hold: dock_hold_active False, dock_suppressing True, dock source fresh, no slew left',
          early[0] is False and early[1] is True and early[3] is None and early[4] is None
          and early[5] is False and early[6] == 0.0)
    n0, tick0, pose0 = docked_node(nt, None, docked_for=0.1)
    n0.callback_set_map_pose(seed, source='dock')
    for _ in range(6):
        tick0()
    check('... the same flags as with 0 after its instant pulse (init %s)' % n0.init,
          (n0._dock_hold_active, n0._tracker_pose_suppressed(), n0.init, n0._map_pose_target,
           n0._georef_target, n0._yaw_level) == early[:6] and off_anchor(pose0())[0] < 1e-9
          and n0._map_odom_placed and n._map_odom_placed)
    n, tick, pose = docked_node(nt, ON, docked_for=0.1)
    n.callback_set_map_pose(seed, source='dock')
    for _ in range(10):
        p = tick()
    trk = pose_msg((ANCHOR[0] + 0.5, ANCHOR[1], ANCHOR[2]))
    n.callback_set_map_pose(trk, source='tracker')
    check('tracker correction during the walk: rejected (dock owns the pose), walk goes on',
          pose() == p and n._map_pose_target is None and 0.0 < math.hypot(
              tick()[0] - p[0], tick()[1] - p[1]) <= 0.0067)

    print('built: seed pulse immediate as before:')
    n, tick, pose = docked_node(nt, ON, placed=False, docked_for=0.1)
    n.callback_set_map_pose(seed, source='dock')
    check('fresh start (map->odom not placed yet)', off_anchor(pose())[0] < 1e-9
          and off_anchor(pose())[1] < 1e-9 and n._dock_engage_off is None and n._map_odom_placed)
    n, tick, pose = docked_node(nt, ON, err=(1.2, 0.0, 0.0), docked_for=0.1)
    n.callback_set_map_pose(seed, source='dock')
    check('1.2 m off', off_anchor(pose())[0] < 1e-9 and n._dock_engage_off is None)
    n, tick, pose = docked_node(nt, ON, err=(0.1, 0.0, D(20.0)), docked_for=0.1)
    n.callback_set_map_pose(seed, source='dock')
    check('20 deg off', off_anchor(pose())[0] < 1e-9 and off_anchor(pose())[1] < 1e-9
          and n._dock_engage_off is None)
    n, tick, pose = docked_node(nt, ON, docked_for=0.1)
    n.callback_dock_status(types.SimpleNamespace(data=3))
    n.callback_set_map_pose(seed, source='dock')
    check('not docked (dock status 3)', off_anchor(pose())[0] < 1e-9 and n._dock_engage_off is None)
    n, tick, pose = docked_node(nt, ON, docked_for=0.1)
    n._dock_status_rx_t = R.CLOCK[0] - 60.0
    n.callback_set_map_pose(seed, source='dock')
    check('dock feed stale', off_anchor(pose())[0] < 1e-9 and n._dock_engage_off is None)
    for err, placed, st in (((0.2, 0.1, D(2.0)), True, 0), ((0.2, 0.1, D(2.0)), False, 0),
                            ((1.2, 0.0, 0.0), True, 0), ((0.2, 0.1, D(2.0)), True, 3)):
        got = []
        for par in (None, ON):
            n, tick, pose = docked_node(nt, par, err=err, placed=placed, docked_for=0.1)
            n.callback_dock_status(types.SimpleNamespace(data=st))
            n.callback_set_map_pose(seed, source='dock')
            if par is ON and n._dock_engage_off is not None:
                for _ in range(200):
                    tick()
            t = n.map_odom_transform.transform.translation
            got.append((t.x, t.y, n.map_odom_yaw, n._map_odom_placed, n._map_pose_target))
        if err == (0.2, 0.1, D(2.0)) and placed and st == 0:
            check('walked pulse ends on the very map->odom the instant pulse writes',
                  got[0] == got[1])
        else:
            check('immediate case (placed %s, status %d, %.1f m): map->odom equal to 0'
                  % (placed, st, math.hypot(err[0], err[1])), got[0] == got[1])

    print('built: undock in the middle of the walk, then a dock pulse:')
    for label, docked_for in (('walk by the seed pulse, before the hold', 0.1),
                              ('walk by the hold', 10.0)):
        n, tick, pose = docked_node(nt, ON, docked_for=docked_for)
        if docked_for < 2.0:
            n.callback_set_map_pose(seed, source='dock')
        for _ in range(20):
            p = tick()
        left = off_anchor(p)[0]
        n.callback_dock_status(types.SimpleNamespace(data=1))        # undocking
        p2 = tick()
        check('%s: undock after 0.67 s leaves the pose where it got (%.1f cm short), walk over'
              % (label, 100 * left), p2 == p and 0.1 < left < 0.2
              and n._dock_engage_off is None and not n._dock_hold_active)
        for _ in range(30):
            tick()
        check('... the pose does not move on its own afterwards', pose() == p)
        n.callback_dock_status(types.SimpleNamespace(data=3))        # away from the dock
        n.callback_set_map_pose(seed, source='dock')
        check('... a dock pulse then passes at once', off_anchor(pose())[0] < 1e-9
              and off_anchor(pose())[1] < 1e-9)
        n, tick, pose = docked_node(nt, ON, docked_for=docked_for)
        if docked_for < 2.0:
            n.callback_set_map_pose(seed, source='dock')
        for _ in range(20):
            p = tick()
        n._dock_status_val = 1                  # undock edge, but no loop tick in between
        n._docked_since = None
        n.callback_set_map_pose(seed, source='dock')
        check('... and also when it comes before the next loop tick', off_anchor(pose())[0] < 1e-9
              and n._dock_engage_off is None)

    print('built: other writers, map change:')
    n, tick, pose = docked_node(nt, ON, docked_for=0.1)
    n.callback_set_map_pose(seed, source='dock')
    for _ in range(10):
        tick()
    tr = n.map_odom_transform.transform.translation
    tr.x += 0.5                                   # someone else wrote map->odom
    before = (tr.x, tr.y, n.map_odom_yaw)
    tick()
    check('walk before the hold gives way to another writer of map->odom',
          (tr.x, tr.y, n.map_odom_yaw) == before and n._dock_engage_off is None)
    n, tick, pose = docked_node(nt, ON, docked_for=0.1)
    n.callback_set_map_pose(seed, source='dock')
    for _ in range(10):
        p = tick()
    n._drop_dock_anchor('site', 'other')
    far = pose_msg((ANCHOR[0] + 40.0, ANCHOR[1] - 7.0, ANCHOR[2] + 1.0))
    n.callback_set_map_pose(far, source='dock')
    q = pose()
    check('map change during the walk: the new map\'s pulse (40 m away) is written at once',
          n._dock_engage_off is None and abs(q[0] - ANCHOR[0] - 40.0) < 1e-9
          and abs(q[1] - ANCHOR[1] + 7.0) < 1e-9)
    n, tick, pose = docked_node(nt, ON, docked_for=0.1)
    n._dock_anchor_pose = None                    # hold has no waypoint
    n.callback_set_map_pose(seed, source='dock')
    path = [tick() for _ in range(90)]
    check('no DOCKED waypoint cached for the hold: the pulse is still walked to its pose',
          off_anchor(path[-1])[0] < 1e-9 and n._dock_engage_off is None
          and not n._dock_hold_active
          and max(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(path, path[1:])) <= 0.0034)

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
