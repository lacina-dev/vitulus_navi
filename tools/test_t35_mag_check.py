#!/usr/bin/env python3
"""Offline test for T35 (2026-11-14): the magnetometer heading watchdog of
navi_transform. REPORT ONLY: it compares the map heading with
/bno085/mag_heading and sets mag_* fields in /nav_tf/loc_status.

Part 1 drives the pure functions (mag_check_step, mag_heading_diff,
mag_heading_step_event). Part 2 drives the node methods on a stub node (no ROS
master, no node init): a disabled watchdog adds nothing to loc_status, an
enabled one adds four fields and changes no heading state.
Run:  python3 test_t35_mag_check.py   (needs rospy — robot env / vitulus-dev)
"""
import json
import math
import os
import sys
import threading
import types
import importlib.util
import importlib.machinery

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _ros_env  # noqa: E402
_ros_env.ensure()


def _stub_missing_msgs():
    """Dev container without the robot's full devel space: sensor_msgs_ext and
    the generated vitulus_navi.srv are missing (only imported at module level)."""
    try:
        import sensor_msgs_ext.msg  # noqa: F401
    except ImportError:
        pkg = types.ModuleType('sensor_msgs_ext')
        msg = types.ModuleType('sensor_msgs_ext.msg')
        msg.magnetometer = msg.accelerometer = object
        pkg.msg = msg
        sys.modules['sensor_msgs_ext'] = pkg
        sys.modules['sensor_msgs_ext.msg'] = msg
    try:
        import vitulus_navi.srv  # noqa: F401
    except ImportError:
        try:
            import vitulus_navi as pkg
        except ImportError:
            pkg = types.ModuleType('vitulus_navi')
            pkg.__path__ = []
            sys.modules['vitulus_navi'] = pkg
        srv = types.ModuleType('vitulus_navi.srv')
        srv.Point_pose = srv.Point_poseResponse = object
        pkg.srv = srv
        sys.modules['vitulus_navi.srv'] = srv


_stub_missing_msgs()

import rospy  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402
from std_msgs.msg import String  # noqa: E402

NAVI_TF = os.path.join(HERE, '..', 'nodes', 'navi_transform')
RAD = math.radians
MAG_KEYS = ('mag_state', 'mag_diff_deg', 'mag_disagree', 'mag_check')


def load(name, path):
    spec = importlib.util.spec_from_loader(
        name, importlib.machinery.SourceFileLoader(name, path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


CLOCK = [500.0]


class Run(object):
    """Feeds mag_check_step at 10 Hz. nav / mag in degrees; mag is given as the
    MAP heading it stands for (the function gets mag - 90 deg, as the topic)."""

    def __init__(self, nt, cfg=None):
        self.nt = nt
        self.cfg = cfg
        self.st = nt.mag_check_new_state()
        self.t = 100.0
        self.edges = []
        self.out = None

    def go(self, secs, nav=0.0, mag=0.0, state='ok', age=0.05, dock=False,
           anchored=True, since=100.0, have_mag=True):
        for _ in range(int(round(secs * 10))):
            self.t += 0.1
            self.st, self.out = self.nt.mag_check_step(
                self.st, self.t, RAD(nav), RAD(mag - 90.0) if have_mag else None,
                state, age, dock, anchored, since, self.cfg)
            if self.out['edge']:
                self.edges.append((round(self.t, 1), self.out['edge']))
        return self.out


def yaw_quat(msg_orientation, yaw):
    msg_orientation.z = math.sin(yaw / 2.0)
    msg_orientation.w = math.cos(yaw / 2.0)


def make_node(nt, enable):
    """Stub with what publish_loc_status and the watchdog read."""
    n = object.__new__(nt.NavTfNode)
    n._gps_proven = False
    n._gps_good_since = None
    n._heading_good_after_acq = False
    n._map_coords_hysteresis_s = 5.0
    n._map_pose_priority = ['dock', 'relock', 'tracker', 'rtabmap', 'legacy']
    n._map_pose_last_accept = {}
    n._map_pose_hold_s = 5.0
    n._docked = False
    n._anchor_last_advance_t = None
    n._anchor_frozen_warn_s = 5.0
    n.position_fix_usable = False
    n.position_fix_usable_history = [0, 0, 0]
    n.fix_usable = False
    n.heading_fix_usable = False
    n._heading_last_good = 0.0
    n._heading_grace_s = 3.0
    n._rtabmap_confident = False
    n._ha_last_applied_t = None
    n._ha_frozen_since = None
    n._drift_report = None
    n._dock_status_val = 3
    n._dock_status_rx_t = CLOCK[0]
    n._dock_status_fail_closed = True
    n._dock_status_stale_s = 5.0
    n._dock_free_since = 0.0
    n._tracker_dock_grace_s = 8.0
    n._dock_hold_active = False
    n._dock_hold_refusals = 0
    n._dock_anchor_pose = None
    n._budget_used_m = 0.0
    n._budget_used_deg = 0.0
    n._last_published_owner = None
    n.is_indoor = False
    n.init = False
    n.diff_yaw = 0.4
    n.map_odom_yaw = 0.0
    n._map_odom_lock = threading.Lock()
    n.ekf_wheel_nav_odom_msg = Odometry()
    n.ekf_wheel_nav_odom_msg.pose.pose.orientation.w = 1.0
    n.published = []
    n.loc_status_pub = types.SimpleNamespace(
        publish=lambda m: n.published.append(m.data))
    n._mag_check_enable = enable
    if enable:
        n._mag_check_init()
    return n


def set_nav(n, deg):
    yaw_quat(n.ekf_wheel_nav_odom_msg.pose.pose.orientation, RAD(deg))


def mag_msg(map_deg, cov=0.02):
    m = Imu()
    yaw_quat(m.orientation, RAD(map_deg - 90.0))
    m.orientation_covariance[8] = cov
    return m


def status(state, extra=None):
    d = {'state': state, 'reasons': []}
    d.update(extra or {})
    return String(data=json.dumps(d))


def drive(n, secs, nav, mag, state='ok', cov=0.02, dock_feed=True, hook=None):
    """secs of 10 Hz magnetometer messages + 1 Hz status + 1 Hz loc_status."""
    for i in range(int(round(secs * 10))):
        CLOCK[0] += 0.1
        if dock_feed:
            n._dock_status_rx_t = CLOCK[0]      # 10 Hz dock feed
        if hook:
            hook(i)
        set_nav(n, nav)
        if i % 10 == 0:
            n.callback_mag_status(status(state))
        n.callback_mag_heading(mag_msg(mag, cov))
        if i % 10 == 9:
            n.publish_loc_status()
    return json.loads(n.published[-1])


def main():
    nt = load('navi_transform_t35', NAVI_TF)
    fails = []

    def check(name, cond):
        print('  %-72s %s' % (name, 'OK' if cond else 'FAIL'))
        if not cond:
            fails.append(name)

    print('pure: difference')
    d = nt.mag_heading_diff
    check('same heading -> 0', abs(d(RAD(30), RAD(30 - 90))) < 1e-9)
    check('mag 10 deg left of nav -> +10', abs(math.degrees(d(RAD(30), RAD(40 - 90))) - 10) < 1e-6)
    check('across +-180: nav 175, mag -175 -> +10',
          abs(math.degrees(d(RAD(175), RAD(-175 - 90))) - 10) < 1e-6)
    check('across +-180: nav -170, mag 160 -> -30',
          abs(math.degrees(d(RAD(-170), RAD(160 - 90))) + 30) < 1e-6)

    print('pure: agreement / disagreement')
    r = Run(nt)
    o = r.go(20, nav=40, mag=48)
    check('8 deg apart for 20 s -> compared, no disagree',
          o['check'] == 'ok' and not o['disagree'] and abs(o['diff_deg'] - 8) < 1e-6
          and not r.edges)
    r = Run(nt)
    o = r.go(4.8, nav=0, mag=30)
    check('30 deg apart for 4.8 s -> not yet', not o['disagree'] and o['diff_deg'] is not None)
    o = r.go(0.4, nav=0, mag=30)
    check('30 deg apart for > 5 s -> disagree', o['disagree'])
    check('raised exactly once, 5.0-5.2 s after the start',
          len(r.edges) == 1 and r.edges[0][1] == 'raised' and 105.0 <= r.edges[0][0] <= 105.2)
    r.go(30, nav=0, mag=30)
    check('stays raised, no repeated edge', r.out['disagree'] and len(r.edges) == 1)

    r = Run(nt)
    r.go(10, nav=0, mag=5)
    r.go(4.0, nav=0, mag=60)
    o = r.go(10, nav=0, mag=5)
    check('4 s excursion (60 deg) -> never reported', not o['disagree'] and not r.edges)
    r = Run(nt)
    for _ in range(6):
        r.go(4.0, nav=0, mag=40)
        r.go(0.2, nav=0, mag=10)
    check('over the limit with 0.2 s breaks each 4 s (95 %% of the time) -> reported once (%.1f s)'
          % (r.edges[0][0] - 100 if r.edges else -1), [e[1] for e in r.edges] == ['raised'])
    r = Run(nt)
    for _ in range(30):
        r.go(9.0, nav=0, mag=5)
        r.go(1.0, nav=0, mag=40)
    check('1 s excursions between 9 s of agreement for 5 min (30 s over in all) -> no report', not r.edges)
    r = Run(nt)
    for _ in range(12):
        r.go(3.0, nav=0, mag=5)
        r.go(4.5, nav=0, mag=40)
    check('4.5 s excursions between 3 s of agreement (60 % over) -> no report', not r.edges)
    r = Run(nt)
    r.go(100, nav=0, mag=3)
    t1 = r.t
    r.go(6, nav=0, mag=30)
    check('100 s of agreement, then 30 deg -> reported after %.1f s (earlier agreement does not delay)'
          % (r.edges[0][0] - t1 if r.edges else -1), len(r.edges) == 1 and r.edges[0][0] - t1 <= 5.2)
    r = Run(nt)
    o = r.go(20, nav=0, mag=25.0)
    check('exactly at the limit (25.0) -> no disagree', not o['disagree'])
    o = Run(nt).go(20, nav=0, mag=-31)
    check('negative side (-31 deg) -> disagree', o['disagree'] and o['diff_deg'] < 0)

    print('pure: across +-180')
    o = Run(nt).go(20, nav=178, mag=-176)
    check('nav 178 / mag -176 (6 deg) -> no disagree',
          not o['disagree'] and abs(o['diff_deg'] - 6) < 1e-6)
    o = Run(nt).go(20, nav=170, mag=-160)
    check('nav 170 / mag -160 (30 deg) -> disagree',
          o['disagree'] and abs(o['diff_deg'] - 30) < 1e-6)
    r = Run(nt)
    for k in range(200):                    # turning on the spot through +-180
        a = 170.0 + k * 0.5
        r.go(0.1, nav=a, mag=a + 4.0)
    check('turning through +-180 with 4 deg offset -> no disagree', not r.edges)

    print('pure: comparison not valid')
    for state in ('disturbed', 'uncalibrated', 'stale', None, 'something_new'):
        r = Run(nt)
        o = r.go(20, nav=0, mag=90, state=state)
        want = 'stale' if state == 'stale' else 'not_ok'
        check('state %-13s, 90 deg apart -> %s, no diff, no disagree' % (state, want),
              o['check'] == want and o['diff_deg'] is None and not o['disagree'])
    o = Run(nt).go(20, nav=0, mag=90, age=1.5)
    check('message 1.5 s old -> stale', o['check'] == 'stale' and not o['disagree'])
    o = Run(nt).go(20, nav=0, mag=90, age=0.9)
    check('message 0.9 s old -> compared', o['check'] == 'ok' and o['disagree'])
    o = Run(nt).go(20, nav=0, have_mag=False, age=None)
    check('no message ever -> stale', o['check'] == 'stale' and not o['disagree'])
    o = Run(nt).go(20, nav=0, mag=90, dock=True)
    check('in the dock -> dock, no diff, no disagree',
          o['check'] == 'dock' and o['diff_deg'] is None and not o['disagree'])
    o = Run(nt).go(20, nav=0, mag=90, anchored=False)
    check('heading not anchored (init) -> unanchored', o['check'] == 'unanchored' and not o['disagree'])
    o = Run(nt).go(20, nav=0, mag=90, dock=True, state='disturbed', anchored=False)
    check('dock wins over the other reasons', o['check'] == 'dock')

    print('pure: settle after an undock / heading step')
    r = Run(nt)
    for k in range(200):
        r.go(0.1, nav=0, mag=90, since=k * 0.1)
        if k == 79:
            check('7.9 s after the event -> settle, nothing reported',
                  r.out['check'] == 'settle' and r.out['diff_deg'] is None and not r.edges)
    check('disagree 8 + 5 s after the event',
          len(r.edges) == 1 and 112.9 <= r.edges[0][0] <= 113.2)
    o = Run(nt).go(20, nav=0, mag=90, since=None)
    check('no event known -> compared', o['check'] == 'ok' and o['disagree'])
    o = Run(nt, {'settle_s': 2.0, 'disagree_s': 1.0, 'disagree_deg': 10.0}).go(
        3.5, nav=0, mag=12, since=2.5)
    check('parameters are taken (10 deg / 1 s / settle 2 s)', o['disagree'])

    print('pure: how a report ends')
    r = Run(nt)
    r.go(6, nav=0, mag=30)
    o = r.go(20, nav=0, mag=20)
    check('back to 20 deg (under 25, over 15) -> stays raised', o['disagree'] and len(r.edges) == 1)
    o = r.go(600, nav=0, mag=20)
    check('... 20 deg for 10 min -> still raised (the headings do not agree)', o['disagree'])
    o = r.go(2.8, nav=0, mag=10)
    check('10 deg for 2.8 s -> still raised', o['disagree'])
    o = r.go(0.4, nav=0, mag=10)
    check('10 deg for > 3 s -> cleared', not o['disagree'] and r.edges[-1][1] == 'cleared')
    r = Run(nt)
    r.go(6, nav=0, mag=30)
    for _ in range(12):
        r.go(2.0, nav=0, mag=5)
        r.go(2.0, nav=0, mag=20)
    check('under 15 deg only half of the time -> not cleared', r.out['disagree'])
    r = Run(nt)
    r.go(6, nav=0, mag=30)
    for _ in range(12):
        r.go(2.0, nav=0, mag=5)
        r.go(2.0, nav=0, mag=40)
    check('agreeing and 40 deg in turns -> not cleared', r.out['disagree'] and len(r.edges) == 1)
    r = Run(nt)
    r.go(6, nav=0, mag=30)
    o = r.go(59.0, nav=0, mag=30, state='disturbed')
    check('raised, then magnetometer disturbed 59 s -> held', o['disagree'] and o['check'] == 'not_ok')
    o = r.go(1.5, nav=0, mag=30, state='disturbed')
    check('... 60 s after the last comparison -> dropped', not o['disagree'] and r.edges[-1][1] == 'cleared')
    r = Run(nt, {'hold_s': 10.0})
    r.go(6, nav=0, mag=30)
    o = r.go(10.5, nav=0, mag=30, age=2.0)
    check('hold_s parameter (10 s), magnetometer stale -> dropped', not o['disagree'] and o['check'] == 'stale')
    r = Run(nt)
    r.go(6, nav=0, mag=30)
    for _ in range(20):                     # one comparison every 50 s keeps it
        r.go(49.9, nav=0, mag=30, state='disturbed')
        r.go(0.1, nav=0, mag=30)
    check('one comparison over the limit every 50 s -> hold never runs out, stays raised',
          r.out['disagree'] and len(r.edges) == 1)
    r = Run(nt)
    r.go(6, nav=0, mag=30)
    o = r.go(0.1, nav=0, mag=30, since=0.0)
    check('raised, then a heading step (settle) -> dropped at once', not o['disagree'])
    r = Run(nt)
    r.go(6, nav=0, mag=30)
    o = r.go(0.1, nav=0, mag=30, dock=True)
    check('raised, then docked -> dropped at once', not o['disagree'])

    def sparse(ok_s, period_s, total_s, mag, pre=None, cfg=None):
        r = Run(nt, cfg)
        if pre:
            pre(r)
        t0, e0 = r.t, len(r.edges)
        while r.t - t0 < total_s:
            r.go(ok_s, nav=0, mag=mag)
            r.go(period_s - ok_s, nav=0, mag=mag, state='disturbed')
        return [(round(t - t0, 1), e) for t, e in r.edges[e0:]], r

    print('pure: lasting error, magnetometer ok only in sparse stretches (sliding window)')
    for ok_s, per, limit in ((1.0, 10.0, 55), (0.5, 2.5, 30), (2.0, 15.0, 35), (1.0, 20.0, 105),
                             (3.0, 30.0, 35), (0.1, 2.0, 105)):
        e, r = sparse(ok_s, per, 1800, 30)
        check('30 deg, ok %.1f s every %5.1f s (%2.0f %%) -> reported once, after %5.1f s'
              % (ok_s, per, 100 * ok_s / per, e[0][0] if e else -1),
              [x[1] for x in e] == ['raised'] and e[0][0] <= limit)
    e, r = sparse(5.5, 300.0, 900, 30)
    check('30 deg, ok 5.5 s every 300 s -> reported after %.1f s, dropped 60 s after the last '
          'comparison, reported again in the next stretch' % (e[0][0] if e else -1),
          [x[1] for x in e] == ['raised', 'cleared'] * 3 and e[0][0] <= 5.2
          and 64.0 <= e[1][0] <= 66.0)
    for ok_s, per in ((1.0, 30.0), (0.1, 3.0), (2.0, 60.0)):
        e, r = sparse(ok_s, per, 1800, 30)
        check('30 deg, ok %.1f s every %4.1f s (under 5 s per 120 s) -> not reported (documented)'
              % (ok_s, per), e == [])
    for ok_s, per, limit in ((1.0, 10.0, 35), (0.5, 5.0, 35), (2.0, 20.0, 25), (1.0, 12.0, 40), (0.1, 2.0, 65)):
        e, r = sparse(ok_s, per, 1800, 2, pre=lambda r: r.go(6, nav=0, mag=30))
        check('raised, then 2 deg with ok %.1f s every %4.1f s -> cleared once, after %5.1f s'
              % (ok_s, per, e[0][0] if e else -1), [x[1] for x in e] == ['cleared'] and e[0][0] <= limit)
    e, r = sparse(1.0, 10.0, 600, 30, cfg={'window_s': 30.0})
    check('window_s 30: ok 1 s every 10 s (3 s per window) -> not reported', e == [])
    e, r = sparse(1.0, 10.0, 600, 30, cfg={'window_s': 30.0, 'disagree_s': 2.5})
    check('window_s 30, disagree_s 2.5 -> reported', [x[1] for x in e] == ['raised'])

    print('pure: short excursions')
    r = Run(nt)
    r.go(1.2, nav=0, mag=30)
    for _ in range(5):                      # nothing but the excursions was compared
        r.go(25.0, nav=0, mag=0, state='disturbed')
        r.go(1.2, nav=0, mag=30)
    check('1.2 s at 30 deg every 26 s, NO agreeing comparison in between -> reported (%.0f s): '
          'all that was compared in 2 min was over the limit'
          % (r.edges[0][0] - 100 if r.edges else -1), [x[1] for x in r.edges] == ['raised'])
    r = Run(nt)
    for _ in range(40):                     # the same, each excursion inside an ok stretch
        r.go(1.0, nav=0, mag=3)
        r.go(1.0, nav=0, mag=30)
        r.go(1.0, nav=0, mag=3)
        r.go(23.0, nav=0, mag=0, state='disturbed')
    check('1 s at 30 deg every 26 s inside agreeing ok stretches (1 s before and after) -> never', not r.edges)
    r = Run(nt)
    for _ in range(40):
        r.go(0.5, nav=0, mag=3)
        r.go(2.0, nav=0, mag=30)
        r.go(23.5, nav=0, mag=0, state='disturbed')
    check('2 s at 30 deg after 0.5 s of agreement in every ok stretch (80 %% of what is compared) '
          '-> reported (%.0f s)' % (r.edges[0][0] - 100 if r.edges else -1),
          bool(r.edges) and r.edges[0][1] == 'raised')
    r = Run(nt)
    for _ in range(40):
        r.go(1.0, nav=0, mag=3)
        r.go(2.0, nav=0, mag=30)
        r.go(23.0, nav=0, mag=0, state='disturbed')
    check('2 s at 30 deg after 1 s of agreement in every ok stretch (67 %) -> never', not r.edges)
    r = Run(nt, {'share': 0.5})
    for _ in range(12):
        r.go(3.0, nav=0, mag=5)
        r.go(4.5, nav=0, mag=40)
    check('share parameter 0.5: 60 % over -> reported', bool(r.edges) and r.edges[0][1] == 'raised')
    r = Run(nt)
    r.go(3.0, nav=0, mag=30)
    r.go(125.0, nav=0, mag=30, state='disturbed')
    o = r.go(4.0, nav=0, mag=30)
    check('3 s over, 125 s gap (out of the window), 4 s over -> no report', not o['disagree'])
    o = r.go(1.5, nav=0, mag=30)
    check('... 5 s after the gap -> disagree', o['disagree'])
    r = Run(nt)
    r.go(3.0, nav=0, mag=30)
    r.go(5.0, nav=0, mag=30, state='disturbed')
    r.go(0.5, nav=0, mag=30, since=1.0)
    o = r.go(4.0, nav=0, mag=30)
    check('a heading step (settle) empties the window', not o['disagree'])
    r = Run(nt)
    r.go(700, nav=0, mag=3)
    check('window stays bounded (120 s at 10 Hz)', len(r.st['win']) <= 1202)
    st0 = nt.mag_check_new_state()
    nt.mag_check_step(st0, 1.0, 0.0, 1.0, 'ok', 0.0, False, True, None)
    check('the state passed in is not modified', st0 == nt.mag_check_new_state())

    print('pure: clock jumps, non-finite input')
    r = Run(nt)
    r.go(0.2, nav=0, mag=30)
    r.t += 10.0
    o = r.go(0.1, nav=0, mag=30)
    check('two samples over the limit 10 s apart (clock jump) -> no report', not o['disagree'] and not r.edges)
    r = Run(nt)
    for _ in range(4):
        r.go(0.1, nav=0, mag=30)
        r.t += 5.0
    check('a jump adds at most 1 s per step', not r.edges)
    r = Run(nt)
    r.go(4.0, nav=0, mag=30)
    r.t += 500.0
    o = r.go(0.5, nav=0, mag=30)
    check('4 s over, clock +500 s (out of the window), 0.5 s over -> no report', not o['disagree'])
    r = Run(nt)
    r.go(4.0, nav=0, mag=30)
    r.t -= 100.0
    o = r.go(2.0, nav=0, mag=30)
    check('clock back by 100 s -> counted from zero, no report', not o['disagree'])
    o = r.go(3.5, nav=0, mag=30)
    check('... and reported 5 s later', o['disagree'])
    for bad in (float('nan'), float('inf'), -float('inf')):
        st, o = nt.mag_check_step(nt.mag_check_new_state(), 5.0, bad, 0.1, 'ok', 0.0, False, True, None)
        check('nav yaw %s -> error, no diff' % bad, o['check'] == 'error' and o['diff_deg'] is None and not o['disagree'])
        st, o = nt.mag_check_step(nt.mag_check_new_state(), 5.0, 0.1, bad, 'ok', 0.0, False, True, None)
        check('mag yaw %s -> error, no diff' % bad, o['check'] == 'error' and o['diff_deg'] is None)
    st, o = nt.mag_check_step(nt.mag_check_new_state(), 5.0, 0.0, 0.1, 'ok', float('nan'), False, True, None)
    check('age nan -> stale', o['check'] == 'stale')
    st, o = nt.mag_check_step(nt.mag_check_new_state(), 5.0, 0.0, 0.1, 'ok', -3.0, False, True, None)
    check('negative age (clock back) -> stale', o['check'] == 'stale')
    r = Run(nt)
    r.go(6, nav=0, mag=30)
    r.st, o = nt.mag_check_step(r.st, r.t + 0.1, float('nan'), 0.1, 'ok', 0.0, False, True, None)
    check('raised, then one non-finite step -> held (like not_ok)', o['disagree'] and o['check'] == 'error')

    print('pure: heading step detector (net move >= 8 deg in 3 s)')
    ev = nt.mag_heading_step_event
    h = []
    fired = [ev(h, 10.0 + k * 0.1, 0.4 + RAD(0.2) * math.sin(k)) for k in range(300)]
    check('0.2 deg jitter of the offset -> no event', not any(fired))
    h = []
    fired = [ev(h, 10.0 + k * 0.1, 0.4 + RAD(1.6) * math.sin(2 * math.pi * k * 0.1)) for k in range(600)]
    check('+-1.6 deg wobble at 1 Hz for 60 s -> no event', not any(fired))
    for per in (0.7, 2.0, 3.0, 6.0, 11.0):
        h = []
        fired = [ev(h, 10.0 + k * 0.1, 0.4 + RAD(2.5) * math.sin(2 * math.pi * k * 0.1 / per)) for k in range(600)]
        check('+-2.5 deg wobble, period %.1f s -> no event' % per, not any(fired))
    h = []
    fired = [ev(h, 10.0 + k * 0.1, 0.4 + RAD(2.0) * ((k // 7) % 3 - 1)) for k in range(600)]
    check('2 deg re-anchor steps back and forth (as on the drives) -> no event', not any(fired))
    h = []
    fired = [ev(h, 10.0 + k * 0.1, 0.4 + (RAD(20) if k >= 50 else 0.0)) for k in range(100)]
    check('20 deg step (set_heading) -> one event at the step',
          fired.index(True) == 50 and sum(fired) == 1)
    h = []
    fired = [ev(h, 10.0 + k * 0.1, 0.4 + (RAD(-9) if k >= 50 else 0.0)) for k in range(100)]
    check('-9 deg step -> one event', fired.index(True) == 50 and sum(fired) == 1)
    h = []
    fired = [ev(h, 10.0 + k * 0.1, 0.4 + RAD(0.5) * min(k, 40)) for k in range(100)]
    last = max(i for i, f in enumerate(fired) if f)
    check('5 deg/s slew for 4 s (hand-over) -> events during the slew, none after',
          sum(fired) == 2 and 16 <= fired.index(True) <= 17 and last <= 40)
    h = []
    fired = [ev(h, 10.0 + k * 0.1, 0.4 + RAD(0.2) * min(k, 100)) for k in range(200)]
    check('2 deg/s slew (slower than 8 deg / 3 s) -> not seen (documented limit)', not any(fired))
    h = []
    fired = [ev(h, 10.0 + k * 0.1, RAD(179) + RAD(0.1) * math.sin(k) * 15) for k in range(100)]
    check('offset jitter across +-180 -> no event', not any(fired))
    h = []
    fired = [ev(h, 10.0 + k * 0.1, RAD(175) + (RAD(12) if k >= 50 else 0.0)) for k in range(100)]
    check('12 deg step across +-180 -> one event', sum(fired) == 1)
    h = []
    ev(h, 10.0, 0.4)
    check('step seen only after a 30 s gap -> event', ev(h, 40.0, 0.4 + RAD(10)))
    h = []
    for k in range(1000):
        ev(h, 10.0 + k * 0.1, 0.4)
    check('history stays bounded', len(h) <= 35)
    h = []
    for k in range(20):
        ev(h, 10.0 + k * 0.1, 0.4)
    check('non-finite offset -> no event, history restarted',
          not ev(h, 12.0, float('nan')) and h == [] and not ev(h, 12.1, 2.0) and len(h) == 1)
    check('clock back -> no event, history restarted', not ev(h, 5.0, 3.0) and h == [(5.0, 3.0)])

    print('node: watchdog disabled')
    real_gt = rospy.get_time
    rospy.get_time = lambda: CLOCK[0]
    subs = []
    real_sub = rospy.Subscriber
    rospy.Subscriber = lambda *a, **k: subs.append(a[0]) or types.SimpleNamespace()
    real_param = rospy.get_param
    rospy.get_param = lambda name, default=None: default
    real_logs = rospy.logwarn_throttle, rospy.logwarn, rospy.loginfo
    rospy.logwarn_throttle = rospy.logwarn = rospy.loginfo = lambda *a, **k: None
    try:
        n = make_node(nt, False)
        n.publish_loc_status()
        st = json.loads(n.published[-1])
        check('loc_status published, no mag_* field', 'owner' in st and not [k for k in st if k.startswith('mag')])
        check('no subscriber, no watchdog attribute',
              not subs and not [a for a in vars(n) if a.startswith('_mag_') and a != '_mag_check_enable'])
        n2 = object.__new__(nt.NavTfNode)
        n2.__dict__.update({k: v for k, v in vars(make_node(nt, False)).items()
                            if k != '_mag_check_enable'})
        n2.published = []
        n2.loc_status_pub = types.SimpleNamespace(publish=lambda m: n2.published.append(m.data))
        n2.publish_loc_status()
        check('stub without the attribute at all (older tests) still publishes',
              len(n2.published) == 1 and n2.published[0] == n.published[-1])

        print('node: watchdog enabled')
        n = make_node(nt, True)
        check('two subscribers on the default topics',
              subs == ['/bno085/mag_heading', '/bno085/mag_heading/status'])
        n.publish_loc_status()
        st = json.loads(n.published[-1])
        check('before any magnetometer message: stale, null, false',
              [st.get(k) for k in MAG_KEYS] == ['stale', None, False, 'stale'])
        base = {k: v for k, v in st.items() if k not in MAG_KEYS}
        n0 = make_node(nt, False)
        n0.publish_loc_status()
        check('the other loc_status fields are the same as with the watchdog off',
              base == json.loads(n0.published[-1]))
        st = drive(n, 12, nav=20, mag=26)
        check('agreement: ok / 6.0 / false / ok',
              [st.get(k) for k in MAG_KEYS] == ['ok', 6.0, False, 'ok'])
        frozen = (n.diff_yaw, n.map_odom_yaw, n.init, n._heading_good_after_acq,
                  n._gps_proven, n._dock_hold_active)
        warns, infos = [], []
        real_warn, real_info = rospy.logwarn, rospy.loginfo
        rospy.logwarn = lambda *a, **k: warns.append(a[0] % a[1:] if len(a) > 1 else a[0])
        rospy.loginfo = lambda *a, **k: infos.append(a[0] % a[1:] if len(a) > 1 else a[0])
        try:
            st = drive(n, 4, nav=20, mag=55)
            check('35 deg apart for 4 s -> diff shown, not yet disagree',
                  st['mag_diff_deg'] == 35.0 and st['mag_disagree'] is False and not warns)
            st = drive(n, 3, nav=20, mag=55)
            check('35 deg apart for 7 s -> mag_disagree true', st['mag_disagree'] is True)
            drive(n, 20, nav=20, mag=55)
            mw = [w for w in warns if 'mag_check' in w]
            check('exactly one warning, with the numbers',
                  len(mw) == 1 and '+35.0 deg' in mw[0] and 'report only' in mw[0])
            st = drive(n, 2, nav=20, mag=22)
            check('back in agreement for 2 s -> still raised', st['mag_disagree'] is True)
            st = drive(n, 3, nav=20, mag=22)
            mi = [w for w in infos if 'mag_check' in w and 'ENABLED' not in w]
            check('... for 5 s -> cleared, exactly one info',
                  st['mag_disagree'] is False and len(mi) == 1 and 'CLEARED' in mi[0])
        finally:
            rospy.logwarn, rospy.loginfo = real_warn, real_info
        check('heading state untouched (diff_yaw, map_odom_yaw, init, heading_good, ...)',
              frozen == (n.diff_yaw, n.map_odom_yaw, n.init, n._heading_good_after_acq,
                         n._gps_proven, n._dock_hold_active))

        st = drive(n, 12, nav=20, mag=80, state='disturbed', cov=1e3)
        check('disturbed: state shown, not_ok, null, false',
              [st.get(k) for k in MAG_KEYS] == ['disturbed', None, False, 'not_ok'])
        st = drive(n, 12, nav=20, mag=80, state='ok', cov=1e3)
        check('status ok but the message has the 1e3 covariance -> not compared',
              st['mag_check'] == 'not_ok' and st['mag_state'] == 'disturbed')
        st = drive(n, 3, nav=20, mag=24)
        n._dock_status_val = 0
        st = drive(n, 12, nav=20, mag=80)
        check('docked (dock_status 0) -> dock', st['mag_check'] == 'dock' and st['mag_disagree'] is False)
        n._dock_status_val = 1
        st = drive(n, 2, nav=20, mag=80)
        check('undocking (dock_status 1) -> dock', st['mag_check'] == 'dock')
        n._dock_status_val = 3
        n._dock_free_since = CLOCK[0]
        st = drive(n, 7, nav=20, mag=80)
        check('7 s after leaving the dock -> settle', st['mag_check'] == 'settle' and st['mag_diff_deg'] is None)
        st = drive(n, 4, nav=20, mag=80)
        check('11 s after -> compared (60 deg), not yet disagree',
              st['mag_check'] == 'ok' and st['mag_diff_deg'] == 60.0 and not st['mag_disagree'])
        st = drive(n, 3, nav=20, mag=80)
        check('14 s after -> disagree', st['mag_disagree'] is True)
        n._dock_hold_active = True
        st = drive(n, 1, nav=20, mag=80)
        check('dock hold active -> dock, disagree dropped',
              st['mag_check'] == 'dock' and st['mag_disagree'] is False)
        n._dock_hold_active = False

        st = drive(n, 12, nav=20, mag=24)
        n.diff_yaw += RAD(40)                       # /nav_tf/set_heading
        st = drive(n, 6, nav=60, mag=110)
        check('heading step (diff_yaw +40 deg) -> settle for 8 s', st['mag_check'] == 'settle')
        st = drive(n, 9, nav=60, mag=110)
        check('... then compared and reported', st['mag_check'] == 'ok' and st['mag_disagree'] is True)
        n.map_odom_yaw = RAD(-30)                   # map-pose yaw correction
        st = drive(n, 2, nav=60, mag=110)
        check('map_odom_yaw step -> settle; nav yaw = odom yaw + map_odom_yaw',
              st['mag_check'] == 'settle' and st['mag_disagree'] is False)
        st = drive(n, 8, nav=60, mag=30 + 5)
        check('map heading 60 - 30 = 30, mag 35 -> diff 5.0', st['mag_diff_deg'] == 5.0)
        n.map_odom_yaw = 0.0
        drive(n, 10, nav=60, mag=62)

        n.init = True
        st = drive(n, 3, nav=60, mag=140)
        check('init (no anchor) -> unanchored', st['mag_check'] == 'unanchored')
        n.init = False
        st = drive(n, 3, nav=60, mag=140)
        check('first anchor -> settle', st['mag_check'] == 'settle')
        drive(n, 10, nav=60, mag=62)
        n.is_indoor = True
        st = drive(n, 3, nav=60, mag=140)
        check('indoor (map->odom not ours) -> unanchored', st['mag_check'] == 'unanchored')
        n.is_indoor = False
        drive(n, 12, nav=60, mag=62)

        CLOCK[0] += 2.0                             # magnetometer went silent
        n._dock_status_rx_t = CLOCK[0]
        n.publish_loc_status()
        st = json.loads(n.published[-1])
        check('no message for 2 s -> stale at the next loc_status',
              st['mag_check'] == 'stale' and st['mag_diff_deg'] is None)
        CLOCK[0] += 3.0
        n._dock_status_rx_t = CLOCK[0]
        n.publish_loc_status()
        st = json.loads(n.published[-1])
        check('status silent > 3 s -> mag_state stale', st['mag_state'] == 'stale')
        n.callback_mag_status(String(data='not json'))
        n.callback_mag_status(String(data='[1, 2]'))
        n.callback_mag_status(status('ok', {'new_key': {'a': 1}, 'reasons': None}))
        check('broken / extended status JSON is tolerated', n._mag_state == 'ok')
        m = mag_msg(0.0)
        m.orientation.z = m.orientation.w = float('nan')
        rx = n._mag_rx_t
        n.callback_mag_heading(m)
        check('non-finite heading message is ignored', n._mag_rx_t == rx)

        print('node: dock state unknown')
        n = make_node(nt, True)
        n._dock_status_val = None
        n._dock_status_rx_t = 0.0
        st = drive(n, 20, nav=20, mag=80, dock_feed=False)
        check('dock_status never received -> dock, no diff, no disagree (as dock_suppressing)',
              [st.get(k) for k in MAG_KEYS] == ['ok', None, False, 'dock'] and st['dock_suppressing'] is True)
        n = make_node(nt, True)
        st = drive(n, 12, nav=20, mag=24)
        check('fresh feed, undocked -> compared', st['mag_check'] == 'ok')
        st = drive(n, 20, nav=20, mag=80, dock_feed=False)
        check('dock feed stale (> 5 s, last value undocked) -> dock, no disagree',
              st['mag_check'] == 'dock' and st['mag_disagree'] is False and st['dock_suppressing'] is True)
        st = drive(n, 2, nav=20, mag=80)
        check('feed back -> compared again', st['mag_check'] == 'ok')

        print('node: non-finite values, failing tick')

        def strict(payload):
            def bad(c):
                raise ValueError(c)
            return json.loads(payload, parse_constant=bad)
        n = make_node(nt, True)
        drive(n, 12, nav=20, mag=24)
        o = n.ekf_wheel_nav_odom_msg.pose.pose.orientation
        o.z = o.w = float('nan')
        n.callback_mag_status(status('ok'))
        n.callback_mag_heading(mag_msg(24))
        n.publish_loc_status()
        try:
            st = strict(n.published[-1])
        except ValueError:
            st = {}
        check('NaN odometry quaternion -> strict JSON, error, null',
              st.get('mag_check') == 'error' and st.get('mag_diff_deg', 0) is None and 'owner' in st)
        n = make_node(nt, True)
        drive(n, 12, nav=20, mag=24)
        n.map_odom_yaw = float('nan')
        drive(n, 2, nav=20, mag=24)
        try:
            st = strict(n.published[-1])
        except ValueError:
            st = {}
        check('NaN map_odom_yaw -> strict JSON, error, null, false',
              [st.get(k, 0) for k in MAG_KEYS] == ['ok', None, False, 'error'])
        n.map_odom_yaw = 0.0
        n.diff_yaw = float('inf')
        drive(n, 2, nav=20, mag=24)
        try:
            st = strict(n.published[-1])
        except ValueError:
            st = {}
        check('inf diff_yaw -> strict JSON, payload complete', 'owner' in st and st.get('mag_check') in ('ok', 'settle'))
        n.diff_yaw = 0.4
        bad_payloads = 0
        for p in n.published:
            try:
                strict(p)
            except ValueError:
                bad_payloads += 1
        check('no NaN / Infinity token in any payload of the run', bad_payloads == 0)

        n = make_node(nt, True)
        drive(n, 16, nav=20, mag=55)
        check('(35 deg apart -> raised before the failure)', json.loads(n.published[-1])['mag_disagree'] is True)
        n.ekf_wheel_nav_odom_msg = None
        k = len(n.published)
        thr = []
        real_thr = rospy.logwarn_throttle
        rospy.logwarn_throttle = lambda period, msg, *a: thr.append(msg)
        try:
            n.publish_loc_status()
            n.callback_mag_heading(mag_msg(24))
        finally:
            rospy.logwarn_throttle = real_thr
        st = json.loads(n.published[-1])
        check('tick raises -> loc_status still published, error / null / false (no old result)',
              len(n.published) == k + 1 and 'owner' in st
              and [st.get(m, 0) for m in MAG_KEYS[1:]] == [None, False, 'error'])
        check('... logged (throttled), the heading callback survives', len(thr) == 2 and 'mag_check' in thr[0])

        print('node: diff_yaw wobble is not a heading step')
        n = make_node(nt, True)

        def wobble(i):
            n.diff_yaw = 0.4 + RAD(1.6) * math.sin(2 * math.pi * i * 0.1)
        st = drive(n, 12, nav=20, mag=24)
        k = len(n.published)
        drive(n, 40, nav=20, mag=24, hook=wobble)
        checks = set(json.loads(p)['mag_check'] for p in n.published[k:])
        check('+-1.6 deg at 1 Hz for 40 s -> compared all the time, never settle', checks == {'ok'})
        st = drive(n, 8, nav=20, mag=55, hook=wobble)
        check('... and a 35 deg error is reported while it wobbles', st['mag_disagree'] is True)

        print('node: the switch in __init__')
        import inspect
        src = inspect.getsource(nt.NavTfNode.__init__)
        check('__init__ calls _mag_check_setup() and nothing else of the watchdog',
              src.count('self._mag_check_setup()') == 1 and src.count('_mag_') == 1)
        del subs[:]
        asked = []
        for value in (False, True):
            rospy.get_param = lambda name, default=None, v=value: (
                asked.append(name) or (v if name == '~mag_check_enable' else default))
            m = object.__new__(nt.NavTfNode)
            m._mag_check_setup()
            if not value:
                check('~mag_check_enable false -> no subscriber, one parameter read, no state',
                      subs == [] and asked == ['~mag_check_enable']
                      and list(vars(m)) == ['_mag_check_enable'] and m._mag_check_enable is False)
            else:
                check('~mag_check_enable true -> the two subscribers and the state',
                      subs == ['/bno085/mag_heading', '/bno085/mag_heading/status']
                      and m._mag_check_enable is True and hasattr(m, '_mag_st')
                      and '~mag_disagree_window_s' in asked and '~mag_step_deg' in asked)
        rospy.get_param = lambda name, default=None: (
            {'~mag_check_enable': True, '~mag_heading_topic': '/x/h', '~mag_status_topic': '/x/s',
             '~mag_disagree_deg': 15.0}.get(name, default))
        del subs[:]
        m = object.__new__(nt.NavTfNode)
        m._mag_check_setup()
        check('topic and limit parameters are taken',
              subs == ['/x/h', '/x/s'] and m._mag_cfg['disagree_deg'] == 15.0)
    finally:
        rospy.get_time, rospy.Subscriber, rospy.get_param = real_gt, real_sub, real_param
        rospy.logwarn_throttle, rospy.logwarn, rospy.loginfo = real_logs

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
