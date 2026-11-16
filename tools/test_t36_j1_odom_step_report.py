#!/usr/bin/env python3
"""T36 J1 (~odom_step_report): /nav_tf/loc_status and /nav_tf/odom_step say
when the pose of the nav EKF (= odom->base_link) moved by more than the
measured rates explain (wheels x dt, gyro x dt) and why: the node's own
set_pose (which path), a spike of the EKF's own twist (and which twist input
of the EKF was off as well), the heading anchor, or unknown. REPORT ONLY.

  - odom_step_residual / odom_step_cause / odom_step_fields (pure),
  - the node: parameter off = no subscriber, no publisher, no field; on = the
    fields, every other field of loc_status equal to off,
  - a built drive (wheels 0.3 m/s, EKF 30 Hz) with steps of each cause written
    through the node's own set_pose paths,
  - the recorded drive magtest3 (2026-11-14): how many steps the report sees
    (numbers; the recording does not say when set_pose ran).

Run:  python3 test_t36_j1_odom_step_report.py   (needs rospy - robot env)
"""
import json
import math
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

import numpy as np  # noqa: E402
import rospy  # noqa: E402
from geometry_msgs.msg import Pose  # noqa: E402

ON = {'odom_step_report': True}
FIELDS = ('odom_step_n', 'odom_step_max_xy_m', 'odom_step_max_yaw_deg', 'odom_step_spikes',
          'odom_step_causes', 'odom_step_last')
SUBS = []


class Sub(object):
    def __init__(self, name='', *a, **k):
        SUBS.append(name)

    def __getattr__(self, name):
        return lambda *a, **k: None


class Proxy(object):
    def __init__(self, *a, **k):
        pass

    def __call__(self, req):
        return None

    def __getattr__(self, name):
        return lambda *a, **k: None


class Drive(object):
    """Wheels at v, gyro at w; the EKF pose follows them exactly unless told
    otherwise. One tick = one EKF message (30 Hz)."""
    def __init__(self, nt, params, t0=9000.0):
        R.CLOCK[0] = t0
        del SUBS[:]
        self.n = R.make_node(nt, params)
        self.t, self.x, self.y, self.yaw = t0, 0.0, 0.0, 0.0
        self.v, self.w = 0.3, 0.0
        self.pub = getattr(self.n, '_osr_pub', None)
        if self.pub is not None:
            self.pub.keep = []

    def tick(self, n=1, dx=0.0, dyaw=0.0, ekf_vx=None):
        for _ in range(n):
            dt = 1.0 / 32.0                       # binary fraction: exact stamps
            self.t += dt
            R.CLOCK[0] = self.t + 0.005
            self.yaw += self.w * dt + dyaw
            self.x += (self.v * dt + dx) * math.cos(self.yaw)
            self.y += (self.v * dt + dx) * math.sin(self.yaw)
            self.n.callback_wheel_odom(R.odom_msg(self.t, 0, 0, 0, 0, self.v, 0.0))
            self.n.imu_msg = R.imu_msg(self.t, self.yaw, self.w)
            m = R.odom_msg(self.t, self.x, self.y, 0.0, self.yaw,
                           self.v if ekf_vx is None else ekf_vx, self.w)
            self.n.callback_ekf_wheel_nav_odom(m)
            if getattr(self.n, '_osr_enable', False):
                self.n._osr_on_ekf(m)
            dx = dyaw = 0.0
            ekf_vx = None

    def events(self):
        return [json.loads(m.data) for m in self.pub.keep]


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-72s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    nt = R.load('navi_transform_t36_j1')
    rospy.Subscriber = Sub
    rospy.ServiceProxy = Proxy
    rospy.wait_for_service = lambda *a, **k: None

    print('pure:')
    res, ryaw, dt = nt.odom_step_residual(10.0, 1.0, 2.0, 0.5, 10.03125, 1.0 + 0.3 * 0.03125 * math.cos(0.5),
                                          2.0 + 0.3 * 0.03125 * math.sin(0.5), 0.5, 0.3, 0.0)
    check('pose moved exactly wheels x dt: residual 0', res < 1e-12 and ryaw == 0.0 and dt == 0.03125)
    res, ryaw, _ = nt.odom_step_residual(10.0, 0.0, 0.0, 0.0, 10.03125, 0.03, 0.04, 0.02, 0.0, 0.0)
    check('standing, pose moved 5 cm and 0.02 rad: residual 5 cm, 0.02 rad',
          abs(res - 0.05) < 1e-12 and abs(ryaw - 0.02) < 1e-12)
    res, ryaw, _ = nt.odom_step_residual(10.0, 0.0, 0.0, 3.1, 10.03125, 0.0, 0.0, -3.1, 0.0, 0.0)
    check('yaw residual wraps through pi', abs(ryaw - (2 * math.pi - 6.2)) < 1e-9)
    cz = nt.odom_step_cause
    check('step 0.1 s after an own set_pose: set_pose:<path>',
          cz(True, 0.1, 'position_anchor', None, 0.0, 0.3) == ('set_pose:position_anchor', ''))
    check('... 0.3 s after it (limit 0.25): unknown',
          cz(True, 0.3, 'position_anchor', None, 0.3, 0.3) == ('unknown', ''))
    check('EKF linear.x 3.5 m/s at wheels 0.26: ekf_twist_spike, source ekf',
          cz(True, 2.0, 'position_anchor', None, 3.5, 0.26) == ('ekf_twist_spike', 'ekf'))
    check('... with /vo/odom at 3.4 m/s: source vo; with licp at 2 m/s: source licp',
          cz(True, None, '', None, 3.5, 0.26, vo_vx=3.4) == ('ekf_twist_spike', 'vo')
          and cz(True, None, '', None, 3.5, 0.26, vo_vx=0.25, licp_vx=2.0) == ('ekf_twist_spike', 'licp'))
    check('... right after a set_pose: the set_pose path, the spike source kept',
          cz(True, 0.03, 'emergency', None, 3.5, 0.26) == ('set_pose:emergency', 'ekf'))
    check('yaw-only step 0.1 s after diff_yaw moved: heading_anchor; with an xy step: unknown',
          cz(False, None, '', 0.1, 0.3, 0.3) == ('heading_anchor', '')
          and cz(True, None, '', 0.1, 0.3, 0.3) == ('unknown', ''))
    ev = [(100.0, 0.02, 0.1, 'set_pose:position_anchor', ''), (130.0, 0.05, -4.0, 'unknown', ''),
          (150.0, 0.17, 0.3, 'ekf_twist_spike', 'ekf')]
    f = nt.odom_step_fields(ev, 165.0, {'cause': 'ekf_twist_spike'})
    check('fields over the last 60 s: 2 steps, max 0.17 m / 4 deg, 1 spike, causes, last',
          f == {'odom_step_n': 2, 'odom_step_max_xy_m': 0.17, 'odom_step_max_yaw_deg': 4.0,
                'odom_step_spikes': 1, 'odom_step_causes': {'unknown': 1, 'ekf_twist_spike': 1},
                'odom_step_last': {'cause': 'ekf_twist_spike'}}, str(f))

    print('node, parameter off and on:')
    d_off = Drive(nt, None)
    subs_off = list(SUBS)
    d_on = Drive(nt, ON)
    subs_on = list(SUBS)
    check('off (default): no extra subscriber, no publisher, no state',
          d_off.n._osr_enable is False and not hasattr(d_off.n, '_osr_pub')
          and not hasattr(d_off.n, '_osr_events'))
    extra = sorted(subs_on[len(subs_off):]) if subs_on[:len(subs_off)] == subs_off else None
    more = sorted(set(subs_on)) != sorted(set(subs_off)) or len(subs_on) - len(subs_off)
    check('on: 3 subscribers more (EKF, /vo/odom, /licp/odom_cov), publisher /nav_tf/odom_step',
          len(subs_on) - len(subs_off) == 3 and d_on.pub.name == '/nav_tf/odom_step'
          and subs_on.count('/odometry/ekf_wheel_nav_odom') == subs_off.count('/odometry/ekf_wheel_nav_odom') + 1
          and '/licp/odom_cov' in subs_on, str((extra, more)))
    for d in (d_off, d_on):
        d.tick(60)
        d.n.publish_loc_status()
    j_off = json.loads(d_off.n.loc_status_pub.last.data)
    j_on = json.loads(d_on.n.loc_status_pub.last.data)
    check('off: no odom_step field', not [k for k in j_off if k.startswith('odom_step')])
    check('on: the six fields, every other field equal to off; a clean drive = 0 steps',
          all(k in j_on for k in FIELDS) and {k: v for k, v in j_on.items() if k not in FIELDS} == j_off
          and j_on['odom_step_n'] == 0 and j_on['odom_step_last'] is None and d_on.events() == [])

    print('built drive, steps of each cause:')
    d = Drive(nt, ON)
    n = d.n
    d.tick(30)
    # the position anchor writes the EKF (its own path) and the pose steps 3 cm
    n.ekf_wheel_nav_odom_msg.pose.pose.position.x = d.x
    n.ekf_wheel_nav_odom_msg.pose.pose.position.y = d.y
    n._set_anchor_pose_slewed(d.x + 0.03, d.y, 0.0)
    d.tick(1, dx=0.03)
    e = d.events()
    check('3 cm step after the position anchor wrote: set_pose:position_anchor',
          len(e) == 1 and e[0]['cause'] == 'set_pose:position_anchor' and abs(e[0]['xy_m'] - 0.03) < 1e-3
          and e[0]['since_set_pose_s'] is not None and e[0]['since_set_pose_s'] < 0.1, str(e[-1:]))
    n._set_anchor_pose_slewed(d.x + 0.005, d.y, 0.0)        # inside the deadband: no set_pose
    d.tick(12)
    d.tick(1, dx=0.02)
    e = d.events()
    check('2 cm step 0.4 s later, nothing written (deadband): unknown',
          len(e) == 2 and e[1]['cause'] == 'unknown', str(e[-1:]))
    d.tick(10)
    n.rtk_odom_msg.pose.pose.position.x = d.x + 0.04
    n.rtk_odom_msg.pose.pose.position.y = d.y
    n.ekf_wheel_nav_odom_msg.pose.pose.position.x = d.x
    n.ekf_wheel_nav_odom_msg.pose.pose.position.y = d.y

    def loop3sec():                    # the emergency write is recognised by its caller
        n.set_wheel_base_nav_pose()
    loop3sec()
    d.tick(1, dx=0.04)
    e = d.events()
    check('step after the emergency set_pose of loop3sec: set_pose:emergency',
          len(e) == 3 and e[2]['cause'] == 'set_pose:emergency', str(e[-1:]))
    d.tick(10)
    n.callback_set_pose(type('M', (), {'pose': type('P', (), {'pose': Pose()})()})())
    d.tick(1, dx=0.05)
    n._bridge['x'], n._bridge['y'] = d.x, d.y
    d.tick(10)
    n.set_bridge_live_pose()
    d.tick(1, dx=0.05)
    e = d.events()
    check('manual /nav_tf/set_pose and the bridge: set_pose:manual, set_pose:bridge',
          [x['cause'] for x in e[3:]] == ['set_pose:manual', 'set_pose:bridge'], str([x['cause'] for x in e]))
    d.tick(15)
    n._osr_vo = R.odom_msg(d.t, 0, 0, 0, 0, 0.29, 0.0)
    d.tick(1, dx=0.17, ekf_vx=3.45)
    e = d.events()
    check('17 cm step with EKF linear.x 3.45 m/s, inputs sane: ekf_twist_spike, source ekf',
          e[-1]['cause'] == 'ekf_twist_spike' and e[-1]['spike'] == 'ekf' and e[-1]['ekf_vx'] == 3.45
          and e[-1]['wheel_vx'] == 0.3 and e[-1]['vo_vx'] == 0.29 and e[-1]['licp_vx'] is None,
          str(e[-1:]))
    d.tick(15)
    n._osr_licp = R.odom_msg(d.t + 1.0 / 32.0, 0, 0, 0, 0, 2.9, 0.0)
    d.tick(1, dx=0.1, ekf_vx=2.2)
    check('... with /licp/odom_cov at 2.9 m/s: source licp', d.events()[-1]['spike'] == 'licp')
    d.tick(15)
    n.init = False
    n.diff_yaw += math.radians(4.0)                 # the heading anchor moved diff_yaw
    d.tick(1)
    d.tick(1, dyaw=math.radians(4.0))
    e = d.events()
    check('4 deg yaw step right after diff_yaw moved: heading_anchor',
          e[-1]['cause'] == 'heading_anchor' and abs(e[-1]['yaw_deg'] - 4.0) < 0.01, str(e[-1:]))
    d.tick(20)
    d.tick(1, dyaw=math.radians(-5.0))
    check('5 deg yaw step out of nothing: unknown', d.events()[-1]['cause'] == 'unknown'
          and abs(d.events()[-1]['yaw_deg'] + 5.0) < 0.01)
    n0 = len(d.events())
    d.w = 0.9
    d.tick(60)                                       # a turn the gyro explains
    d.w = 0.0
    d.tick(5, dx=0.004)                              # 4 mm: under the threshold
    check('turning at 0.9 rad/s with the gyro, 4 mm steps: nothing', len(d.events()) == n0)
    n.publish_loc_status()
    j = json.loads(n.loc_status_pub.last.data)
    check('loc_status: %d steps, max %.2f m / %.1f deg, %d spikes, causes %s'
          % (j['odom_step_n'], j['odom_step_max_xy_m'], j['odom_step_max_yaw_deg'],
             j['odom_step_spikes'], sorted(j['odom_step_causes'])),
          j['odom_step_n'] == 9 and j['odom_step_max_xy_m'] == 0.17
          and j['odom_step_max_yaw_deg'] == 5.0 and j['odom_step_spikes'] == 2
          and j['odom_step_causes']['unknown'] == 2 and j['odom_step_last']['cause'] == 'unknown'
          and j['odom_step_last']['age_s'] > 1.5)
    R.CLOCK[0] += 61.0
    n.publish_loc_status()
    j = json.loads(n.loc_status_pub.last.data)
    check('61 s later: the window is empty, the last step is still named',
          j['odom_step_n'] == 0 and j['odom_step_causes'] == {} and j['odom_step_last']['age_s'] > 61)
    gap = Drive(nt, ON)
    gap.tick(5)
    gap.t += 1.0                                     # the EKF was silent for 1 s
    gap.x += 0.3
    gap.tick(5)
    check('a 1 s gap between EKF messages is not judged', gap.events() == [])
    stale = Drive(nt, ON)
    stale.tick(5)
    stale.v = 0.0
    stale.tick(1)
    for k in range(20):                              # wheel odometry and IMU stopped coming
        stale.t += 1.0 / 32.0
        R.CLOCK[0] = stale.t
        stale.x += 0.05 if k >= 10 else 0.0          # 5 cm steps once they are 0.3 s old
        stale.n._osr_on_ekf(R.odom_msg(stale.t, stale.x, 0, 0, 0, 0.3, 0))
    check('wheel odometry / IMU older than 0.2 s: no verdict', stale.events() == [])
    bad = Drive(nt, ON)
    bad.n._osr_events = None
    bad.tick(3, dx=0.05)
    bad.n.publish_loc_status()
    check('a fault inside the report: the EKF callback and loc_status go on',
          'owner' in json.loads(bad.n.loc_status_pub.last.data))

    print('recorded drive magtest3 (2026-11-14), the report on the recorded EKF:')
    data = R.load_data()
    R.CLOCK[0] = float(data['ekf'][0, 0])
    n = R.make_node(nt, ON)
    n._osr_pub.keep = []
    ev = []
    for prio, key in enumerate(('wo', 'imu', 'ekf')):
        arr = data[key]
        ev.extend((arr[i, 0], prio, key, i) for i in range(len(arr)))
    ev.sort(key=lambda x: (x[0], x[1]))
    for t, _p, key, i in ev:
        R.CLOCK[0] = t
        r = data[key][i]
        if key == 'wo':
            n.callback_wheel_odom(R.odom_msg(*r[1:8]))
        elif key == 'imu':
            n.imu_msg = R.imu_msg(r[1], r[2], r[3])
        else:
            n._osr_on_ekf(R.odom_msg(*r[1:8]))
    e = [json.loads(m.data) for m in n._osr_pub.keep]
    xy = np.array([x['xy_m'] for x in e])
    yw = np.abs(np.array([x['yaw_deg'] for x in e]))
    mins = len(data['ekf']) / 30.0 / 60.0
    print('    %d EKF messages (%.1f min): %d steps = %.0f / min; xy > 1 cm %d (max %.1f cm), '
          '> 3 cm %d; yaw > 3 deg %d (max %.1f); twist spikes %d'
          % (len(data['ekf']), mins, len(e), len(e) / mins, (xy > 0.01).sum(), 100 * xy.max(),
             (xy > 0.03).sum(), (yw > 3.0).sum(), yw.max(), sum(1 for x in e if x['spike'])))
    check('the recorded drive has steps over 1 cm and the report finds them',
          (xy > 0.01).sum() > 50 and xy.max() < 0.5 and all(x['cause'] == 'unknown' or x['spike']
                                                            or x['cause'] == 'heading_anchor' for x in e))

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
