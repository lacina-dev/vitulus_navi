#!/usr/bin/env python3
"""The mission during a cmd_vel_mux hold: REAL mower_unit_smach states, real
actionlib, fake MBF, on a private master (GW_TEST_PORT, default 11831).

  python3 tools/test_hold_gate.py      refuses :11311, starts its own roscore

  1 EXE_PLANNER_PATH: goal cancelled, no stall/recovery however long the hold,
    afterwards 'replan_needed' = a new window from where the robot stands
  2 STOP during that hold ends it at once
  3 control: with no hold a stopped robot does stall ('aborted'), so 1 proves it
  4 wrap_tree: no state starts while held, nested containers too; STOP passes
  5 WaitForRpmReached: the blade held off does not run the 30 s out
  6 DETOUR under a hold: no line skipped, blade back on; without a hold 'no_detour'
  7 REMOW: a hold does not use up the one attempt
"""
import os
import signal
import subprocess
import sys
import threading
import time

PORT = int(os.environ.get('GW_TEST_PORT', '11831'))
if PORT == 11311:
    sys.exit('refusing: 11311 is the robot master')
os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:%d' % PORT
os.environ['ROS_IP'] = '127.0.0.1'
os.environ.pop('ROS_HOSTNAME', None)
if os.environ.get('GW_TEST_HOME'):
    os.environ['ROS_HOME'] = os.environ['GW_TEST_HOME']
    os.environ['ROS_LOG_DIR'] = os.environ['GW_TEST_HOME'] + '/log'

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))

FAIL = []


def ok(label, cond, detail=''):
    print('  %-62s %s%s' % (label, 'ok' if cond else 'FAIL', '' if cond else ' (%s)' % (detail,)),
          flush=True)
    if not cond:
        FAIL.append(label)


class UD(object):
    pass


def main():
    core = subprocess.Popen(['roscore', '-p', str(PORT)], stdout=subprocess.DEVNULL,
                            stderr=subprocess.DEVNULL, start_new_session=True)
    try:
        time.sleep(4.0)
        run()
    finally:
        print('FAILED: %s' % FAIL if FAIL else 'ALL OK', flush=True)
        os.killpg(core.pid, signal.SIGTERM)
        try:
            core.wait(5)
        except subprocess.TimeoutExpired:
            os.killpg(core.pid, signal.SIGKILL)
        os._exit(1 if FAIL else 0)     # rospy threads stay behind otherwise


def run():
    import actionlib
    import rospy
    import smach
    import tf2_ros
    from geometry_msgs.msg import PoseStamped, TransformStamped
    from mbf_msgs.msg import (ExePathAction, ExePathResult, GetPathAction, GetPathResult)
    from mbf_msgs.srv import CheckPath, CheckPathResponse
    from nav_msgs.msg import Path
    from std_msgs.msg import Bool
    from vitulus_msgs.msg import Mower

    rospy.init_node('hold_gate_test', disable_signals=True)
    import mower_unit_smach.states as st
    from mower_unit_smach.hold_gate import HOLD
    from mower_unit_smach.publishers import MowerPublishers

    robot = {'x': 0.0, 'moving': True}
    server = {'goals': 0, 'cancels': 0, 'abort_after': None, 'finish_after': None}

    def exe_cb(goal):
        server['goals'] += 1
        t0 = time.monotonic()
        while not rospy.is_shutdown():
            if exe.is_preempt_requested():
                server['cancels'] += 1
                exe.set_preempted()
                return
            el = time.monotonic() - t0
            if server['abort_after'] is not None and el > server['abort_after']:
                exe.set_aborted(ExePathResult(outcome=103))
                return
            if server['finish_after'] is not None and el > server['finish_after']:
                exe.set_succeeded(ExePathResult(outcome=0))
                return
            time.sleep(0.05)

    exe = actionlib.SimpleActionServer('/move_base_flex/exe_path', ExePathAction, exe_cb, False)
    exe.start()

    def get_cb(goal):
        p = Path()
        p.header.frame_id = 'map'
        p.poses = [PoseStamped(), PoseStamped()]
        gp.set_succeeded(GetPathResult(outcome=0, path=p, cost=1.0))

    gp = actionlib.SimpleActionServer('/move_base_flex/get_path', GetPathAction, get_cb, False)
    gp.start()
    rospy.Service('/move_base_flex/check_path_cost', CheckPath,
                  lambda r: CheckPathResponse(cost=0))
    br = tf2_ros.TransformBroadcaster()

    def tf_loop():
        r = rospy.Rate(20)
        while not rospy.is_shutdown():
            if robot['moving'] and not HOLD.held():
                robot['x'] += 0.02
            t = TransformStamped()
            t.header.stamp = rospy.Time.now()
            t.header.frame_id, t.child_frame_id = 'map', 'base_link'
            t.transform.translation.x = robot['x']
            t.transform.rotation.w = 1.0
            br.sendTransform(t)
            r.sleep()
    threading.Thread(target=tf_loop, daemon=True).start()

    held_pub = rospy.Publisher('/cmd_vel_mux/held', Bool, queue_size=5, latch=True)
    mower_pub = rospy.Publisher('/mower/status', Mower, queue_size=1)
    blade = []
    rospy.Subscriber('/mower/set_motor_on', Bool, lambda m: blade.append(m.data), queue_size=5)
    pubs = MowerPublishers()
    HOLD.start()

    def set_hold(on):
        held_pub.publish(Bool(on))
        t_end = time.monotonic() + 2.0
        while HOLD.held() != on and time.monotonic() < t_end:
            time.sleep(0.02)

    def path(n=300):
        p = Path()
        p.header.frame_id = 'map'
        for i in range(n):
            q = PoseStamped()
            q.header.frame_id = 'map'
            q.pose.position.x = i * 0.03
            q.pose.orientation.w = 1.0
            p.poses.append(q)
        return p

    def exe_ud():
        ud = smach.UserData()
        full = path()
        ud.path_chunk, ud.path, ud.paths = full, full, [full]
        ud.zone_name, ud.index_path, ud.final_pose = 'z', 0, None
        ud.path_window_start_index = 0
        ud.restore_height_pending, ud.zone_cut_height = False, 40
        ud.consecutive_nav_failures = 0
        return ud

    def in_thread(state, ud):
        box = {}

        def go():
            box['out'] = state.execute(ud)
            box['t'] = time.monotonic()
        th = threading.Thread(target=go, daemon=True)
        th.start()
        return box, th

    time.sleep(1.0)
    print('1 EXE_PLANNER_PATH in a hold')
    s = st.ExecutePathWithFeedback(pubs)
    box, th = in_thread(s, exe_ud())
    time.sleep(1.5)
    set_hold(True)
    time.sleep(1.0)
    ok('goal cancelled at once', server['cancels'] == 1, server)
    time.sleep(7.0)
    ok('8 s held (stall limit 5 s): still waiting', 'out' not in box, box)
    set_hold(False)
    th.join(3)
    ok("after the hold: 'replan_needed'", box.get('out') == 'replan_needed', box)

    print('2 STOP during the hold')
    s = st.ExecutePathWithFeedback(pubs)
    box, th = in_thread(s, exe_ud())
    time.sleep(1.0)
    set_hold(True)
    time.sleep(1.0)
    t = time.monotonic()
    s.request_preempt()
    th.join(3)
    ok('state ended within 1 s', 'out' in box and box['t'] - t < 1.0, box)
    set_hold(False)

    print('3 control: no hold, robot not moving')
    s = st.ExecutePathWithFeedback(pubs)
    robot['moving'] = False
    box, th = in_thread(s, exe_ud())
    th.join(15)
    robot['moving'] = True
    ok("real stall ends in 'aborted'", box.get('out') == 'aborted', box)

    print('4 wrap_tree')
    log = []

    class Leaf(smach.State):
        def __init__(self, name):
            smach.State.__init__(self, outcomes=['done', 'preempted'])
            self.name = name

        def execute(self, ud):
            log.append(self.name)
            return 'done'

    inner = smach.StateMachine(outcomes=['done', 'preempted'])
    with inner:
        smach.StateMachine.add('C', Leaf('C'), transitions={'done': 'done'})
    sm = smach.StateMachine(outcomes=['done', 'preempted'])
    with sm:
        smach.StateMachine.add('A', Leaf('A'), transitions={'done': 'IN'})
        smach.StateMachine.add('IN', inner, transitions={'done': 'done'})
    HOLD.wrap_tree(sm)
    set_hold(True)
    box, th = in_thread(sm, smach.UserData())
    time.sleep(1.0)
    ok('nothing starts while held', log == [], log)
    set_hold(False)
    th.join(3)
    ok('runs after it, nested too', log == ['A', 'C'] and box.get('out') == 'done', (log, box))
    del log[:]
    set_hold(True)
    box, th = in_thread(sm, smach.UserData())
    time.sleep(0.8)
    sm.request_preempt()
    th.join(3)
    set_hold(False)
    ok('preempt passes the gate, the state does not run',
       log == [] and box.get('out') == 'preempted', (log, box))

    print('5 WaitForRpmReached')
    rpm = {'v': 0}

    def mower_loop():
        r = rospy.Rate(10)
        while not rospy.is_shutdown():
            m = Mower()
            m.status, m.moto_rpm = ('RUN' if rpm['v'] else 'OFF'), rpm['v']
            mower_pub.publish(m)
            r.sleep()
    threading.Thread(target=mower_loop, daemon=True).start()
    s = st.WaitForRpmReached(pubs, timeout=2.0)
    ud = UD()
    ud.zone_rpm = 2000
    box, th = in_thread(s, ud)
    time.sleep(0.5)
    set_hold(True)
    time.sleep(4.0)
    ok('held past its 2 s: waiting', 'out' not in box, box)
    set_hold(False)
    rpm['v'] = 2000
    th.join(3)
    ok('reached', box.get('out') == 'reached', box)
    rpm['v'] = 0

    print('6 DETOUR')
    server.update(abort_after=0.5)
    s = st.DetourAroundObstacle(pubs)

    def det_ud():
        ud = smach.UserData()
        ud.path, ud.path_window_start_index = path(), 0
        ud.zone_name, ud.index_path = 'z', 0
        ud.program = UD()
        ud.program.last_result = 'x'
        ud.skipped_spans = []
        return ud

    box, th = in_thread(s, det_ud())
    th.join(12)
    ok("no hold: 'no_detour'", box.get('out') == 'no_detour', box)
    del blade[:]
    ud = det_ud()
    box, th = in_thread(s, ud)
    time.sleep(0.8)
    set_hold(True)
    time.sleep(4.0)
    ok('held: waits instead of giving up', 'out' not in box, box)
    set_hold(False)
    th.join(5)
    ok("after the hold: 'resumed', line not skipped", box.get('out') == 'resumed', box)
    ok('blade back on', blade and blade[-1] is True, blade)
    ok('no span recorded', ud.skipped_spans == [], ud.skipped_spans)
    server.update(abort_after=None)

    print('7 REMOW')
    s = st.RemowSkippedSpans(pubs)
    calls = []

    def attempt(span):
        calls.append(span)
        if len(calls) == 1:
            set_hold(True)
            threading.Timer(1.0, lambda: set_hold(False)).start()
            return 'fail'
        return 'ok'
    s._attempt_span = attempt
    ud = smach.UserData()
    ud.zone_name = 'z'
    ud.skipped_spans = [{'zone': 'z', 'path_idx': 0, 'poses': [1, 2], 'reason': 'detour'}]
    out = s.execute(ud)
    ok('span tried again after the hold', out == 'done' and len(calls) == 2, (out, calls))


if __name__ == '__main__':
    main()
