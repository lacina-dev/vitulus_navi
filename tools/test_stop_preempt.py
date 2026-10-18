#!/usr/bin/env python3
"""Off-robot test: a user STOP always ends the mission (vitulus-field#53).

Uses the REAL smach / smach_ros containers and the REAL mower_unit_smach code
against fakes: no master, no init_node, no action server.

Covers
  * classify_mbf_result: PREEMPTED -> None (smach_ros decides), others as before
  * a real SimpleActionState whose MBF goal is cancelled by a container
    preempt ends 'preempted' (was 'aborted' -> RECOVERY, preempt lost);
    the old classifier still reproduces the lost preempt
  * real MISSION_CONCURRENCE (build_mission_concurrence, mission child faked):
    - STOP_MONITOR fires but the child loses the preempt -> mission keeps
      running; the next STOP via UserStopLatch ends it in 'stop_preempt'
    - STOP via the latch before STOP_MONITOR -> 'stop_preempt', never the
      generic 'preempted' (-> TERMINAL_ERROR)
    - the latch is reset at every mission start; request() without a
      running mission does nothing

Run:  python3 test_stop_preempt.py   (needs the ROS python env: rospy, smach,
      smach_ros, mbf_msgs, vitulus_msgs ... importable; no ROS master)
"""
import os
import sys
import threading
import time

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))

import rospy
import rospy.rostime
import smach
import smach_ros
import smach_ros.simple_action_state as sas
import actionlib
from std_msgs.msg import Bool
from mbf_msgs.msg import ExePathAction, ExePathGoal, ExePathResult

# Wall clock instead of /clock, no master.
rospy.rostime.set_rostime_initialized(True)


class FakeSubscriber(object):
    def __init__(self, *args, **kwargs):
        pass

    def unregister(self):
        pass


rospy.Subscriber = FakeSubscriber

from mower_unit_smach import helpers, mission  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print('%s %s %s' % ('PASS' if cond else 'FAIL', name, detail))
    if not cond:
        FAILS.append(name)


def wait_until(pred, timeout=5.0):
    end = time.time() + timeout
    while time.time() < end:
        if pred():
            return True
        time.sleep(0.02)
    return pred()


class Result(object):
    def __init__(self, outcome, message=''):
        self.outcome = outcome
        self.message = message


# ---------------------------------------------------------------------------
# classify_mbf_result
# ---------------------------------------------------------------------------
GS = actionlib.GoalStatus
check('classify succeeded',
      helpers.classify_mbf_result('X', GS.SUCCEEDED, Result(0)) == 'succeeded')
check('classify succeeded status, failed outcome',
      helpers.classify_mbf_result('X', GS.SUCCEEDED, Result(50)) == 'aborted')
check('classify aborted',
      helpers.classify_mbf_result('X', GS.ABORTED, Result(50)) == 'aborted')
check('classify no result',
      helpers.classify_mbf_result('X', GS.ABORTED, None) == 'aborted')
check('classify preempted defers to smach_ros',
      helpers.classify_mbf_result(
          'X', GS.PREEMPTED, Result(101, 'Controller canceled')) is None)


# ---------------------------------------------------------------------------
# Real SimpleActionState + fake MBF action client
# ---------------------------------------------------------------------------
class FakeClient(object):
    """MBF exe_path: runs until cancelled, then ends PREEMPTED (outcome 101)."""

    def __init__(self, *args, **kwargs):
        self.done_cb = None
        self.state = GS.PENDING

    def wait_for_server(self, timeout=None):
        return True

    def send_goal(self, goal, done_cb=None, active_cb=None, feedback_cb=None):
        self.done_cb = done_cb
        self.state = GS.ACTIVE

    def cancel_goal(self):
        def finish():
            time.sleep(0.05)
            self.state = GS.PREEMPTED
            r = ExePathResult()
            r.outcome = 101
            r.message = 'Controller canceled'
            self.done_cb(GS.PREEMPTED, r)
        threading.Thread(target=finish).start()

    def get_state(self):
        return self.state


sas.SimpleActionClient = FakeClient


def legacy_classify(action_name, status, result, log_pub=None):
    if status == GS.SUCCEEDED and getattr(result, 'outcome', None) == 0:
        return 'succeeded'
    return 'aborted'


def run_exe_path(classifier):
    """EXE_PATH_TO_START-like SM; returns its outcome after a preempt."""
    sm = smach.StateMachine(outcomes=['succeeded', 'recovery', 'preempted'])
    with sm:
        smach.StateMachine.add(
            'EXE', smach_ros.SimpleActionState(
                '/move_base_flex/exe_path', ExePathAction,
                goal=ExePathGoal(),
                result_cb=lambda ud, s, r: classifier('EXE', s, r)),
            transitions={'succeeded': 'succeeded',
                         'aborted': 'RECOVERY',
                         'preempted': 'preempted'})
        smach.StateMachine.add(
            'RECOVERY', smach.CBState(lambda ud: 'done', outcomes=['done']),
            transitions={'done': 'recovery'})
    exe = sm['EXE']
    out = {}
    t = threading.Thread(target=lambda: out.update(o=sm.execute()))
    t.start()
    wait_until(lambda: exe._action_client.done_cb is not None)
    sm.request_preempt()
    t.join(5.0)
    return out.get('o'), sm.preempt_requested()


outcome, still = run_exe_path(legacy_classify)
check('old classifier loses the preempt (bug reproduced)',
      outcome == 'recovery' and not still, 'outcome=%s' % outcome)
outcome, _ = run_exe_path(helpers.classify_mbf_result)
check('cancelled exe_path ends preempted', outcome == 'preempted',
      'outcome=%s' % outcome)


# ---------------------------------------------------------------------------
# MISSION_CONCURRENCE with a fake mission child
# ---------------------------------------------------------------------------
class WaitPreempt(smach.State):
    """Runs until preempted; drop=True mimics the lost preempt of #53."""

    def __init__(self, drop=False):
        smach.State.__init__(self, outcomes=['aborted', 'preempted'])
        self.drop = drop
        self.entered = threading.Event()

    def execute(self, ud):
        self.entered.set()
        while not self.preempt_requested():
            time.sleep(0.01)
        self.service_preempt()
        return 'aborted' if self.drop else 'preempted'


class Pubs(object):
    def __init__(self):
        self.sent = []

    def __getattr__(self, name):
        pubs = self

        class P(object):
            def publish(self, msg):
                pubs.sent.append((name, getattr(msg, 'data', msg)))
        return P()


def build(child_sm):
    orig = mission.build_mission_child_sm
    mission.build_mission_child_sm = lambda pubs: child_sm
    try:
        parent = smach.StateMachine(outcomes=['x'])
        for k in ['program', 'prg_start_time', 'unfinished_active',
                  'unfinished_zone', 'unfinished_path', 'unfinished_window',
                  'path_window_start_index', 'path_chunk', 'error_reason',
                  'restore_height_pending', 'consecutive_nav_failures',
                  'skipped_spans']:
            parent.userdata[k] = None
        latch = mission.UserStopLatch()
        cc = mission.build_mission_concurrence(Pubs(), parent, latch)
        return cc, latch, parent
    finally:
        mission.build_mission_child_sm = orig


def start(cc, parent):
    out = {}
    t = threading.Thread(target=lambda: out.update(o=cc.execute(parent.userdata)))
    t.start()
    return t, out


def stop_monitor_fire(cc):
    mon = cc['STOP_MONITOR']
    mon._cb(Bool(True), None)


# 1) STOP_MONITOR fires, the child loses the preempt; the next STOP ends it.
child = smach.StateMachine(outcomes=['succeeded', 'aborted', 'rejected', 'preempted'])
with child:
    smach.StateMachine.add('EXE', WaitPreempt(drop=True),
                           transitions={'aborted': 'RECOVERY_THEN_MOW',
                                        'preempted': 'preempted'})
    smach.StateMachine.add('RECOVERY_THEN_MOW', WaitPreempt(),
                           transitions={'aborted': 'aborted',
                                        'preempted': 'preempted'})
cc, latch, parent = build(child)
check('latch idle before mission', latch.request() is False)
t, out = start(cc, parent)
wait_until(lambda: child['EXE'].entered.is_set())
stop_monitor_fire(cc)
wait_until(lambda: child['RECOVERY_THEN_MOW'].entered.is_set())
time.sleep(0.3)
check('first STOP lost -> mission still running (old symptom)', t.is_alive())
check('second STOP re-preempts the mission', latch.request() is True)
t.join(5.0)
check('mission ends after second STOP', not t.is_alive())
check('outcome stop_preempt -> STOPPED', out.get('o') == 'stop_preempt',
      'outcome=%s' % out.get('o'))

# 2) Global STOP handler first: STOP_MONITOR preempted, still STOPPED.
child2 = smach.StateMachine(outcomes=['succeeded', 'aborted', 'rejected', 'preempted'])
with child2:
    smach.StateMachine.add('MOW', WaitPreempt(),
                           transitions={'aborted': 'aborted',
                                        'preempted': 'preempted'})
cc2, latch2, parent2 = build(child2)
t, out = start(cc2, parent2)
wait_until(lambda: child2['MOW'].entered.is_set())
check('STOP via latch accepted', latch2.request() is True)
t.join(5.0)
check('latch-only STOP -> stop_preempt (not TERMINAL_ERROR)',
      out.get('o') == 'stop_preempt', 'outcome=%s' % out.get('o'))
check('latch idle after mission', latch2.request() is False)

# 3) The latch is reset at the next mission start.
check('latch still set from the stopped mission', latch2.requested())
child2['MOW'].entered.clear()
t, out = start(cc2, parent2)
wait_until(lambda: child2['MOW'].entered.is_set())
check('latch reset at mission start', not latch2.requested())
cc2['MISSION_CHILD'].request_preempt()   # e.g. battery monitor
t.join(5.0)
check('no stale STOP on the next mission', out.get('o') == 'preempted',
      'outcome=%s' % out.get('o'))

print('')
if FAILS:
    print('FAILED: %d (%s)' % (len(FAILS), ', '.join(FAILS)))
    sys.exit(1)
print('ALL PASS')
sys.exit(0)
