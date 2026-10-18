#!/usr/bin/env python3
"""Off-robot test: a preempt during ZONE_IT / PATH_IT never kills the mission
state machine (vitulus-field#54).

Uses the REAL smach containers and the REAL mower_unit_smach code against
fakes: no master, no init_node.

Covers
  * old wiring (loop_outcomes only): a preempt that the zone serviced inside
    and the zone ending 'continue' -> InvalidTransitionError (bug reproduced)
  * set_iterated_state: the same run ends 'preempted'; no preempt -> the
    iterator is exhausted 'succeeded'; break outcomes pass unchanged
  * the real build_mission_child_sm wires ZONE_IT and PATH_IT this way
  * real MISSION_CONCURRENCE: STOP whose preempt the zone swallowed ends
    'stop_preempt' (-> STOPPED) instead of an exception

Run:  python3 test_iterator_preempt.py   (needs the ROS python env: rospy,
      smach, smach_ros, mbf_msgs, vitulus_msgs ... importable; no ROS master)
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
from std_msgs.msg import Bool

rospy.rostime.set_rostime_initialized(True)


class FakeSubscriber(object):
    def __init__(self, *args, **kwargs):
        pass

    def unregister(self):
        pass


class FakeClient(object):
    """Action client / service proxy that is never called while building."""

    def __init__(self, *args, **kwargs):
        pass

    def wait_for_server(self, timeout=None):
        return True

    def wait_for_service(self, timeout=None):
        return True


rospy.Subscriber = FakeSubscriber
rospy.ServiceProxy = FakeClient
rospy.get_param = lambda name, default=None: default
import actionlib  # noqa: E402
import smach_ros.simple_action_state as sas  # noqa: E402
actionlib.SimpleActionClient = FakeClient
sas.SimpleActionClient = FakeClient

from mower_unit_smach import mission  # noqa: E402

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


class Step(smach.State):
    """One zone step: waits for go, services a preempt and goes on anyway.

    This is what happened in #54: the STOP preempt reached a state that
    serviced it and returned a normal outcome, the zone went on to
    WAIT_SET_MOTOR_OFF -> 'continue' while ZONE_IT kept its preempt flag.
    """

    def __init__(self, outcome='continue'):
        smach.State.__init__(self, outcomes=['continue', 'aborted', 'preempted'])
        self.outcome = outcome
        self.entered = threading.Event()
        self.go = threading.Event()

    def execute(self, ud):
        self.entered.set()
        self.go.wait(5.0)
        self.go.clear()
        if self.preempt_requested():
            self.service_preempt()
        return self.outcome


def zone_sm(step):
    zone = smach.StateMachine(outcomes=['continue', 'aborted', 'preempted'])
    with zone:
        smach.StateMachine.add('STEP', step,
                               transitions={'continue': 'continue',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
    return zone


def mission_sm(step, fixed):
    """MISSION_CHILD-like SM: ZONE_IT over 3 zones -> succeeded."""
    sm = smach.StateMachine(outcomes=['succeeded', 'aborted', 'rejected', 'preempted'])
    zone_it = smach.Iterator(
        outcomes=['succeeded', 'preempted', 'aborted'],
        input_keys=[], output_keys=[], it=lambda: range(3),
        it_label='index', exhausted_outcome='succeeded')
    with zone_it:
        if fixed:
            mission.set_iterated_state('PROCESS_ZONE', zone_sm(step),
                                       loop_outcomes=['continue'])
        else:
            smach.Iterator.set_contained_state('PROCESS_ZONE', zone_sm(step),
                                               loop_outcomes=['continue'])
    with sm:
        smach.StateMachine.add('ZONE_IT', zone_it,
                               transitions={'succeeded': 'succeeded',
                                            'aborted': 'aborted',
                                            'preempted': 'preempted'})
    return sm


def run(sm, step, preempt, zones=3):
    out = {}

    def target():
        try:
            out['o'] = sm.execute()
        except Exception as ex:  # smach raises on an invalid transition
            out['exc'] = ex
    t = threading.Thread(target=target)
    t.start()
    for i in range(zones):
        if not wait_until(step.entered.is_set):
            break
        step.entered.clear()
        if preempt and i == 0:
            sm.request_preempt()
        step.go.set()
    t.join(5.0)
    return out


# ---------------------------------------------------------------------------
# Iterator wiring
# ---------------------------------------------------------------------------
step = Step()
out = run(mission_sm(step, fixed=False), step, preempt=True)
check('old wiring: swallowed preempt -> InvalidTransitionError (bug reproduced)',
      'exc' in out and 'continue' in str(out.get('exc')), repr(out))

step = Step()
out = run(mission_sm(step, fixed=True), step, preempt=True)
check('set_iterated_state: swallowed preempt -> preempted',
      out.get('o') == 'preempted', repr(out))

step = Step()
out = run(mission_sm(step, fixed=True), step, preempt=False)
check('no preempt: all zones -> succeeded', out.get('o') == 'succeeded',
      repr(out))

step = Step(outcome='aborted')
out = run(mission_sm(step, fixed=True), step, preempt=False, zones=1)
check('break outcome passes unchanged', out.get('o') == 'aborted', repr(out))


# ---------------------------------------------------------------------------
# The real mission SM uses it for ZONE_IT and PATH_IT
# ---------------------------------------------------------------------------
class Pubs(object):
    def __getattr__(self, name):
        class P(object):
            def publish(self, msg):
                pass
        return P()


real = mission.build_mission_child_sm(Pubs())
zone_it = real['ZONE_IT']
path_it = zone_it['PROCESS_ZONE']['PATH_IT']
check('ZONE_IT maps continue -> preempted',
      zone_it._final_outcome_map == {'continue': 'preempted'},
      repr(zone_it._final_outcome_map))
check('PATH_IT maps its loop outcomes -> preempted',
      path_it._final_outcome_map == {'continue_path': 'preempted',
                                     'aborted_path': 'preempted'},
      repr(path_it._final_outcome_map))
for it in (zone_it, path_it):
    reg = set(it.get_registered_outcomes())
    check('%s: every final outcome is registered' % it._state_label,
          all(v in reg for v in it._final_outcome_map.values()), repr(reg))


# ---------------------------------------------------------------------------
# MISSION_CONCURRENCE: the #54 run ends in STOPPED, not in an exception
# ---------------------------------------------------------------------------
def build_cc(child_sm):
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
        cc = mission.build_mission_concurrence(Pubs(), parent,
                                               mission.UserStopLatch())
        return cc, parent
    finally:
        mission.build_mission_child_sm = orig


step = Step()
cc, parent = build_cc(mission_sm(step, fixed=True))
out = {}


def run_cc():
    try:
        out['o'] = cc.execute(parent.userdata)
    except Exception as ex:
        out['exc'] = ex


t = threading.Thread(target=run_cc)
t.start()
wait_until(step.entered.is_set)
step.entered.clear()
cc['STOP_MONITOR']._cb(Bool(True), None)   # first STOP of #54
check('STOP preempt reaches the zone step', wait_until(step.preempt_requested))
step.go.set()
t.join(5.0)
check('STOP swallowed in the zone -> stop_preempt (STOPPED)',
      out.get('o') == 'stop_preempt', repr(out))

print('')
if FAILS:
    print('FAILED: %d (%s)' % (len(FAILS), ', '.join(FAILS)))
    sys.exit(1)
print('ALL PASS')
sys.exit(0)
