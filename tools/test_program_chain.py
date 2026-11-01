#!/usr/bin/env python3
"""Offline test of a chain of programs (mower_unit_smach/chain.py).

No ROS master. The whole run against the real state machine is in vitulus_agent
test/sim/run_chain_sim.sh; here is what is awkward to provoke there.

Run:  python3 tools/test_program_chain.py
"""
import json
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))
sys.path.insert(0, HERE)
import _ros_env  # noqa: E402
_ros_env.ensure()

import rospy  # noqa: E402
from std_msgs.msg import String  # noqa: E402
from vitulus_msgs.msg import PlannerProgram, PlannerProgramList  # noqa: E402


class Pub(object):
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


class Pubs(object):
    def __init__(self):
        self.chain_state = Pub()


rospy.Subscriber = lambda *a, **kw: None
rospy.sleep = lambda d: None
rospy.is_shutdown = lambda: False
for name in ('loginfo', 'logwarn'):
    setattr(rospy, name, lambda *a, **kw: None)

from mower_unit_smach import chain as C  # noqa: E402

FAILED = []


def check(label, condition, detail=''):
    print('%-66s %s%s' % (label, 'ok' if condition else 'FAIL',
                          '' if condition else ' (%s)' % (detail,)))
    if not condition:
        FAILED.append(label)


def program(name, mark=''):
    p = PlannerProgram(name=name, last_result='failed: on_path-Z-1-0')
    p.header.frame_id = mark
    return p


def chain_msg(mark, *names):
    msg = PlannerProgramList(program_list=[program(n) for n in names])
    msg.header.frame_id = mark
    return msg


def fresh():
    c = C.ProgramChain()
    c._pubs = Pubs()
    return c


def state(c):
    return json.loads(c._pubs.chain_state.sent[-1].data)


C.ARRIVE_S = 0.05

# a plain Run never picks a chain up, not even a pending one of the same name
c = fresh()
c._on_chain(chain_msg('chain:1', 'A', 'B'))
c.begin(program('A'))
check('plain Run: no next program', not c.has_next('A'))
check('plain Run: nothing reported', c._pubs.chain_state.sent == [])

# the marked first program takes its list, once
c = fresh()
c._on_chain(chain_msg('chain:1', 'A', 'B', 'C'))
c.begin(program('A', 'chain:1'))
check('chain: running', state(c)['state'] == 'running', state(c))
check('chain: A has a next one', c.has_next('A'))
check('chain: another name has not', not c.has_next('B'))
check('chain: not midway in the first', not c.midway())
b = c.advance('A')
check('chain: next is B', b.name == 'B')
check('chain: next starts from scratch', b.last_result == 'failed: on_init', b.last_result)
check('chain: midway', c.midway())
check('chain: what is done and what remains',
      state(c)['done'] == ['A'] and state(c)['remaining'] == ['B', 'C'], state(c))
check('chain: advance from a wrong name gives nothing', c.advance('A') is None)
c.interrupt('stop')
check('chain: interrupted keeps the undone one in remaining',
      state(c)['state'] == 'interrupted' and state(c)['remaining'] == ['B', 'C']
      and state(c)['reason'] == 'stop', state(c))
check('chain: interrupted has no next', not c.has_next('B'))
check('chain: interrupted is not midway', not c.midway())
c.begin(program('A', 'chain:1'))
check('chain: a list is never used twice', state(c)['state'] == 'lost', state(c))
c.begin(program('A'))
check('next plain Run clears it', state(c)['state'] == 'idle', state(c))

# the board at the next program (mission #279: set_power true = MCU reset)
c = fresh()
c._on_chain(chain_msg('chain:8', 'A', 'B'))
c.begin(program('A', 'chain:8'))
check('board: first program powers on as ever', not c.board_stays_on('READY'))
c.advance('A')
check('board: next program leaves a powered board alone', c.board_stays_on('READY'))
check('board: a busy board too, no reset of an error',
      c.board_stays_on('CHANGE_HEIGHT') and c.board_stays_on('ERR'))
check('board: off is switched on', not c.board_stays_on('OFF'))
check('board: silent is switched on as ever', not c.board_stays_on(None))
c.interrupt('stop')
check('board: outside a running chain as ever', not c.board_stays_on('READY'))

# to the end
c = fresh()
c._on_chain(chain_msg('chain:2', 'A', 'B'))
c.begin(program('A', 'chain:2'))
c.advance('A')
check('last program has no next', not c.has_next('B'))
c.finish()
check('done', state(c)['state'] == 'done' and state(c)['remaining'] == []
      and state(c)['done'] == ['A', 'B'], state(c))
c.interrupt('stop')
check('interrupt after the end changes nothing', state(c)['state'] == 'done', state(c))

# the mark does not fit the list
c = fresh()
c._on_chain(chain_msg('chain:3', 'A', 'B'))
c.begin(program('A', 'chain:4'))
check('other id: lost, docks after it', state(c)['state'] == 'lost' and not c.has_next('A'),
      state(c))
c = fresh()
c._on_chain(chain_msg('chain:5', 'B', 'A'))
c.begin(program('A', 'chain:5'))
check('list of another first program: lost', state(c)['state'] == 'lost', state(c))
c = fresh()
c.begin(program('A', 'chain:6'))
check('list never came: lost', state(c)['state'] == 'lost', state(c))

# the battery at a program boundary: the real state against scripted readings
from mower_unit_smach import states as ST  # noqa: E402
from vitulus_msgs.msg import Power_status  # noqa: E402


class FakeAny(object):
    def publish(self, msg):
        pass


class FakePubs(object):
    def __getattr__(self, name):
        return FakeAny()


class UD(object):
    def __init__(self):
        self.program = program('A')
        self.prg_start_time = None


def boundary(reading, preempt_at=None):
    """(outcome, chain state, waits) of one NEXT_PROGRAM."""
    c = fresh()
    c._on_chain(chain_msg('chain:7', 'A', 'B'))
    c.begin(program('A', 'chain:7'))
    st = ST.NextProgramState(FakePubs(), c)
    calls = []

    def wait(topic, cls, timeout=None):
        calls.append(topic)
        if preempt_at == len(calls):
            st.request_preempt()
        value = reading(len(calls))
        if value is None:
            raise rospy.ROSException('timeout')
        return Power_status(battery_capacity=value)
    ST.rospy.wait_for_message = wait
    ST.rospy.get_param = lambda name, default=None: default
    ST.rospy.Time.now = staticmethod(lambda: rospy.Time(100, 0))
    ST.publish_mission_rejected = lambda *a: rejected.append(a[3])
    del rejected[:]
    return st.execute(UD()), state(c), len(calls)


rejected = []
out, st_, _n = boundary(lambda i: 90)
check('battery 90: next program', out == 'next' and st_['current'] == 'B', (out, st_))
out, st_, _n = boundary(lambda i: 40)
check('battery at the limit: next program', out == 'next', out)
out, st_, _n = boundary(lambda i: 30)
check('battery 30: refused, going home', out == 'rejected' and rejected == ['battery'],
      (out, rejected))
check('battery 30: A done, B remains, chain still running for the refusal',
      st_['done'] == ['A'] and st_['remaining'] == ['B'] and st_['state'] == 'running', st_)
out, st_, n = boundary(lambda i: None)
check('battery unknown: as at a start, it goes on', out == 'next' and n == 5, (out, n))
out, st_, n = boundary(lambda i: None, preempt_at=2)
check('STOP while waiting for the battery wins', out == 'preempted' and n == 2, (out, n))
out, st_, _n = boundary(lambda i: 30, preempt_at=1)
check('STOP together with a low battery wins', out == 'preempted' and not rejected,
      (out, rejected))
check('STOP at the boundary: A stays done, B remains (mission #279)',
      st_['done'] == ['A'] and st_['remaining'] == ['B'], st_)

# STOPPED / TERMINAL_ERROR hear a Reset on one subscriber for the whole stay
from mower_unit_smach import top_level as T  # noqa: E402
from std_msgs.msg import Bool  # noqa: E402


class Sub(object):
    made = 0
    gone = 0

    def __init__(self, *a, **kw):
        Sub.made += 1

    def unregister(self):
        Sub.gone += 1


T.rospy.Subscriber = Sub
reset = T.ResetListener()
reset.start()
check('reset: nothing yet', not reset.requested())
reset._cb(Bool(False))
check('reset: False is no Reset', not reset.requested())
reset._cb(Bool(True))
check('reset: heard', reset.requested())
check('reset: still heard on the next look', reset.requested())
check('reset: one subscriber however long it waits', Sub.made == 1, Sub.made)
reset.stop()
check('reset: released', Sub.gone == 1, Sub.gone)
reset.start()
check('reset: one from the stay before does not count', not reset.requested())
reset.stop()

print('\nFAILED: %s' % FAILED if FAILED else '\nall ok')
sys.exit(1 if FAILED else 0)
