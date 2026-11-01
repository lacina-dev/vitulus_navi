"""Several programs, one run (owner 2026-10-31, mission #277).

"Mow zone A, then Test": without this the program docked after A, the next one
asked for a new unlock, undocked again and waited for RTK. A chain runs as one
job: after a program the blade is already off, the next program starts right
there through the mission's own off-the-dock path, and only the last one docks.

The planner sends the WHOLE chain before the first program (latched
/web_plan/program_chain, PlannerProgramList, every program with its current
zones) and marks that first program with the same id in header.frame_id. So
  * a Run without the mark (UI, calendar) can never pick a chain up;
  * nothing is asked at a program boundary: the next program is here already.

Programs stay what they are: each one has its own last_result, its own
checkpoint and its own /mower_smach/active_program. A chain is never saved.

/mower_smach/chain (latched JSON) says how far it got:
    {id, state, programs, index, current, done, remaining, reason, t}
state: 'running' | 'done' | 'interrupted' | 'lost' (the first program came, its
chain did not: the robot docks after it) | 'idle'.
remaining = what has not been finished, the interrupted program included.
"""
import copy
import json
import threading
import time

import rospy
from std_msgs.msg import String
from vitulus_msgs.msg import PlannerProgramList

CHAIN_TOPIC = '/web_plan/program_chain'
MARK = 'chain:'
ARRIVE_S = 3.0      # the list is published before the program, see begin()


class ProgramChain(object):

    def __init__(self):
        self._lock = threading.Lock()
        self._sub = None
        self._pubs = None
        self._on_program = None
        self._pending = None    # last PlannerProgramList from the planner
        self._id = None
        self._programs = []
        self._index = 0
        self._state = 'idle'
        self._reason = None

    def attach(self, pubs, on_program=None):
        """`on_program(program)` hears each program taken over by advance()."""
        self._pubs = pubs
        self._on_program = on_program
        if self._sub is None:
            self._sub = rospy.Subscriber(CHAIN_TOPIC, PlannerProgramList,
                                         self._on_chain, queue_size=1)

    def _on_chain(self, msg):
        with self._lock:
            self._pending = msg

    def _take(self, mark):
        """The list for this start, once. A chain is never used twice."""
        with self._lock:
            msg = self._pending
            if msg is None or msg.header.frame_id != mark:
                return None
            self._pending = None
            return msg

    def begin(self, program):
        """A mission starts. Only a program the planner marked starts a chain."""
        mark = getattr(getattr(program, 'header', None), 'frame_id', '') or ''
        if not mark.startswith(MARK):
            if self._state != 'idle':
                self._set([], None, 0, 'idle', None)
            return
        msg = self._take(mark)
        deadline = time.monotonic() + ARRIVE_S
        while msg is None and time.monotonic() < deadline and not rospy.is_shutdown():
            rospy.sleep(0.05)       # another connection than the program's
            msg = self._take(mark)
        if msg is None or not msg.program_list \
                or msg.program_list[0].name != program.name:
            rospy.logwarn('[CHAIN] %s: no chain list for %s, it will dock after it',
                          mark, program.name)
            self._set([program], mark, 0, 'lost', 'no_chain_list')
            return
        self._set(list(msg.program_list), mark, 0, 'running', None)
        rospy.loginfo('[CHAIN] %s: %s', mark,
                      ' -> '.join(p.name for p in msg.program_list))

    def has_next(self, name):
        """This program is running in a chain and it is not the last one."""
        with self._lock:
            return (self._state == 'running' and self._index + 1 < len(self._programs)
                    and self._programs[self._index].name == name)

    def advance(self, name):
        """The next program to run after `name`, None when there is none."""
        if not self.has_next(name):
            return None
        with self._lock:
            self._index += 1
            program = copy.deepcopy(self._programs[self._index])
        program.last_result = 'failed: on_init'
        if self._on_program is not None:
            self._on_program(program)
        self._report()
        return program

    def leave_next(self, name):
        """`name` is done, what follows does not start: the rest stays as it is."""
        if self.has_next(name):
            with self._lock:
                self._index += 1
            self._report()

    def midway(self):
        """A program after the first one runs: the robot is out, working."""
        with self._lock:
            return self._state == 'running' and self._index > 0

    def board_stays_on(self, status):
        """The mower board is left alone at the start of a next program.

        /mower/set_power true at a powered board is an MCU reset, the firmware
        does that on purpose (set_power_callback, pm_batt_switch_callback). The
        board goes silent for 14 s with bat_out_switch on all the time, and the
        supervisor's grace needs bat_out going False->True: mission #279 faulted
        here. `status` is the last /mower/status, None when silent: a board that
        reports OFF, or nothing, is switched on the usual way.
        """
        return self.midway() and status not in (None, 'OFF')

    def finish(self):
        """The last program is done."""
        if self._state == 'running':
            with self._lock:
                self._state = 'done'
                self._index = len(self._programs)
            self._report()

    def interrupt(self, reason):
        """Stop, rain, battery, an error: the rest stays undone."""
        if self._state == 'running':
            with self._lock:
                self._state = 'interrupted'
                self._reason = reason
            self._report()

    def _set(self, programs, chain_id, index, state, reason):
        with self._lock:
            self._programs = programs
            self._id = chain_id
            self._index = index
            self._state = state
            self._reason = reason
        self._report()

    def report_dict(self):
        with self._lock:
            names = [p.name for p in self._programs]
            current = names[self._index] if self._index < len(names) else None
            return {'id': self._id, 'state': self._state, 'programs': names,
                    'index': self._index, 'current': current,
                    'done': names[:self._index], 'remaining': names[self._index:],
                    'reason': self._reason, 't': round(time.time(), 1)}

    def _report(self):
        if self._pubs is not None:
            self._pubs.chain_state.publish(String(json.dumps(self.report_dict())))


PROGRAM_CHAIN = ProgramChain()
