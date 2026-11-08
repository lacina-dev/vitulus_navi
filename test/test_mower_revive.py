#!/usr/bin/env python3
"""T24: a mower board that is silent when the mission switches it on, offline.

    python3 test/test_mower_revive.py      (sourced workspace)

The 8. 11. test run: no /mower/status after the undock, /mower/set_power True went
over the dead link, WAIT_FOR_MOWER_ON timed out, critical with reason UNKNOWN.
"""
import os
import sys
import threading
import time
import types
import unittest

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:1'     # never the robot's master

import rospy  # noqa: E402
from vitulus_msgs.msg import Mower  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'src')
sys.path.insert(0, SRC)
from mower_unit_smach import mower_revive as mr  # noqa: E402
sys.path.remove(SRC)

rospy.rostime.set_rostime_initialized(True)


# ---------------------------------------------------------------------------
# revive(): the PM sequence on a simulated clock
# ---------------------------------------------------------------------------
class SimIO(object):
    """PM read-back follows a command after `pm_lag`; the board answers `boot_s`
    after bat_out on (None: never)."""

    def __init__(self, bat=False, boot_s=None, pm_lag=0.1, pm_deaf=False,
                 preempt_at=None):
        self.t = 0.0
        self.bat = bat
        self.cmds = []
        self.boot_s = boot_s
        self.pm_lag = pm_lag
        self.pm_deaf = pm_deaf
        self.preempt_at = preempt_at
        self.on_t = None
        self.pending = None
        self.last_status_t = None

    def now(self):
        return self.t

    def sleep(self, dt):
        if self.preempt_at is not None and self.t >= self.preempt_at:
            return False
        self.t += dt
        if self.pending is not None and self.t >= self.pending[0]:
            self.bat = self.pending[1]
            if self.bat and self.on_t is None:
                self.on_t = self.t
            self.pending = None
        if (self.boot_s is not None and self.on_t is not None
                and self.t - self.on_t >= self.boot_s):
            self.last_status_t = self.t
        return True

    def status_t(self):
        return self.last_status_t

    def bat_out(self):
        return self.bat

    def set_bat(self, value):
        self.cmds.append((round(self.t, 1), value))
        if not self.pm_deaf:
            self.pending = (self.t + self.pm_lag, value)


class Revive(unittest.TestCase):
    def test_dead_link_rail_off_switches_on_and_the_board_answers(self):
        io = SimIO(bat=False, boot_s=5.0)        # 8. 11.: 5 s after bat_out on
        self.assertEqual(mr.revive(io), 'alive')
        self.assertEqual([v for _, v in io.cmds], [True])
        self.assertLess(io.t, 6.0)

    def test_rail_on_is_cycled_off_first(self):
        io = SimIO(bat=True, boot_s=3.0)
        self.assertEqual(mr.revive(io), 'alive')
        self.assertEqual([v for _, v in io.cmds], [False, True])
        self.assertGreaterEqual(io.cmds[1][0], mr.OFF_S - 0.01)

    def test_unknown_rail_is_cycled_too(self):
        io = SimIO(bat=None, boot_s=3.0)
        self.assertEqual(mr.revive(io), 'alive')
        self.assertEqual([v for _, v in io.cmds][:2], [False, True])

    def test_never_answers_rail_goes_off_again(self):
        io = SimIO(bat=False, boot_s=None)
        self.assertEqual(mr.revive(io), 'silent')
        self.assertEqual([v for _, v in io.cmds], [True, False])
        self.assertGreaterEqual(io.t, mr.REVIVE_S - 0.01)
        self.assertLess(io.t, mr.REVIVE_S + 1.0)

    def test_stop_in_the_wait_ends_it_with_the_rail_off(self):
        io = SimIO(bat=False, boot_s=None, preempt_at=4.0)
        self.assertEqual(mr.revive(io), 'preempted')
        self.assertEqual([v for _, v in io.cmds], [True, False])
        self.assertLess(io.t, 4.5)

    def test_deaf_pm_gets_it_again_bounded(self):
        io = SimIO(bat=False, boot_s=None, pm_deaf=True)
        self.assertEqual(mr.revive(io), 'silent')
        ons = [v for _, v in io.cmds if v]
        self.assertEqual(len(ons), mr.RESEND_MAX)
        self.assertFalse(io.cmds[-1][1])

    def test_status_during_the_off_phase_is_enough(self):
        io = SimIO(bat=True, boot_s=None)
        io.last_status_t = None
        orig = io.sleep

        def sleep(dt):
            ok = orig(dt)
            if io.t >= 0.5:
                io.last_status_t = io.t
            return ok
        io.sleep = sleep
        self.assertEqual(mr.revive(io), 'alive')
        self.assertEqual([v for _, v in io.cmds], [False])


class MaySwitch(unittest.TestCase):
    def test_only_a_board_without_blade(self):
        for status in ('READY', 'HOME', 'CHANGE_HEIGHT'):
            self.assertTrue(mr.may_switch(status, None, False), status)
        self.assertTrue(mr.may_switch('READY', False, False))

    def test_off_and_init_never_switch(self):
        # The firmware reports OFF and INIT over a running motor as well.
        for status in ('OFF', 'INIT'):
            for commanded in (None, False):
                self.assertFalse(mr.may_switch(status, commanded, False), status)
            self.assertNotIn(status, mr.SAFE)

    def test_a_blade_that_may_turn_keeps_the_old_way(self):
        self.assertFalse(mr.may_switch('RUN', None, False))
        for status in ('ERR', 'BLOCKED', 'TEMP', None, ''):
            self.assertFalse(mr.may_switch(status, None, False), status)
        self.assertFalse(mr.may_switch('READY', True, False))   # a True of the mission
        self.assertFalse(mr.may_switch('READY', None, True))    # mower_link armed


# ---------------------------------------------------------------------------
# The states, with a fake board, PM and watch (real rospy.sleep, short budgets)
# ---------------------------------------------------------------------------
class Sink(object):
    def __init__(self, hook=None):
        self.sent = []
        self.hook = hook

    def publish(self, msg):
        self.sent.append(msg)
        if self.hook is not None:
            self.hook(msg)


class Board(object):
    """The mower MCU (silent or not) and the PM's bat_out, behind a watch."""

    def __init__(self, alive=True, status='OFF', wakes_on_bat=None, wake_s=0.3,
                 ready_s=0.3, bat=False):
        self.clock = time.monotonic
        self.lock = threading.Lock()
        self.alive = alive
        self.status = status
        self.wakes_on_bat = wakes_on_bat     # silent board: bat_out on wakes it?
        self.wake_s = wake_s
        self.ready_s = ready_s
        self.started = self.clock() - 60.0
        self.last_status = (self.clock() - (0.0 if alive else 60.0), self._msg())
        self.last_bat_out = (self.clock(), bat)
        self.set_power = []
        self.bat_cmds = []
        self.stop = threading.Event()
        threading.Thread(target=self._run, daemon=True).start()

    def _msg(self):
        m = Mower()
        m.status = self.status
        return m

    def _run(self):
        while not self.stop.is_set():
            with self.lock:
                self.last_bat_out = (self.clock(), self.last_bat_out[1])
                if self.alive:
                    self.last_status = (self.clock(), self._msg())
            time.sleep(0.05)

    def on_bat(self, msg):
        self.bat_cmds.append(msg.data)
        with self.lock:
            self.last_bat_out = (self.clock(), msg.data)
        if msg.data and not self.alive and self.wakes_on_bat:
            threading.Timer(self.wake_s, self._wake).start()

    def _wake(self):
        with self.lock:
            self.alive = True

    def on_power(self, msg):
        self.set_power.append(msg.data)
        if msg.data and self.alive:
            threading.Timer(self.ready_s, self._ready).start()

    def _ready(self):
        with self.lock:
            self.status = 'READY'

    # MowerPowerWatch API
    def silent_s(self):
        with self.lock:
            return self.clock() - self.last_status[0]

    def fresh(self):
        with self.lock:
            return self.alive and self.clock() - self.last_status[0] < 1.0

    def status_now(self, timeout=2.5):
        end = self.clock() + min(timeout, 0.4)
        while self.clock() < end:
            if self.fresh():
                return self.status
            time.sleep(0.05)
        return None

    def wait_for_message(self, topic, cls, timeout=1.0):
        if self.fresh():
            time.sleep(0.05)
            return self._msg()
        time.sleep(min(timeout, 0.1))
        raise rospy.ROSException('timeout')


def make_pubs(board):
    return types.SimpleNamespace(
        mower_set_power=Sink(board.on_power), log_info=Sink(), smach_status=Sink(),
        pm_play_melody=Sink(), stop_reason=Sink(), save_program=Sink())


class Patched(unittest.TestCase):
    """The module globals the states read, pointed at the fake board."""

    def setUp(self):
        self.saved = {}

    def patch(self, mod, name, value):
        self.saved.setdefault((mod, name), getattr(mod, name))
        setattr(mod, name, value)

    def tearDown(self):
        for (mod, name), value in self.saved.items():
            setattr(mod, name, value)
        if getattr(self, 'board', None) is not None:
            self.board.stop.set()

    def use(self, board, link=None):
        self.board = board
        self.patch(mr, 'power_watch', lambda: board)
        self.patch(mr, 'mower_status_now', board.status_now)
        self.patch(mr, 'link_state', lambda: link or (False, None))
        self.patch(mr.rospy, 'sleep', lambda d: time.sleep(min(d, 0.05)))

    def state(self, pubs, **kw):
        st = mr.PowerOnMower(pubs, revive_s=kw.get('revive_s', 1.0), off_s=0.2)
        st.pub_bat = Sink(self.board.on_bat)
        return st


def program():
    return types.SimpleNamespace(name='P', last_result='')


class PowerOn(Patched):
    def test_live_link_is_switched_on_as_before(self):
        board = Board(alive=True, status='OFF')
        self.use(board)
        pubs = make_pubs(board)
        ud = types.SimpleNamespace(program=program())
        self.assertEqual(self.state(pubs).execute(ud), 'done')
        self.assertEqual(board.set_power, [True])
        self.assertEqual(board.bat_cmds, [])            # the PM is not touched
        self.assertEqual(ud.program.last_result, 'failed: on_mower')
        self.assertEqual([m.data for m in pubs.log_info.sent], ['Turn on mower'])

    def test_dead_link_pm_wakes_it_then_set_power(self):
        board = Board(alive=False, status='READY', wakes_on_bat=True)
        self.use(board)
        pubs = make_pubs(board)
        out = self.state(pubs).execute(types.SimpleNamespace(program=program()))
        self.assertEqual(out, 'done')
        self.assertEqual(board.bat_cmds, [True])
        self.assertEqual(board.set_power, [True])

    def test_dead_link_pm_brings_nothing(self):
        board = Board(alive=False, status='READY', wakes_on_bat=False)
        self.use(board)
        pubs = make_pubs(board)
        out = self.state(pubs).execute(types.SimpleNamespace(program=program()))
        self.assertEqual(out, 'silent')
        self.assertEqual(board.bat_cmds[0], True)
        self.assertFalse(board.bat_cmds[-1])            # the rail is off again
        self.assertEqual(board.set_power, [])           # nothing into the dead link

    def test_dead_link_after_a_blade_true_keeps_the_old_way(self):
        board = Board(alive=False, status='READY', wakes_on_bat=True)
        self.use(board, link=(True, True))
        pubs = make_pubs(board)
        out = self.state(pubs).execute(types.SimpleNamespace(program=program()))
        self.assertEqual(out, 'done')
        self.assertEqual(board.bat_cmds, [])
        self.assertEqual(board.set_power, [True])

    def test_dead_link_last_seen_running_keeps_the_old_way(self):
        board = Board(alive=False, status='RUN', wakes_on_bat=True)
        self.use(board)
        pubs = make_pubs(board)
        out = self.state(pubs).execute(types.SimpleNamespace(program=program()))
        self.assertEqual(out, 'done')
        self.assertEqual(board.bat_cmds, [])

    def test_dead_link_last_seen_off_or_init_never_touches_bat_out(self):
        for status in ('OFF', 'INIT'):
            board = Board(alive=False, status=status, wakes_on_bat=True)
            self.use(board)
            pubs = make_pubs(board)
            out = self.state(pubs).execute(types.SimpleNamespace(program=program()))
            self.assertEqual(out, 'done', status)
            self.assertEqual(board.bat_cmds, [], status)
            self.assertEqual(board.set_power, [True], status)   # the old way
            board.stop.set()

    def test_stop_in_the_wait(self):
        board = Board(alive=False, status='READY', wakes_on_bat=False)
        self.use(board)
        pubs = make_pubs(board)
        st = self.state(pubs, revive_s=5.0)
        threading.Timer(0.8, st.request_preempt).start()
        t0 = time.monotonic()
        out = st.execute(types.SimpleNamespace(program=program()))
        self.assertEqual(out, 'preempted')
        self.assertLess(time.monotonic() - t0, 2.5)
        self.assertFalse(board.bat_cmds[-1])
        self.assertEqual(board.set_power, [])


class Timeout(Patched):
    def test_silent_goes_on_answering_aborts(self):
        board = Board(alive=False)
        self.use(board)
        self.assertEqual(mr.MowerOnTimeout('X').execute(None), 'silent')
        board.alive = True
        time.sleep(0.15)
        self.assertEqual(mr.MowerOnTimeout('X').execute(None), 'abort')

    def test_no_watch_is_not_silent(self):
        self.patch(mr, 'power_watch', lambda: None)
        self.board = None
        self.assertEqual(mr.MowerOnTimeout('X').execute(None), 'abort')


class LinkHome(Patched):
    def test_home_with_mower_link(self):
        st = mr.MowerLinkHome(types.SimpleNamespace(log_info=Sink()))
        asked = []

        def request(reason):
            asked.append(reason)
            st.request_preempt()
            return True
        self.patch(mr.run_flags, 'home_latch', types.SimpleNamespace(request=request))
        self.board = None
        ud = types.SimpleNamespace(error_reason='')
        self.assertEqual(st.execute(ud), 'preempted')
        self.assertEqual(asked, ['mower_link'])
        self.assertEqual(ud.error_reason, 'mower_link')

    def test_no_latch_aborts(self):
        st = mr.MowerLinkHome(types.SimpleNamespace(log_info=Sink()))
        self.patch(mr.run_flags, 'home_latch', None)
        self.board = None
        ud = types.SimpleNamespace(error_reason='')
        self.assertEqual(st.execute(ud), 'aborted')
        self.assertEqual(ud.error_reason, 'mower_link')


# ---------------------------------------------------------------------------
# The real mission child SM from POWER_ON_MOWER, ZONE_IT replaced by a marker
# ---------------------------------------------------------------------------
class Mission(Patched):
    def build(self, board):
        sys.path.insert(0, SRC)
        from mower_unit_smach import mission, states
        sys.path.remove(SRC)
        import smach
        self.use(board)
        self.patch(states.rospy, 'wait_for_message', board.wait_for_message)
        self.patch(rospy, 'get_param', lambda name, default=None: default)  # no master

        def zone_sm(_pubs, _it):    # the zones need action servers; ZONE_IT is a marker
            inner = smach.StateMachine(outcomes=['succeeded', 'preempted', 'aborted',
                                                 'continue', 'critical_failure'])
            with inner:
                smach.StateMachine.add('NOP', smach.CBState(
                    lambda ud: 'succeeded', outcomes=['succeeded']),
                    transitions={'succeeded': 'succeeded'})
            return inner
        self.patch(mission, '_build_process_zone_sm', zone_sm)
        pubs = make_pubs(board)

        class AnyPubs(types.SimpleNamespace):
            def __getattr__(self, name):
                return Sink()
        sm = mission.build_mission_child_sm(AnyPubs(**vars(pubs)))
        reached = []

        class ZoneMark(smach.State):
            def __init__(self):
                smach.State.__init__(self, outcomes=['succeeded', 'preempted', 'aborted',
                                                     'critical_failure'])

            def execute(self, ud):
                reached.append(True)
                return 'preempted'
        sm._states['ZONE_IT'] = ZoneMark()
        for label in ('POWER_ON_MOWER', 'POWER_ON_MOWER_RETRY'):
            st = sm._states[label]
            st.revive_s, st.off_s = 1.0, 0.2
            st.pub_bat = Sink(board.on_bat)
        for label in ('WAIT_FOR_MOWER_ON', 'WAIT_FOR_MOWER_ON_RETRY'):
            sm._states[label].timeout = 1.5
        sm.set_initial_state(['POWER_ON_MOWER'])
        sm.userdata.program = program()
        latch = types.SimpleNamespace(reason=None)

        def request(reason):
            latch.reason = reason
            sm.request_preempt()
            return True
        latch.request = request
        self.patch(mr.run_flags, 'home_latch', latch)
        return sm, reached, latch

    def test_transitions(self):
        board = Board(alive=True)
        sm, _, _ = self.build(board)
        tr = sm._transitions
        self.assertEqual(tr['POWER_ON_MOWER'], {'done': 'WAIT_FOR_MOWER_ON',
                                                'silent': 'POWER_ON_MOWER_RETRY',
                                                'preempted': 'preempted'})
        self.assertEqual(tr['WAIT_FOR_MOWER_ON']['timeout'], 'MOWER_ON_TIMEOUT')
        self.assertEqual(tr['WAIT_FOR_MOWER_ON']['error'], 'aborted')
        self.assertEqual(tr['MOWER_ON_TIMEOUT'], {'silent': 'POWER_ON_MOWER_RETRY',
                                                  'abort': 'aborted'})
        self.assertEqual(tr['POWER_ON_MOWER_RETRY']['silent'], 'MOWER_LINK_HOME')
        self.assertEqual(tr['WAIT_FOR_MOWER_ON_RETRY']['reached'], 'ZONE_IT')
        self.assertEqual(tr['MOWER_ON_TIMEOUT_RETRY'], {'silent': 'MOWER_LINK_HOME',
                                                        'abort': 'aborted'})
        self.assertEqual(tr['SET_PROGRAM_SPEED']['done'], 'POWER_ON_MOWER')

    def test_live_link_unchanged(self):
        board = Board(alive=True, status='OFF')
        sm, reached, latch = self.build(board)
        self.assertEqual(sm.execute(), 'preempted')     # the marker's outcome
        self.assertEqual(reached, [True])
        self.assertEqual(board.set_power, [True])
        self.assertEqual(board.bat_cmds, [])
        self.assertIsNone(latch.reason)

    def test_dead_link_woken_by_the_pm_goes_on(self):
        board = Board(alive=False, status='READY', wakes_on_bat=True)
        sm, reached, latch = self.build(board)
        self.assertEqual(sm.execute(), 'preempted')
        self.assertEqual(reached, [True])
        self.assertEqual(board.bat_cmds, [True])
        self.assertEqual(board.set_power, [True])
        self.assertIsNone(latch.reason)

    def test_dead_twice_goes_home_as_mower_link(self):
        board = Board(alive=False, status='READY', wakes_on_bat=False)
        sm, reached, latch = self.build(board)
        self.assertEqual(sm.execute(), 'preempted')
        self.assertEqual(reached, [])
        self.assertEqual(latch.reason, 'mower_link')
        self.assertEqual(sm.userdata.error_reason, 'mower_link')
        self.assertEqual(board.bat_cmds, [True, False, True, False])
        self.assertEqual(board.set_power, [])
        self.assertFalse(board.last_bat_out[1])         # rail off for the way home

    def test_dead_off_or_init_retry_and_mower_link_without_bat_out(self):
        # 8. 11.: the board's last word was OFF.  No PM, but the retry and
        # interrupted:mower_link stay.
        for status in ('OFF', 'INIT'):
            board = Board(alive=False, status=status, wakes_on_bat=True)
            sm, reached, latch = self.build(board)
            self.assertEqual(sm.execute(), 'preempted', status)
            self.assertEqual(reached, [], status)
            self.assertEqual(board.bat_cmds, [], status)
            self.assertEqual(board.set_power, [True, True], status)   # first + retry
            self.assertEqual(latch.reason, 'mower_link', status)
            self.assertEqual(sm.userdata.error_reason, 'mower_link', status)
            self.tearDown()
            self.saved = {}

    def test_dies_after_set_power_retry_then_home(self):
        board = Board(alive=True, status='OFF', wakes_on_bat=False, ready_s=99)
        orig = board.on_power

        def on_power(msg):          # the board goes silent at set_power, for good
            orig(msg)
            board.alive = False
        board.on_power = on_power
        sm, reached, latch = self.build(board)
        sm._states['POWER_ON_MOWER'].pubs.mower_set_power.hook = on_power
        self.patch(mr, 'STATUS_SILENT_S', 1.0)     # the 1.5 s wait stands for 30 s
        self.assertEqual(sm.execute(), 'preempted')
        self.assertEqual(reached, [])
        self.assertEqual(board.set_power, [True, True])  # last word OFF: no PM, retry
        self.assertEqual(board.bat_cmds, [])
        self.assertEqual(latch.reason, 'mower_link')

    def test_answering_but_never_ready_aborts_as_before(self):
        board = Board(alive=True, status='OFF', ready_s=99)
        sm, reached, latch = self.build(board)
        self.assertEqual(sm.execute(), 'aborted')
        self.assertEqual(board.set_power, [True])
        self.assertEqual(board.bat_cmds, [])
        self.assertIsNone(latch.reason)


if __name__ == '__main__':
    unittest.main()
