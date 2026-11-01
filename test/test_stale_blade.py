#!/usr/bin/env python3
"""The blade while the map rules limit is stale, offline: fake clock, no master.

    python3 test/test_stale_blade.py      (sourced workspace)
"""
import os
import sys
import types
import unittest

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:1'     # never the robot's master

import rospy  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'src')
sys.path.insert(0, SRC)
from mower_unit_smach import hold_gate, stale_blade as sb  # noqa: E402
sys.path.remove(SRC)
rospy.rostime.set_rostime_initialized(True)


def mowing():
    b = sb.StaleBlade()
    b.on_program(busy=True, program='Zahrada', stop_reason='')
    b.on_mower('RUN', 3000, 3000)
    b.on_command(True)
    return b


def run(b, t0, t1, stale, step=0.2, on_true=None):
    out = []
    t = t0
    while t < t1 - 1e-9:
        for c in b.tick(t, stale):
            out.append((round(t, 1), c))
            if c:
                b.on_mower('READY', 0, 3000)  # soft start ahead
            else:
                b.on_mower('READY', 0, 3000)  # command obeyed, status follows
        t = round(t + step, 6)
    return out


class Grace(unittest.TestCase):
    def test_a_short_respawn_touches_nothing(self):
        b = mowing()
        self.assertEqual(run(b, 0.0, 1.9, True), [])
        self.assertEqual(run(b, 1.9, 20.0, False), [])
        self.assertFalse(b.owed)

    def test_stale_past_the_grace_stops_it(self):
        b = mowing()
        self.assertEqual(run(b, 0.0, 2.2, True), [(2.0, False)])
        self.assertTrue(b.owed)

    def test_grace_restarts_after_each_recovery(self):
        b = mowing()
        for k in range(5):
            self.assertEqual(run(b, k * 4.0, k * 4.0 + 1.8, True), [])
            self.assertEqual(run(b, k * 4.0 + 1.8, k * 4.0 + 4.0, False), [])

    def test_reasserted_at_most_1hz_when_it_comes_back(self):
        b = mowing()
        run(b, 0.0, 2.2, True)
        b.on_mower('RUN', 800, 3000)          # recovery, or the firmware's retry
        out = run(b, 2.2, 3.4, True)
        self.assertEqual(out, [(3.0, False)])

    def test_nothing_without_a_program(self):
        b = mowing()
        b.on_program(busy=False, program=' ')
        self.assertEqual(run(b, 0.0, 9.0, True), [])

    def test_blade_already_off_is_left_alone(self):
        b = mowing()
        b.on_mower('READY', 0, 0)
        self.assertEqual(run(b, 0.0, 9.0, True), [])
        self.assertEqual(run(b, 9.0, 30.0, False), [])


class Resume(unittest.TestCase):
    def stopped(self):
        b = mowing()
        run(b, 0.0, 3.0, True)
        return b

    def test_true_after_a_stable_limit_then_released_when_up(self):
        b = self.stopped()
        self.assertEqual(run(b, 3.0, 7.8, False), [])
        self.assertTrue(b.owed, 'the hold stands')
        self.assertEqual(run(b, 7.8, 8.2, False), [(8.0, True)])
        self.assertTrue(b.owed, 'still held during the soft start')
        b.on_mower('RUN', 1500, 3000)
        b.tick(9.0, False)
        self.assertTrue(b.owed, 'rpm not there yet')
        b.on_mower('RUN', 2950, 3000)
        b.tick(10.0, False)
        self.assertFalse(b.owed)

    def test_no_spin_up_is_failed_and_released(self):
        b = self.stopped()
        run(b, 3.0, 8.3, False)
        run(b, 8.3, 40.0, False)
        self.assertFalse(b.owed)
        self.assertTrue(b.failed)
        b.on_command(True)                    # BLOCKED recovery starts it itself
        self.assertFalse(b.failed)

    def test_mission_false_cancels(self):
        b = self.stopped()
        b.on_command(False)
        self.assertEqual(run(b, 3.0, 30.0, False), [])
        self.assertFalse(b.owed)

    def test_stop_reason_cancels(self):
        b = self.stopped()
        b.on_program(stop_reason='interrupted:battery')
        self.assertEqual(run(b, 3.0, 30.0, False), [])
        self.assertFalse(b.owed)

    def test_program_end_cancels(self):
        b = self.stopped()
        b.on_program(busy=False, program=' ')
        self.assertEqual(run(b, 3.0, 30.0, False), [])

    def test_mission_had_it_off_before_is_no_debt(self):
        b = mowing()
        b.on_command(False)
        b.on_mower('READY', 1500, 3000)       # coasting into the stale
        run(b, 0.0, 3.0, True)
        self.assertEqual(run(b, 3.0, 30.0, False), [])

    def test_temp_is_not_an_attempt(self):
        b = self.stopped()
        b.on_mower('TEMP', 0, 3000)
        self.assertEqual(run(b, 3.0, 30.0, False), [])
        self.assertTrue(b.owed)


class CrashLoop(unittest.TestCase):
    def test_a_guard_that_keeps_dying_never_cycles_the_blade(self):
        b = mowing()
        cmds = []
        t = 0.0
        for _ in range(50):                   # 3 s alive, 3 s silent
            cmds += run(b, t, t + 3.0, True)
            cmds += run(b, t + 3.0, t + 6.0, False)
            t += 6.0
        self.assertEqual([c for _, c in cmds], [False])
        self.assertTrue(b.owed, 'robot stands, blade off')

    def test_longer_lives_back_off(self):
        b = mowing()
        trues = 0
        t = 0.0
        for _ in range(30):                   # 12 s alive, 4 s silent, for 8 minutes
            b.on_mower('RUN', 3000, 3000) if not b.owed else None
            for _when, c in run(b, t, t + 4.0, True) + run(b, t + 4.0, t + 16.0, False):
                trues += 1 if c else 0
            t += 16.0
        self.assertLessEqual(trues, 2)

    def test_a_healthy_guard_ends_the_wait(self):
        b = mowing()
        run(b, 0.0, 3.0, True)
        self.assertEqual(len([c for _, c in run(b, 3.0, 9.0, False) if c]), 1)
        b.on_mower('RUN', 3000, 3000)
        b.tick(9.2, False)
        self.assertFalse(b.owed)


class Review2(unittest.TestCase):
    def stopped(self):
        b = mowing()
        run(b, 0.0, 3.0, True)
        return b

    def test_m1_fault_before_the_restart_drops_the_debt(self):
        for fault in ('ERR', 'BLOCKED'):
            b = self.stopped()
            b.on_mower(fault, 0, 3000)
            self.assertEqual(run(b, 3.0, 4.0, False), [])
            self.assertFalse(b.owed, fault)

    def test_m1_temp_still_waits(self):
        b = self.stopped()
        b.on_mower('TEMP', 0, 3000)
        run(b, 3.0, 4.0, False)
        self.assertTrue(b.owed)

    def test_m3_guards_false_keeps_the_debt(self):
        b = self.stopped()
        b.on_command(False, guard=True)
        self.assertTrue(b.owed)

    def test_m3_mission_false_still_clears(self):
        b = self.stopped()
        b.on_command(False)
        self.assertFalse(b.owed)

    def test_m3_coast_is_not_commanded_off(self):
        # A False at a firmware that is already off only confuses the guard.
        b = mowing()
        b.on_mower('READY', 1500, 3000)
        self.assertEqual(run(b, 0.0, 3.0, True), [])

    def test_l1_mission_false_right_after_our_true_gets_one_more(self):
        b = self.stopped()
        self.assertEqual(run(b, 3.0, 8.2, False), [(8.0, True)])
        self.assertTrue(b.on_command(False, now=8.5))
        self.assertFalse(b.on_command(False, now=8.6), 'once')

    def test_l1_later_false_is_plain(self):
        b = self.stopped()
        run(b, 3.0, 8.2, False)
        self.assertFalse(b.on_command(False, now=9.5))

    def test_l2_on_a_noblade_no_true_and_the_robot_goes_on(self):
        b = self.stopped()
        self.assertEqual(b.tick(8.0, False, noblade=True), [])
        self.assertFalse(b.owed, 'the hold must end: only the mission can drive out')
        for t in range(9, 40):
            self.assertEqual(b.tick(float(t), False, noblade=True), [])

    def test_l2_never_silent_after_the_area(self):
        b = self.stopped()
        b.tick(8.0, False, noblade=True)
        self.assertEqual(b.tick(30.0, False, noblade=False), [], 'settles first')
        self.assertEqual(b.tick(35.0, False, noblade=False), [True])
        self.assertTrue(b.owed, 'with the rpm check')
        b.on_mower('RUN', 3000, 3000)
        b.tick(31.0, False)
        self.assertFalse(b.owed)

    def test_l2_mission_false_in_between_ends_it(self):
        b = self.stopped()
        b.tick(8.0, False, noblade=True)
        b.on_command(False)
        self.assertEqual(b.tick(30.0, False), [])

    def test_l2_true_taken_by_the_guard_is_the_guards(self):
        b = self.stopped()
        run(b, 3.0, 8.2, False)
        self.assertEqual(b.tick(8.4, False, noblade=True), [])
        self.assertFalse(b.owed)
        self.assertEqual(b.tick(30.0, False), [], 'the guard owes it now')

    def test_l3_no_restart_during_a_mux_hold(self):
        b = self.stopped()
        for t in (3.0, 8.0, 20.0):
            self.assertEqual(b.tick(t, False, mux_held=True), [])
        self.assertTrue(b.owed)
        self.assertEqual(b.tick(21.0, False), [])
        self.assertEqual(b.tick(26.0, False), [True])

    def test_l4_flapping_stale_does_not_reset_the_grace(self):
        b = mowing()
        out = []
        t = 0.0
        while t < 4.0:
            out += b.tick(t, True) + b.tick(t + 0.4, False)
            t += 0.8
        self.assertEqual(out, [False])


class Delta(unittest.TestCase):
    def deferred(self):
        b = mowing()
        run(b, 0.0, 3.0, True)
        b.tick(8.0, False, noblade=True)
        self.assertTrue(b.deferred)
        return b

    def test_d1_no_true_in_temp_and_it_keeps_waiting(self):
        b = self.deferred()
        b.on_mower('TEMP', 0, 3000)
        self.assertEqual(b.tick(30.0, False), [])
        self.assertTrue(b.deferred)
        b.on_mower('READY', 0, 3000)
        b.tick(31.0, False)
        self.assertEqual(b.tick(36.0, False), [True])

    def test_d1_err_and_blocked_drop(self):
        for fault in ('ERR', 'BLOCKED'):
            b = self.deferred()
            b.on_mower(fault, 0, 3000)
            self.assertEqual(b.tick(30.0, False), [])
            self.assertFalse(b.deferred or b.owed, fault)

    def test_d1_already_running_needs_no_true(self):
        for status, rpm in (('RUN', 3000), ('READY', 1500)):
            b = self.deferred()
            b.on_mower(status, rpm, 3000)
            self.assertEqual(b.tick(30.0, False), [])
            self.assertFalse(b.deferred or b.owed, status)

    def test_d1_settles_before_the_true(self):
        b = self.deferred()
        self.assertEqual(b.tick(10.0, False), [], 'off the area just now')
        self.assertEqual(b.tick(14.9, False), [])
        self.assertEqual(b.tick(15.0, False), [True])

    def test_d1_back_on_the_area_restarts_the_settle(self):
        b = self.deferred()
        b.tick(10.0, False)
        b.tick(12.0, False, noblade=True)
        self.assertEqual(b.tick(15.0, False), [])
        self.assertEqual(b.tick(20.0, False), [True])

    def test_d1_rpm_check_after_the_true(self):
        b = self.deferred()
        b.tick(10.0, False)
        b.tick(15.0, False)
        self.assertTrue(b.owed, 'held until up')
        b.tick(45.1, False)
        self.assertTrue(b.failed)

    def test_program_change_drops_the_debt(self):
        b = mowing()
        run(b, 0.0, 3.0, True)
        b.on_program(program='Jiny')
        self.assertFalse(b.owed)

    def test_program_change_drops_the_deferred(self):
        b = self.deferred()
        b.on_program(program='Jiny')
        self.assertEqual(b.tick(30.0, False), [])
        self.assertFalse(b.deferred)

    def test_a_late_name_is_no_change(self):
        b = sb.StaleBlade()
        b.on_program(busy=True)
        b.on_mower('RUN', 3000, 3000)
        run(b, 0.0, 3.0, True)
        self.assertTrue(b.owed)
        b.on_program(program='Zahrada')
        self.assertTrue(b.owed)
        b.on_program(program='Jiny')
        self.assertFalse(b.owed)

    def test_escalates_after_10_minutes_of_a_running_program(self):
        b = mowing()
        run(b, 0.0, 599.0, True, step=1.0)
        self.assertFalse(b.escalate_due(599.0))
        b.tick(600.0, True)
        self.assertTrue(b.escalate_due(600.0))

    def test_flapping_does_not_reset_the_10_minutes(self):
        b = mowing()
        t = 0.0
        while t < 601.0:
            b.tick(t, True)
            b.tick(t + 2.0, False)
            t += 2.5
        self.assertTrue(b.escalate_due(t))

    def test_idle_does_not_count(self):
        b = mowing()
        b.on_program(busy=False, program=' ')
        run(b, 0.0, 700.0, True, step=1.0)
        self.assertFalse(b.escalate_due(700.0))
        b.on_program(busy=True, program='Z')
        b.tick(700.0, True)
        self.assertFalse(b.escalate_due(700.0), 'a program started into it has its 10 min')

    def test_recovery_ends_the_count(self):
        b = mowing()
        run(b, 0.0, 300.0, True, step=1.0)
        run(b, 300.0, 303.0, False, step=1.0)
        run(b, 303.0, 700.0, True, step=1.0)
        self.assertFalse(b.escalate_due(700.0))


class Gate(unittest.TestCase):
    def test_hold_stands_until_the_blade_is_up(self):
        h = hold_gate.HoldGate()
        h._status_cb(String('{"limit": {"stale": true}}'))
        self.assertTrue(h.limit_stale())
        h.set_blade_wait(True)
        h._status_cb(String('{"limit": {"stale": false}}'))
        self.assertTrue(h._held, 'limit back, blade not')
        h.set_blade_wait(False)
        self.assertFalse(h._held)

    def test_l3_mux_hold_is_told_apart(self):
        h = hold_gate.HoldGate()
        h.set_blade_wait(True)
        self.assertFalse(h.mux_held())
        h._cb(types.SimpleNamespace(data=True))
        self.assertTrue(h.mux_held())

    def test_l10_the_wait_says_why(self):
        h = hold_gate.HoldGate()
        h.set_blade_wait(True)
        said = types.SimpleNamespace(sent=[], publish=None)
        said.publish = said.sent.append
        real = hold_gate.rospy.sleep
        hold_gate.rospy.sleep = lambda _t: h.set_blade_wait(False)
        h._sub = h._status_sub = object()
        try:
            h.wait(None, said)
        finally:
            hold_gate.rospy.sleep = real
        self.assertEqual(said.sent, ['Held: blade coming back'])


class Watch(unittest.TestCase):
    def watch(self):
        real = (sb.rospy.Subscriber, sb.rospy.Timer, sb.rospy.get_name)
        sb.rospy.Subscriber = lambda *a, **k: None
        sb.rospy.Timer = lambda *a, **k: None
        sb.rospy.get_name = lambda: '/mower_unit_smach'
        self.addCleanup(lambda: (setattr(sb.rospy, 'Subscriber', real[0]),
                                 setattr(sb.rospy, 'Timer', real[1]),
                                 setattr(sb.rospy, 'get_name', real[2])))
        sent = []
        raw = types.SimpleNamespace(publish=lambda m: sent.append(m.data))
        logs = types.SimpleNamespace(publish=lambda m: None)
        pubs = types.SimpleNamespace(mower_set_motor_on=raw, log_info=logs)
        hold = types.SimpleNamespace(start=lambda: None, limit_stale=lambda: False,
                                     mux_held=lambda: False, set_blade_wait=lambda v: None)
        rules = types.SimpleNamespace(set_local_failed=lambda v: None,
                                      noblade_here=lambda: False)
        real_param = sb.rospy.get_param
        sb.rospy.get_param = lambda name, default=None: default
        self.addCleanup(lambda: setattr(sb.rospy, 'get_param', real_param))
        return sb.StaleBladeWatch(pubs, hold, rules), pubs, sent

    def test_the_missions_false_is_seen_and_still_goes_out(self):
        w, pubs, sent = self.watch()
        w.core.owed = True
        pubs.mower_set_motor_on.publish(Bool(False))
        self.assertEqual(sent, [False])
        self.assertFalse(w.core.owed)

    def test_m3_the_guards_false_on_the_topic(self):
        w, _pubs, _sent = self.watch()
        w.core.owed = True
        msg = Bool(False)
        msg._connection_header = {'callerid': '/map_rules_guard'}
        w._command(msg)
        self.assertTrue(w.core.owed)

    def test_l1_a_crossing_false_goes_out_twice(self):
        w, pubs, sent = self.watch()
        w.core.last_true = sb.time.monotonic()
        pubs.mower_set_motor_on.publish(Bool(False))
        self.assertEqual(sent, [False, False])

    def test_the_guards_callerid_is_a_param(self):
        w, _pubs, _sent = self.watch()
        self.assertEqual(w.guard, '/map_rules_guard')
        w.guard = '/other_guard'
        w.core.owed = True
        msg = Bool(False)
        msg._connection_header = {'callerid': '/other_guard'}
        w._command(msg)
        self.assertTrue(w.core.owed)

    def test_escalation_goes_to_the_latch_once(self):
        w, _pubs, _sent = self.watch()
        asked = []
        w.latch = types.SimpleNamespace(request=lambda r: asked.append(r) or True)
        w.core.escalate_due = lambda now: True
        w.tick(None)
        w.tick(None)
        self.assertEqual(asked, [sb.TERMINAL_REASON])

    def test_escalation_retries_while_no_mission_child_runs(self):
        w, _pubs, _sent = self.watch()
        asked = []
        w.latch = types.SimpleNamespace(request=lambda r: asked.append(r) or False)
        w.core.escalate_due = lambda now: True
        w.tick(None)
        w.tick(None)
        self.assertEqual(len(asked), 2)

    def test_own_callerid_on_the_topic_is_not_a_mission_command(self):
        w, _pubs, _sent = self.watch()
        w.core.owed = True
        msg = Bool(False)
        msg._connection_header = {'callerid': '/mower_unit_smach'}
        w._command(msg)
        self.assertTrue(w.core.owed)

    def test_another_nodes_false_counts(self):
        w, _pubs, _sent = self.watch()
        w.core.owed = True
        msg = Bool(False)
        msg._connection_header = {'callerid': '/ui'}
        w._command(msg)
        self.assertFalse(w.core.owed)

    def test_a_tick_failure_does_not_kill_the_timer(self):
        w, _pubs, _sent = self.watch()
        w.hold.limit_stale = lambda: 1 / 0
        w.tick(None)


if __name__ == '__main__':
    unittest.main()
