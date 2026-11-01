#!/usr/bin/env python3
"""Firmware TEMP during a mission, offline: fake clock, no master.

    python3 test/test_temp_watch.py      (sourced workspace)
"""
import json
import os
import sys
import threading
import types
import unittest

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:1'     # never the robot's master

import rospy  # noqa: E402
from std_msgs.msg import Bool, String  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'src')
sys.path.insert(0, SRC)
from mower_unit_smach import hold_gate, monitors, states, temp_watch as tw  # noqa: E402
sys.path.remove(SRC)
rospy.rostime.set_rostime_initialized(True)


def feed(w, seq):
    """[(t, status, temp, rpm)] -> the last keep, all actions."""
    keep, actions = True, []
    for t, status, temp, rpm in seq:
        keep, acts = w.on_status(t, status, temp, rpm, 3400)
        actions += acts
    return keep, actions


def mowing(t=0.0):
    return [(t, 'RUN', 20.0, 3400), (t + 0.5, 'RUN', 20.0, 3400)]


# /mower/status as received on 1. 11. 2026
INCIDENT = [(17.5, 'RUN', 20.0, 3400), (18.0, 'RUN', 20.0, 3400), (24.05, 'RUN', 42.5, 3402),
            (24.55, 'RUN', 42.5, -31966), (25.05, 'TEMP', 90.7, -31966),
            (25.55, 'TEMP', 90.7, 2596), (26.05, 'READY', 20.9, 2596)]


class Core(unittest.TestCase):

    def test_incident_rides_through(self):
        w = tw.TempWatch()
        keep, actions = feed(w, INCIDENT)
        self.assertTrue(keep)
        self.assertEqual(actions, ['waiting', 'spike', 'blade_on'])
        self.assertTrue(w.hold)                       # stands until the blade is up
        keep, actions = feed(w, [(26.6, 'RUN', 20.9, 300), (32.0, 'RUN', 20.9, 3390)])
        self.assertTrue(keep)
        self.assertEqual(actions, ['blade_up'])
        self.assertFalse(w.hold)

    def test_implausible_rpm_is_not_up(self):
        w = tw.TempWatch()
        feed(w, INCIDENT)
        for rpm, setpoint in ((-31966, 3400), (32000, 0)):
            self.assertEqual(w.on_status(27.0, 'RUN', 20.9, rpm, setpoint), (True, []))
            self.assertTrue(w.hold)

    def test_real_overtemp_ends_the_mission(self):
        w = tw.TempWatch()
        feed(w, mowing())
        keep, _ = feed(w, [(1 + i * 0.5, 'TEMP', 71.0, 0) for i in range(25)])
        self.assertFalse(keep)
        self.assertEqual(w.reason, 'temp')
        self.assertFalse(w.hold)

    def test_not_below_cool_is_no_spike(self):
        w = tw.TempWatch()
        feed(w, mowing())
        keep, actions = feed(w, [(1.0, 'TEMP', 75.0, 0), (2.0, 'READY', 60.0, 0),
                                 (11.5, 'READY', 60.0, 0)])
        self.assertFalse(keep)
        self.assertNotIn('blade_on', actions)
        self.assertEqual(w.reason, 'temp')

    def test_board_off_no_restart(self):
        w = tw.TempWatch()
        feed(w, mowing())
        keep, actions = feed(w, [(1.0, 'TEMP', 90.0, 0), (2.0, 'OFF', 20.0, 0),
                                 (11.5, 'OFF', 20.0, 0)])
        self.assertFalse(keep)
        self.assertNotIn('blade_on', actions)

    def test_false_during_wait_blocks_the_restart(self):
        w = tw.TempWatch()
        feed(w, mowing() + [(1.0, 'TEMP', 90.0, 0)])
        w.on_command(False)
        keep, actions = feed(w, [(2.0, 'READY', 20.0, 0)])
        self.assertTrue(keep)
        self.assertEqual(actions, ['spike'])
        self.assertFalse(w.hold)

    def test_no_restart_on_a_noblade_area(self):
        w = tw.TempWatch()
        feed(w, mowing() + [(1.0, 'TEMP', 90.0, 0)])
        keep, actions = w.on_status(2.0, 'READY', 20.0, 0, 3400, blade_held=True)
        self.assertTrue(keep)
        self.assertEqual(actions, ['spike'])

    def test_blade_never_on_is_not_started(self):
        w = tw.TempWatch()
        _, actions = feed(w, [(0.0, 'READY', 20.0, 0), (1.0, 'TEMP', 90.0, 0),
                              (2.0, 'READY', 20.0, 0)])
        self.assertNotIn('blade_on', actions)

    def test_repeated_spikes_go_home(self):
        w = tw.TempWatch()
        for n in range(2):
            t = n * 100.0
            feed(w, mowing(t) + [(t + 1, 'TEMP', 90.0, 0), (t + 2, 'READY', 20.0, 0),
                                 (t + 8, 'RUN', 20.0, 3400)])
        keep, _ = feed(w, mowing(300.0) + [(301.0, 'TEMP', 90.0, 0), (302.0, 'READY', 20.0, 0)])
        self.assertFalse(keep)
        self.assertEqual(w.reason, 'temp_spike')

    def test_spikes_age_out(self):
        w = tw.TempWatch()
        for n in range(5):
            t = n * 1000.0
            keep, _ = feed(w, mowing(t) + [(t + 1, 'TEMP', 90.0, 0), (t + 2, 'READY', 20.0, 0),
                                           (t + 8, 'RUN', 20.0, 3400)])
            self.assertTrue(keep)

    def test_blade_not_up_goes_home(self):
        w = tw.TempWatch()
        feed(w, INCIDENT)
        keep, _ = feed(w, [(40.0, 'READY', 20.0, 0), (57.0, 'READY', 20.0, 0)])
        self.assertFalse(keep)
        self.assertEqual(w.reason, 'temp_spike')

    def test_safety_decides_without_firmware(self):
        for state, reason in (('overtemp', 'temp'), ('sensor_fault', 'temp_sensor')):
            w = tw.TempWatch()
            feed(w, mowing())
            w.on_safety(state)
            keep, _ = feed(w, [(1.0, 'OFF', 75.0, 0)])
            self.assertFalse(keep)
            self.assertEqual(w.reason, reason)

    def test_safety_unknown_is_no_verdict(self):
        w = tw.TempWatch()
        w.on_safety('overtemp')
        w.on_safety('unknown')
        self.assertTrue(feed(w, mowing())[0])

    def test_nonsense_after_temp_proves_nothing(self):
        for temp in (float('nan'), 0.0, -273.15):
            w = tw.TempWatch()
            feed(w, mowing())
            keep, actions = feed(w, [(1.0, 'TEMP', 90.0, 0), (2.0, 'READY', temp, 0)])
            self.assertTrue(keep)
            self.assertEqual(actions, ['waiting'])
            self.assertTrue(w.hold)
            keep, _ = feed(w, [(11.5, 'READY', temp, 0)])
            self.assertFalse(keep)
            self.assertEqual(w.reason, 'temp')

    def test_silence_in_wait_is_temp_in_restart_is_spike(self):
        w = tw.TempWatch()
        feed(w, mowing() + [(1.0, 'TEMP', 90.0, 0)])
        self.assertEqual(w.silent(), 'temp')
        self.assertFalse(w.hold)
        w = tw.TempWatch()
        feed(w, INCIDENT)
        self.assertEqual(w.silent(), 'temp_spike')
        self.assertEqual(tw.TempWatch().silent(), None)


class SpinUp(unittest.TestCase):
    """The wait states on a scripted /mower/status and a fake clock."""

    def setUp(self):
        self.real = (states.rospy.wait_for_message, states.rospy.get_time,
                     states.rospy.logwarn, states.RULES.blade_held)
        self.clock = [100.0]
        states.rospy.get_time = lambda: self.clock[0]
        states.rospy.logwarn = lambda *a: None
        states.RULES.blade_held = lambda: False
        self.pub = types.SimpleNamespace(publish=lambda *a: None)

    def tearDown(self):
        (states.rospy.wait_for_message, states.rospy.get_time,
         states.rospy.logwarn, states.RULES.blade_held) = self.real

    def script(self, *messages):
        queue = list(messages)

        def next_status(*_a, **_k):
            self.clock[0] += 0.5
            status, rpm = queue.pop(0)
            return types.SimpleNamespace(status=status, moto_rpm=rpm, temp=20.0)
        states.rospy.wait_for_message = next_status

    def rpm_state(self):
        state = states.WaitForRpmReached(types.SimpleNamespace(
            log_info=self.pub, smach_status=self.pub))
        return state.execute(types.SimpleNamespace(zone_rpm=3400))

    def test_spike_in_spin_up_waits(self):
        self.script(('RUN', 800), ('TEMP', 800), ('TEMP', 700), ('READY', 500), ('RUN', 3400))
        self.assertEqual(self.rpm_state(), 'reached')

    def test_status_wait_too(self):
        self.script(('READY', 0), ('TEMP', 0), ('READY', 0), ('RUN', 3400))
        state = states.WaitForMowerStatus('RUN')
        self.assertEqual(state.execute(None), 'reached')

    def test_temp_that_stays_fails_after_the_monitors_window(self):
        self.script(*[('TEMP', 0)] * 40)
        self.assertEqual(self.rpm_state(), 'error')
        self.assertGreater(self.clock[0] - 100.0, tw.DEFAULTS['wait_s'])

    def test_err_and_blocked_fail_at_once(self):
        for status in ('ERR', 'BLOCKED'):
            self.script((status, 0))
            self.assertEqual(self.rpm_state(), 'error')

    def test_monitor_ends_a_real_temp_before_the_state_gives_up(self):
        w = tw.TempWatch()
        grace = tw.TempGrace()
        feed(w, mowing())
        for i in range(50):
            t = 1.0 + i * 0.5
            keep, _ = w.on_status(t, 'TEMP', 71.0, 0, 3400)
            if not keep:
                break
            self.assertTrue(grace.waits(t, 'TEMP'))
        self.assertEqual(w.reason, 'temp')

    def test_spin_up_without_a_seen_run_restarts(self):
        w = tw.TempWatch()
        w.on_command(True)
        feed(w, [(0.0, 'READY', 20.0, 0), (0.5, 'TEMP', 90.0, 0)])
        _, actions = feed(w, [(1.5, 'READY', 20.0, 0)])
        self.assertIn('blade_on', actions)

    def test_true_refused_during_temp_does_not_veto(self):
        w = tw.TempWatch()
        feed(w, mowing() + [(1.0, 'TEMP', 90.0, 0)])
        w.on_command(False)
        w.on_command(True)
        _, actions = feed(w, [(2.0, 'READY', 20.0, 0)])
        self.assertIn('blade_on', actions)

    def test_blade_switched_off_just_before_stays_off(self):
        w = tw.TempWatch()
        feed(w, mowing())
        w.on_command(False)
        _, actions = feed(w, [(1.0, 'TEMP', 90.0, 0), (2.0, 'READY', 20.0, 0)])
        self.assertNotIn('blade_on', actions)


class Shell(unittest.TestCase):

    class Pub(object):
        def __init__(self):
            self.sent = []

        def publish(self, msg):
            self.sent.append(msg.data)

    def setUp(self):
        self.real = (rospy.logwarn, rospy.loginfo)
        rospy.logwarn = rospy.loginfo = lambda *a: None
        self.hold = hold_gate.HoldGate()
        self.shell = tw.TempWatchShell()
        self.shell.core = tw.TempWatch()
        self.shell.hold = self.hold
        self.pubs = types.SimpleNamespace(log_info=self.Pub(), mower_set_motor_on=self.Pub())
        self.shell.pubs = self.pubs
        self.latch = types.SimpleNamespace(running=True, asked=[])

        def request(reason):
            self.latch.asked.append(reason)
            return self.latch.running
        self.latch.request = request
        self.shell.latch = self.latch
        self.shell.safety_sub = types.SimpleNamespace(get_num_connections=lambda: 1)
        self.watch = monitors.TEMP_WATCH
        monitors.TEMP_WATCH = self.shell

    def tearDown(self):
        monitors.TEMP_WATCH = self.watch
        rospy.logwarn, rospy.loginfo = self.real

    @staticmethod
    def msg(status, temp, rpm=3400):
        return types.SimpleNamespace(status=status, temp=temp, moto_rpm=rpm, setpoint_rpm=3400)

    def test_monitor_holds_restarts_and_releases(self):
        cb = monitors.mower_temp_monitor_cb
        self.assertTrue(cb(None, self.msg('RUN', 20.0)))
        self.assertTrue(cb(None, self.msg('TEMP', 90.7, 0)))
        self.assertTrue(self.hold._held)
        self.assertTrue(cb(None, self.msg('READY', 20.9, 0)))
        self.assertEqual(self.pubs.mower_set_motor_on.sent, [True])
        self.assertTrue(self.hold._held)
        self.assertTrue(cb(None, self.msg('RUN', 20.9)))
        self.assertFalse(self.hold._held)

    def test_silence_releases_the_hold(self):
        self.shell.on_status(self.msg('RUN', 20.0))
        self.shell.on_status(self.msg('TEMP', 90.7, 0))
        self.shell.last_seen -= 10.0
        self.shell._tick(None)
        self.assertFalse(self.hold._held)

    def test_silence_ends_through_the_latch(self):
        self.shell.on_status(self.msg('RUN', 20.0))
        self.shell.on_status(self.msg('TEMP', 90.7, 0))
        self.shell.last_seen -= 10.0
        self.shell._tick(None)
        self.assertEqual(self.latch.asked, ['temp'])
        self.assertEqual(self.shell.stop_reason(), 'interrupted:temp')

    def test_safety_verdict_needs_no_status(self):
        self.shell.core.on_safety('sensor_fault')
        self.shell._tick(None)
        self.assertEqual(self.latch.asked, ['temp_sensor'])
        self.assertEqual(self.shell.stop_reason(), 'interrupted:temp_sensor')
        self.shell._tick(None)
        self.assertEqual(self.latch.asked, ['temp_sensor'])       # once

    def test_verdict_with_no_mission_stays_for_the_next(self):
        self.latch.running = False
        self.shell.core.on_safety('overtemp')
        self.shell._tick(None)
        self.assertIsNone(self.shell.core.reason)

    def test_dead_safety_node_leaves_no_stale_verdict(self):
        self.shell.core.on_safety('overtemp')
        self.shell.safety_sub = types.SimpleNamespace(get_num_connections=lambda: 0)
        self.shell._tick(None)
        self.assertEqual(self.latch.asked, [])
        self.assertTrue(self.shell.on_status(self.msg('RUN', 20.0)))

    def restart_due(self):
        self.shell.on_status(self.msg('RUN', 20.0))
        self.shell.on_status(self.msg('TEMP', 90.7, 0))
        self.shell.on_status(self.msg('READY', 20.9, 0))

    def test_no_true_during_a_mux_hold_then_it_goes_out(self):
        self.hold._mux_held = True
        self.restart_due()
        self.assertEqual(self.pubs.mower_set_motor_on.sent, [])
        self.assertTrue(self.shell.core.hold)
        since = self.shell.core.since = 0.0
        self.shell._tick(None)
        self.assertGreater(self.shell.core.since, since)          # the clock waits too
        self.hold._mux_held = False
        self.shell._tick(None)
        self.assertEqual(self.pubs.mower_set_motor_on.sent, [True])
        self.shell._tick(None)
        self.assertEqual(self.pubs.mower_set_motor_on.sent, [True])

    def test_no_true_while_the_limit_is_stale(self):
        self.hold._stale = True
        self.restart_due()
        self.assertEqual(self.pubs.mower_set_motor_on.sent, [])

    def test_stale_blade_false_cancels_the_true(self):
        watch = types.SimpleNamespace(lock=threading.Lock(),
                                      core=types.SimpleNamespace(commanded=False))
        self.shell.blade_watch = watch
        self.restart_due()
        self.assertEqual(self.pubs.mower_set_motor_on.sent, [])
        self.assertFalse(self.hold._held)
        watch.core.commanded = True
        self.shell.core.commanded = None
        self.restart_due()
        self.assertEqual(self.pubs.mower_set_motor_on.sent, [True])

    def test_stop_reasons(self):
        self.assertEqual(self.shell.stop_reason(), 'interrupted:temp')
        self.shell.core.reason = 'temp_spike'
        self.assertEqual(self.shell.stop_reason(), 'interrupted:temp_spike')
        self.shell.core.reason = 'temp_sensor'
        self.assertEqual(self.shell.stop_reason(), 'interrupted:temp_sensor')

    def test_safety_state_arrives_as_json(self):
        self.shell._safety(String(data=json.dumps({'state': 'overtemp'})))
        self.shell._safety(String(data='not json'))
        self.assertEqual(self.shell.core.safety, 'overtemp')
        self.shell._command(Bool(data=False))

    def test_new_mission_starts_clean(self):
        self.shell.core.reason = 'temp'
        self.shell.last_seen = None
        self.assertTrue(self.shell.on_status(self.msg('RUN', 20.0)))


if __name__ == '__main__':
    unittest.main()
