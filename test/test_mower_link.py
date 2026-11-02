#!/usr/bin/env python3
"""The mower board silent or the blade stopped in a mission, offline: fake clock, no
master.    python3 test/test_mower_link.py      (sourced workspace)
"""
import json
import os
import sys
import threading
import unittest
from unittest import mock

os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:1'     # never the robot's master

import rospy  # noqa: E402,F401
from std_msgs.msg import Bool  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
SRC = os.path.join(HERE, '..', 'src')
sys.path.insert(0, SRC)
from mower_unit_smach import mower_link as ml, stale_blade as sb  # noqa: E402
sys.path.remove(SRC)


class Board(object):
    """/mower/status at 2 Hz and the 0.5 s tick of the shell."""

    def __init__(self, params=None):
        self.w = ml.MowerLink(params)
        self.t = 100.0
        self.actions = []
        self.status = ('RUN', 3400)
        self.silent = False
        self.height = 70
        self.running = True
        self.tick_args = {}

    def mow(self):
        self.w.on_command(True, self.t)
        self.run(20.0)
        self.actions = []

    def run(self, seconds):
        end = self.t + seconds
        while self.t < end:
            self.t += 0.5
            if not self.silent:
                self.w.on_status(self.t, self.status[0], self.status[1], 3400, self.height)
            got = self.w.tick(self.t, self.running, **self.tick_args)
            self.actions += got
            self.react(got)

    def react(self, got):
        pass


class IncidentBoard(Board):
    """21:35: it never answers, whatever is done to it."""


class HealsOnPower(Board):
    """Answers after the power cycle, restarted: HOME, at max height, blade off."""

    def react(self, got):
        if 'bat_off' in got:
            self.silent = False
            self.status = ('OFF', 0)
            self.height = 80
        if 'set_power' in got:
            self.status = ('HOME', 0)
        if ('height', 70) in got:
            self.height = 70
            self.status = ('READY', 0)
        if self.status[0] == 'HOME':
            self.status = ('READY', 0)
        if 'blade_on' in got:
            self.status = ('RUN', 3400)


class Silent(unittest.TestCase):

    def test_nothing_without_the_condition(self):
        b = Board()
        b.mow()
        b.run(600.0)
        self.assertEqual(b.actions, [])
        self.assertFalse(b.w.active)

    def test_short_gap_is_no_trigger(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(2.5)
        b.silent = False
        b.run(5.0)
        self.assertEqual(b.actions, [])

    def test_holds_within_silent_s_and_stops_the_blade_first(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(4.0)
        self.assertEqual(b.actions, ['hold'])
        b.run(3.0)
        self.assertIn('blade_off', b.actions)
        self.assertEqual(b.w.why, 'silent')

    def test_incident_ends_in_a_bounded_way_home(self):
        b = IncidentBoard()
        b.mow()
        b.silent = True
        t0 = b.t
        b.run(120.0)
        order = [a for a in b.actions if not isinstance(a, tuple)]
        for first, then in (('hold', 'restart_serial'), ('restart_serial', 'bat_off'),
                            ('bat_off', 'home')):
            self.assertLess(order.index(first), order.index(then))
        i = order.index('home')
        self.assertEqual(order[i - 2:i + 2], ['blade_off', 'bat_off', 'home', 'release'])
        self.assertTrue(b.w.failed)
        self.assertLessEqual(b.w.since + b.w.p['attempt_s'] - t0, 93.5)
        n = len(b.actions)
        b.run(600.0)
        self.assertEqual(len(b.actions), n)         # no second round

    def test_no_true_at_a_silent_board(self):
        b = IncidentBoard()
        b.mow()
        b.silent = True
        b.run(120.0)
        self.assertNotIn('blade_on', b.actions)
        self.assertNotIn('set_power', b.actions)    # it would not arrive

    def test_power_stays_off_while_it_is_silent(self):
        b = IncidentBoard()
        b.mow()
        b.w.bat_out = True                          # the PM does not confirm the off
        b.silent = True
        b.run(28.0)
        self.assertIn('bat_off', b.actions)
        self.assertEqual(b.w.phase, 'power_off')
        b.run(40.0)
        self.assertEqual(b.w.phase, 'power_off')
        self.assertNotIn('set_power', b.actions)

    def test_quiet_after_a_board_restart_on_purpose(self):
        b = Board()
        b.mow()
        b.w.on_power()                              # /mower/set_power by anybody
        b.silent = True
        b.run(40.0)
        self.assertEqual(b.actions, [])


class Back(unittest.TestCase):

    def test_transient_stall_comes_back_alone(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(6.0)
        b.silent = False                            # 1. 11., 17:20: 6 s, nothing lost
        b.status = ('READY', 0)                     # our False arrived
        b.run(1.0)
        self.assertEqual(b.actions[-1:], ['blade_on'])
        self.assertEqual([a for a in b.actions if a[0] == 'rpm'], [('rpm', 3400)])
        self.assertNotIn('restart_serial', b.actions)
        b.status = ('RUN', 3400)
        b.run(1.0)
        self.assertEqual(b.actions[-1:], ['release'])
        self.assertFalse(b.w.active)

    def test_still_turning_needs_no_true(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(5.0)
        b.silent = False
        b.run(1.0)
        self.assertEqual(b.actions[-1], 'release')
        self.assertNotIn('blade_on', b.actions)

    def test_after_a_power_cycle_height_and_rpm_as_they_were(self):
        b = HealsOnPower()
        b.mow()
        b.silent = True
        b.run(70.0)
        self.assertFalse(b.w.active, b.actions)
        a = b.actions
        self.assertLess(a.index('bat_off'), a.index('set_power'))
        self.assertLess(a.index('set_power'), a.index(('height', 70)))
        self.assertLess(a.index(('height', 70)), a.index('blade_on'))
        self.assertEqual(a[-1], 'release')
        self.assertFalse(b.w.failed)

    def test_alive_mcu_after_a_power_cycle_gets_the_firmware_restart(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(28.0)
        b.w.bat_out = False                         # confirmed
        b.run(3.5)
        b.silent = False
        b.status = ('READY', 0)                     # no INIT/HOME: it did not restart
        b.run(2.0)
        self.assertEqual(b.actions[-1], 'set_power')

    def test_a_person_said_off_during_it(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(5.0)
        b.w.on_command(False, b.t)
        b.silent = False
        b.status = ('READY', 0)
        b.run(1.0)
        self.assertNotIn('blade_on', b.actions)
        self.assertEqual(b.actions[-1], 'release')

    def test_mission_blade_off_span_gets_no_true(self):
        b = Board()
        b.mow()
        b.w.on_command(False, b.t)                  # NAV_RECOVERY blade off
        b.silent = True
        b.run(5.0)
        self.assertIn('hold', b.actions)            # the board is still unknown
        b.silent = False
        b.status = ('READY', 0)
        b.run(1.0)
        self.assertNotIn('blade_on', b.actions)

    def test_a_rule_holds_the_blade_no_true(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(5.0)
        b.silent = False
        b.status = ('READY', 0)
        b.tick_args = {'rule_held': True}
        b.run(1.0)
        self.assertNotIn('blade_on', b.actions)
        self.assertEqual(b.actions[-1], 'release')

    def test_faults_are_the_missions(self):
        for status in ('ERR', 'BLOCKED', 'TEMP'):
            b = Board()
            b.mow()
            b.silent = True
            b.run(5.0)
            b.silent = False
            b.status = (status, 0)
            b.run(1.0)
            self.assertEqual(b.actions[-1], 'release', status)
            self.assertNotIn('blade_on', b.actions)

    def test_foreign_hold_waits_for_the_true_without_using_the_attempt(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(5.0)
        b.silent = False
        b.status = ('READY', 0)
        b.tick_args = {'blocked': True}
        b.run(200.0)
        self.assertNotIn('blade_on', b.actions)
        self.assertTrue(b.w.active)
        b.tick_args = {}
        b.run(1.0)
        self.assertIn('blade_on', b.actions)


class Stopped(unittest.TestCase):

    def test_blade_stopped_with_a_talking_board(self):
        b = Board()
        b.mow()
        b.status = ('READY', 0)
        b.run(5.0)
        self.assertEqual(b.actions, [])             # rpm_low_s
        b.run(2.0)
        self.assertEqual(b.actions[0], 'hold')
        self.assertIn('blade_on', b.actions)
        self.assertEqual(b.w.why, 'stopped')
        b.status = ('RUN', 3400)
        b.run(1.0)
        self.assertEqual(b.actions[-1], 'release')

    def test_not_spinning_up_gets_the_known_reset_once_then_fails(self):
        b = Board()
        b.mow()
        b.status = ('READY', 0)
        b.run(40.0)
        self.assertEqual(b.actions.count('blade_on'), 2)
        self.assertEqual(b.actions.count('set_power'), 1)
        self.assertLess(b.actions.index('set_power'), b.actions.index('home'))
        self.assertNotIn('bat_off', b.actions)      # it talks: the blade is known off

    def test_spin_up_is_not_a_fault(self):
        b = Board()
        b.mow()
        b.w.on_command(True, b.t)
        b.status = ('RUN', 200)
        b.run(14.0)
        b.status = ('RUN', 3400)
        b.run(10.0)
        self.assertEqual(b.actions, [])

    def test_what_stops_the_blade_on_purpose(self):
        for args, status in (({'free': False}, ('READY', 0)),
                             ({}, ('ERR', 0)), ({}, ('BLOCKED', 0)), ({}, ('TEMP', 0))):
            b = Board()
            b.mow()
            b.tick_args = args
            b.status = status
            b.run(60.0)
            self.assertEqual(b.actions, [], (args, status))

    def test_commanded_off_is_off(self):
        b = Board()
        b.mow()
        b.w.on_command(False, b.t)
        b.status = ('READY', 0)
        b.run(60.0)
        self.assertEqual(b.actions, [])

    def test_garbage_rpm_is_no_reading(self):
        b = Board()
        b.mow()
        b.status = ('RUN', -31966)
        b.run(20.0)
        self.assertEqual(b.actions, [])


class Gap(unittest.TestCase):
    """A status gap under load is no failure: hold only, nothing counts."""

    def test_back_within_gap_s_only_held(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(5.0)
        b.silent = False
        b.run(1.0)
        self.assertEqual(b.actions, ['hold', 'release'])
        self.assertFalse(b.w.active)
        self.assertEqual(len(b.w.history), 0)

    def test_gaps_never_reach_the_limit(self):
        b = Board()
        b.mow()
        for _ in range(10):
            b.silent = True
            b.run(5.0)
            b.silent = False
            b.run(2.0)
        self.assertNotIn('home', b.actions)
        self.assertNotIn('blade_off', b.actions)

    def test_over_gap_s_counts_once_and_the_ladder_starts_at_the_gap(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(7.0)
        self.assertEqual(len(b.w.history), 1)
        self.assertEqual(b.w.phase, 'resync')
        self.assertLess(b.w.since, b.w.seen_t + 4.0)

    def test_stop_ends_a_gap(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(4.0)
        self.assertEqual(b.w.tick(b.t, False), ['blade_off', 'bat_off', 'release'])
        self.assertFalse(b.w.active)


class TurnsAgainstTheFalse(unittest.TestCase):
    """Status fresh, the False ignored: the way home must not release that blade."""

    def _spent(self):
        b = Board()
        b.mow()
        b.w.on_command(False, b.t)                  # the mission's last word
        b.silent = True
        b.run(4.0)                                  # a gap: still RUN at speed when it ends
        b.silent = False
        b.w.bat_out = True
        b.run(b.w.p['attempt_s'] + 2.0)
        return b

    def test_failing_cuts_the_power(self):
        b = self._spent()
        self.assertTrue(b.w.failed)
        a = b.actions
        self.assertLess(a.index('bat_off'), a.index('home'))
        self.assertLess(a.index('home'), a.index('release'))

    def test_the_cut_is_repeated_until_the_pm_says_off(self):
        b = self._spent()
        n = b.actions.count('bat_off')
        b.run(30.0)
        self.assertGreater(b.actions.count('bat_off'), n)
        self.assertLessEqual(b.actions.count('bat_off'), b.w.p['rail_max'])
        b.w.on_bat_out(False)
        b.run(1.0)
        n = b.actions.count('bat_off')
        b.run(30.0)
        self.assertEqual(b.actions.count('bat_off'), n)

    def test_own_cut_is_not_a_persons(self):
        b = self._spent()
        b.w.on_bat_out(False)
        b.run(2.0)
        self.assertFalse(b.w.foreign_off)

    def test_a_blade_at_rest_keeps_its_power(self):
        b = Board()
        b.mow()
        b.status = ('READY', 0)
        b.run(40.0)
        self.assertIn('home', b.actions)
        self.assertNotIn('bat_off', b.actions)


class Bounds(unittest.TestCase):

    def test_three_in_the_window_the_fourth_goes_home(self):
        b = Board()
        b.mow()
        for _ in range(3):
            b.silent = True
            b.run(7.0)
            b.silent = False
            b.run(2.0)
            self.assertFalse(b.w.active)
            self.assertNotIn('home', b.actions)
        b.silent = True
        b.run(7.0)
        self.assertIn('home', b.actions)
        self.assertNotIn('restart_serial', b.actions)

    def test_the_window_forgets(self):
        b = Board()
        b.mow()
        for _ in range(4):
            b.silent = True
            b.run(7.0)
            b.silent = False
            b.run(700.0)
        self.assertNotIn('home', b.actions)

    def test_history_outlives_a_mission(self):
        b = Board()
        b.mow()
        for _ in range(3):
            b.silent = True
            b.run(7.0)
            b.silent = False
            b.run(2.0)
        b.w.reset()
        b.mow()
        b.silent = True
        b.run(7.0)
        self.assertIn('home', b.actions)


class Review(unittest.TestCase):
    """The independent review of 2. 11.: each of these failed on the code before it."""

    def test_c1_no_power_back_without_proof(self):
        b = IncidentBoard()
        b.mow()
        b.silent = True
        b.w.bat_out = True
        b.run(120.0)
        self.assertIn('bat_off', b.actions)
        self.assertNotIn('bat_on', b.actions)
        self.assertIn('home', b.actions)

    def test_c1_a_live_board_with_the_rail_off_gets_set_power_not_bat_on(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(27.0)
        self.assertIn('bat_off', b.actions)
        b.status = ('OFF', 0)
        b.silent = False
        b.run(1.0)
        self.assertIn('set_power', b.actions)
        self.assertNotIn('bat_on', b.actions)

    def _ended(self, run_s):
        b = Board()
        b.mow()
        b.silent = True
        b.run(run_s)
        self.assertTrue(b.w.active)
        b.running = False
        b.run(1.0)
        return b.actions

    def test_c2_mission_end_in_the_ladder_cuts_the_rail(self):
        a = self._ended(8.0)
        self.assertIn('bat_off', a)
        self.assertIn('release', a)
        self.assertLess(a.index('bat_off'), a.index('release'))

    def test_c2_status_fresh_no_cut(self):
        b = Board()
        b.mow()
        b.status = ('RUN', 0)
        b.run(7.0)
        self.assertTrue(b.w.active)
        b.running = False
        b.run(1.0)
        self.assertNotIn('bat_off', b.actions)

    def test_h1_safety_stop_in_silence_goes_home(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(8.0)
        b.w.on_power()
        b.run(1.0)
        self.assertIn('home', b.actions)
        self.assertIn('bat_off', b.actions)

    def test_h1_safety_stop_with_status_is_only_released(self):
        b = Board()
        b.mow()
        b.status = ('RUN', 0)
        b.run(7.0)
        b.w.on_power()
        b.run(1.0)
        self.assertIn('release', b.actions)
        self.assertNotIn('home', b.actions)

    def test_m1_coasting_rpm_after_the_links_false_proves_nothing(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(7.0)
        self.assertIn('blade_off', b.actions)
        b.silent = False                    # RUN at 3400: the board has not read the False
        b.run(1.0)
        self.assertIn('blade_on', b.actions)
        self.assertTrue(b.w.active or b.actions.index('blade_on') < b.actions.index('release'))

    def test_m2_a_persons_power_off_is_not_overridden(self):
        b = Board()
        b.mow()
        b.w.on_bat_out(True)
        b.w.on_bat_out(False)
        b.status = ('OFF', 0)
        b.run(25.0)
        self.assertNotIn('set_power', b.actions)
        self.assertIn('home_off', b.actions)
        self.assertNotIn('home', b.actions)

    def test_m2_our_own_cut_is_no_person(self):
        b = Board()
        b.mow()
        b.w.on_bat_out(True)
        b.silent = True
        b.run(27.0)
        b.w.on_bat_out(False)
        b.status = ('OFF', 0)
        b.silent = False
        b.run(1.0)
        self.assertIn('set_power', b.actions)
        self.assertNotIn('home', b.actions)

    def test_m4_home_before_release(self):
        b = IncidentBoard()
        b.mow()
        b.silent = True
        b.run(120.0)
        self.assertLess(b.actions.index('home'), b.actions.index('release'))

    def test_m4_bat_off_repeats_until_read_back_and_is_bounded(self):
        b = IncidentBoard()
        b.mow()
        b.silent = True
        b.w.bat_out = True                  # never reads False
        b.run(150.0)
        n = b.actions.count('bat_off')
        self.assertGreater(n, 3)
        self.assertLessEqual(n, 21)

    def test_m4_read_back_ends_the_repeat(self):
        b = IncidentBoard()
        b.mow()
        b.silent = True
        b.w.bat_out = True
        b.run(28.0)
        b.w.bat_out = False
        n = b.actions.count('bat_off')
        b.run(120.0)
        self.assertLessEqual(b.actions.count('bat_off'), n + 1)


class FakeHold(object):
    def __init__(self):
        self.leftover = False

    def foreign_hold(self):
        return False

    def limit_stale(self):
        return False

    def blade_wait(self):
        return False

    def own_hold(self):
        return self.leftover

    def set_link_wait(self, on):
        pass


class Shell(unittest.TestCase):

    def make(self):
        s = ml.MowerLinkShell()
        s.core = ml.MowerLink()
        s.pubs = mock.Mock()
        s.hold = FakeHold()
        s.latch = mock.Mock()
        s.latch.running.return_value = True
        s.temp = mock.Mock(lock=threading.Lock(), core=None)
        s.raw = mock.Mock()
        s.tracked = mock.Mock()
        s.pub_hold = mock.Mock()
        s.pub_state = mock.Mock()
        s.pub_cmd = mock.Mock()
        s.pub_bat = mock.Mock()
        return s

    def test_m3_a_stuck_hold_is_taken_back(self):
        s = self.make()
        s.hold.leftover = True
        s._tick(None)
        sent = json.loads(s.pub_hold.publish.call_args[0][0].data)
        self.assertEqual((sent['source'], sent['hold']), ('mower_link', False))

    def test_h2_no_true_while_the_ladder_runs(self):
        s = self.make()
        under = mock.Mock()
        heard = ml._Heard(under, s.on_mission, s.ladder)
        s.core.phase = 'resync'
        heard.publish(Bool(True))
        under.publish.assert_not_called()
        self.assertTrue(s.core.commanded)
        heard.publish(Bool(False))
        under.publish.assert_called_once()

    def test_h2_a_gap_lets_it_through(self):
        s = self.make()
        under = mock.Mock()
        heard = ml._Heard(under, s.on_mission, s.ladder)
        s.core.phase = 'gap'
        heard.publish(Bool(True))
        under.publish.assert_called_once()

    def test_low_a_ladder_is_republished(self):
        s = self.make()
        s.core.phase = 'resync'
        s.core.since = 0.0
        s._publish({'phase': 'resync'})
        s._publish({'phase': 'resync'})
        self.assertEqual(s.pub_state.publish.call_count, 2)

    def test_low_a_stale_blade_debt_stops_the_links_true(self):
        s = self.make()
        s.stale = mock.Mock()
        s.stale.deferred.return_value = True
        s.core.tick = mock.Mock(return_value=[])
        s._tick(None)
        self.assertTrue(s.core.tick.call_args[0][3])


    def test_n3_the_state_goes_out_before_the_false(self):
        s = self.make()
        order = []
        s.pub_state.publish.side_effect = lambda m: order.append('state')
        s.raw.publish.side_effect = lambda m: order.append('false')
        s.core.tick = mock.Mock(return_value=['blade_off'])
        s._tick(None)
        self.assertEqual(order, ['state', 'false'])

    def test_n6_the_links_true_passes_stale_blades_tracker(self):
        s = self.make()
        s._do('blade_on')
        s.tracked.publish.assert_called_once()
        s.raw.publish.assert_not_called()

    def test_n6_stale_blade_knows_the_true(self):
        """Real wrappers: the mission's True kept back, the link's True heard."""
        s = self.make()
        heard, raw = [], mock.Mock()
        s.raw = raw
        s.tracked = sb._Tracked(raw, lambda v: heard.append(v) or False)
        s.core.phase = 'resync'
        ml._Heard(s.tracked, s.on_mission, s.ladder).publish(Bool(True))
        self.assertEqual(heard, [])                 # kept back, as it was
        s._do('blade_on')
        self.assertEqual(heard, [True])
        raw.publish.assert_called_once()

    def test_p1_alive_before_the_first_phase_and_after_failed(self):
        s = self.make()
        for state in ({'armed': True}, {'failed': True}):
            s.pub_state.publish.reset_mock()
            s._publish(state)
            s._publish(state)
            self.assertEqual(s.pub_state.publish.call_count, 2)
        s.pub_state.publish.reset_mock()
        s._publish({'armed': False})
        s._publish({'armed': False})
        self.assertEqual(s.pub_state.publish.call_count, 1)


class Second(unittest.TestCase):
    """Second review."""

    def test_n1_late_report_of_the_power_still_on(self):
        b = IncidentBoard()
        b.mow()
        b.w.on_bat_out(True)
        b.silent = True
        b.run(27.0)
        self.assertIn('bat_off', b.actions)
        b.w.on_bat_out(True)                        # sent before the cut, arrives after
        b.w.on_bat_out(False)
        b.run(1.0)
        self.assertNotIn('home_off', b.actions)
        self.assertTrue(b.w.active)

    def test_n1_own_cut_ends_with_set_power(self):
        w = ml.MowerLink()
        w._cut(1.0)
        w.bat_out = True
        w.on_bat_out(True)
        self.assertTrue(w.own_cut)
        w._reset(2.0)
        self.assertFalse(w.own_cut)
        w = ml.MowerLink()
        w._cut(1.0)
        w.on_power()
        self.assertFalse(w.own_cut)

    def test_n2_a_lost_false_is_sent_again_until_it_stands(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(3.5)
        self.assertEqual(b.w.phase, 'gap')
        b.w.on_command(False, b.t)                  # NAV_RECOVERY, into the silence
        b.actions = []
        b.silent = False                            # RUN at 3400: it never arrived
        b.run(3.0)
        self.assertEqual(b.w.phase, 'gap')          # the hold stands
        self.assertEqual(b.actions.count('blade_off'), 3)
        b.status = ('READY', 900)                   # coasting
        b.run(1.0)
        self.assertEqual(b.w.phase, 'gap')
        b.status = ('READY', 0)
        b.run(1.0)
        self.assertEqual(b.actions[-1], 'release')

    def test_n2_it_never_stops_the_attempt_ends_it(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(3.5)
        b.w.on_command(False, b.t)
        b.silent = False
        b.run(100.0)
        self.assertIn('home', b.actions)
        self.assertFalse(b.w.active)

    def test_n2_in_the_ladder_too(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(10.0)
        self.assertEqual(b.w.phase, 'resync')
        b.w.on_command(False, b.t)
        b.actions = []
        b.silent = False
        b.run(2.0)
        self.assertTrue(b.w.active)
        self.assertGreaterEqual(b.actions.count('blade_off'), 2)
        self.assertNotIn('release', b.actions)

    def test_n2_a_healthy_gap_is_as_it_was(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(3.5)
        b.silent = False
        b.run(1.0)
        self.assertEqual(b.actions, ['hold', 'release'])

    def test_n4_false_before_every_set_power(self):
        w = ml.MowerLink()
        self.assertEqual(w._reset(1.0), ['blade_off', 'set_power'])

    def test_n4_off_after_settle_sends_it_once_more(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(27.0)
        b.w.bat_out = False
        b.silent = False
        b.status = ('OFF', 0)
        b.run(20.0)
        self.assertEqual(b.actions.count('set_power'), 2)

    def test_n5_a_person_goes_home_with_its_own_reason(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(3.5)
        b.w.on_bat_out(True)
        b.w.on_bat_out(False)
        b.run(1.0)
        self.assertIn('home_off', b.actions)
        s = mock.Mock()
        shell = ml.MowerLinkShell()
        shell.core, shell.latch, shell.pubs = b.w, s, mock.Mock()
        shell._do('home_off')
        s.request.assert_called_once_with('mower_power_off')

    def test_n7_mission_ends_in_the_first_seconds_of_a_silence(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(1.5)
        b.running = False
        b.run(1.0)
        self.assertNotIn('bat_off', b.actions)      # 2.5 s: not yet a silence
        b.run(3.0)
        self.assertIn('bat_off', b.actions)         # repeated until the PM reads False
        self.assertEqual(b.actions[0], 'blade_off')

    def test_n7_status_goes_on_after_the_end_nothing_is_cut(self):
        b = Board()
        b.mow()
        b.running = False
        b.status = ('READY', 0)
        b.run(30.0)
        self.assertEqual(b.actions, [])

    def test_n7_a_new_mission_ends_the_watch(self):
        b = Board()
        b.mow()
        b.silent = True
        b.run(1.0)
        b.running = False
        b.run(0.5)
        self.assertIsNotNone(b.w.watch_seen)
        b.w.reset()
        self.assertIsNone(b.w.watch_seen)


class Latch(unittest.TestCase):

    def test_h2_blade_and_power_are_not_latched(self):
        from mower_unit_smach import publishers
        made = {}

        def fake(topic, kind, **kw):
            made[topic] = kw
            return mock.Mock()
        cls = next(v for k, v in vars(publishers).items() if isinstance(v, type)
                   and v.__module__ == publishers.__name__)
        with mock.patch.object(publishers.rospy, 'Publisher', fake):
            cls()
        self.assertFalse(made['/mower/set_motor_on'].get('latch'))
        self.assertFalse(made['/mower/set_power'].get('latch'))
        self.assertTrue(made['/mower/set_cut_height'].get('latch'))


class End(unittest.TestCase):

    def test_stop_ends_the_ladder_and_nothing_follows(self):
        b = IncidentBoard()
        b.mow()
        b.silent = True
        b.run(5.0)
        b.actions = []
        b.running = False
        b.run(200.0)
        self.assertIn('release', b.actions)
        self.assertNotIn('restart_serial', b.actions)
        self.assertNotIn('home', b.actions)

    def test_stop_with_the_power_off_leaves_it_off(self):
        b = IncidentBoard()
        b.mow()
        b.silent = True
        b.run(28.0)
        self.assertIn('bat_off', b.actions)
        self.assertNotIn('bat_on', b.actions)
        b.running = False
        b.run(20.0)
        self.assertNotIn('bat_on', b.actions)

    def test_safety_nodes_power_off_ends_it_and_is_not_answered(self):
        b = IncidentBoard()
        b.mow()
        b.silent = True
        b.run(5.0)
        b.w.on_power()                              # tilt: False on both topics
        b.w.on_command(False, b.t)
        b.silent = False
        b.status = ('OFF', 0)
        b.run(10.0)
        self.assertEqual(b.actions[-1], 'release')
        self.assertNotIn('set_power', b.actions)
        self.assertNotIn('blade_on', b.actions)

    def test_no_mission_no_trigger(self):
        b = Board()
        b.running = False
        b.w.on_command(True, b.t)
        b.silent = True
        b.run(60.0)
        self.assertEqual(b.actions, [])

    def test_a_blade_never_commanded_is_not_armed(self):
        b = Board()
        b.silent = True
        b.run(60.0)
        self.assertEqual(b.actions, [])


if __name__ == '__main__':
    unittest.main()
