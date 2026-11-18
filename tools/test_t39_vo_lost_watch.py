#!/usr/bin/env python3
"""T39: the lost-VO watch of nodes/dock_idle (~vo_lost_pause), offline.

  - which /vo/odom message counts as usable (9999 = lost / first after reset);
  - a VO lost all night is paused after one full window, tried again every
    retry_s for probe_s, and paused again while it stays lost;
  - a VO that tracks (day) or is lost only now and then (dawn) is never paused;
  - a probe that tracks keeps the VO running, and a later loss pauses it again;
  - no messages at all (camera dead, VO down, paused by a dock_idle that died)
    pauses nothing and asks for ONE resume per silence;
  - messages arriving while "paused" (someone resumed it, the VO respawned)
    put the watch back to plain watching;
  - the dock pause restarts the watch (reset).
No ROS master; rospy is only imported by the node file.

Run:  python3 tools/test_t39_vo_lost_watch.py
"""
import importlib.machinery
import importlib.util
import os
import sys
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _ros_env  # noqa: E402
_ros_env.ensure()

_PATH = os.path.join(os.path.dirname(HERE), 'nodes', 'dock_idle')
_loader = importlib.machinery.SourceFileLoader('dock_idle_node', _PATH)
_spec = importlib.util.spec_from_loader('dock_idle_node', _loader)
di = importlib.util.module_from_spec(_spec)
_loader.exec_module(di)

BAD = [9999.0] + [0.0] * 6 + [9999.0] + [0.0] * 27 + [9999.0]
GOOD = [4e-4] + [0.0] * 34 + [4.4e-4]


class Sim(object):
    """Feeds the watch like the node does: messages at `hz` while the VO is
    'running', step() once a second, actions applied to the fake VO."""

    def __init__(self, **kw):
        self.w = di.VoLostWatch(**kw)
        self.k = 0                 # ticks of 1/hz: no float drift in the clock
        self.t = 1000.0
        self.w.reset(self.t)
        self.running = True
        self.actions = []          # (t - 1000, action)

    def run(self, seconds, good, hz=10, publish=True):
        """good: bool or callable(t) -> bool (one hz per Sim)"""
        steps = int(round(seconds * hz))
        for i in range(steps):
            self.k += 1
            self.t = 1000.0 + self.k / float(hz)
            if self.running and publish:
                g = good(self.t) if callable(good) else good
                self.w.on_msg(self.t, g)
            if self.k % hz == 0:
                a = self.w.step(self.t)
                if a:
                    self.actions.append((round(self.t - 1000.0), a))
                    self.running = (a == 'resume')


class Usable(unittest.TestCase):
    def test_lost_and_good(self):
        self.assertFalse(di.vo_twist_usable(BAD, 1000.0))
        self.assertTrue(di.vo_twist_usable(GOOD, 1000.0))

    def test_one_bad_axis_is_enough(self):
        c = list(GOOD)
        c[35] = 9999.0
        self.assertFalse(di.vo_twist_usable(c, 1000.0))

    def test_zero_nan_short(self):
        self.assertFalse(di.vo_twist_usable([0.0] * 36, 1000.0))
        c = list(GOOD)
        c[0] = float('nan')
        self.assertFalse(di.vo_twist_usable(c, 1000.0))
        self.assertFalse(di.vo_twist_usable([], 1000.0))


class Night(unittest.TestCase):
    def test_pause_probe_pause(self):
        s = Sim()
        s.run(400, good=False)
        acts = s.actions
        self.assertEqual(acts[0], (30, 'pause'))
        self.assertEqual(acts[1], (120, 'resume'))
        self.assertEqual(acts[2], (128, 'pause'))
        self.assertEqual(acts[3], (218, 'resume'))
        self.assertEqual(acts[4], (226, 'pause'))
        # duty: the VO runs 8 s of every 98 s
        self.assertEqual([a for _t, a in acts][:7],
                         ['pause', 'resume', 'pause', 'resume', 'pause', 'resume', 'pause'])

    def test_few_good_frames_do_not_save_it(self):
        s = Sim()
        n = [0]

        def good(_t):
            n[0] += 1
            return n[0] % 10 == 0          # 10 % usable
        s.run(60, good=good)
        self.assertEqual(s.actions[0][1], 'pause')

    def test_no_pause_before_a_full_window(self):
        s = Sim()
        s.run(29, good=False)
        self.assertEqual(s.actions, [])

    def test_too_few_messages(self):
        s = Sim(min_msgs=30)
        s.run(120, good=False, hz=1)          # 30 msgs in 30 s -> pause
        self.assertEqual(s.actions[0][1], 'pause')
        s = Sim(min_msgs=40)
        s.run(120, good=False, hz=1)          # never 40 in the window
        self.assertEqual(s.actions, [])


class Day(unittest.TestCase):
    def test_tracking_never_paused(self):
        s = Sim()
        s.run(600, good=True)
        self.assertEqual(s.actions, [])

    def test_dawn_half_lost_never_paused(self):
        s = Sim()
        n = [0]

        def good(_t):
            n[0] += 1
            return n[0] % 2 == 0
        s.run(600, good=good)
        self.assertEqual(s.actions, [])

    def test_silence_pauses_nothing_and_asks_one_resume(self):
        """A VO silent for a whole window may be paused by a dock_idle that
        died: one resume (safe direction), never a pause, never a second one."""
        s = Sim()
        s.run(600, good=False, publish=False)
        self.assertEqual(s.actions, [(30, 'resume')])
        self.assertEqual(s.w.state, 'run')

    def test_silence_resume_is_rearmed_by_a_message(self):
        w = di.VoLostWatch()
        w.reset(0.0)
        self.assertIsNone(w.step(29.0))
        self.assertEqual(w.step(30.0), 'resume')
        self.assertIsNone(w.step(100.0))
        w.on_msg(101.0, True)                  # it came back ...
        self.assertIsNone(w.step(130.0))
        self.assertEqual(w.step(131.0), 'resume')   # ... and went silent again

    def test_no_silence_resume_while_the_watch_holds_the_pause(self):
        s = Sim()
        s.run(100, good=False)                 # paused at 30, VO silent since
        self.assertEqual(s.actions, [(30, 'pause')])


class Recovery(unittest.TestCase):
    def test_probe_that_tracks_keeps_running(self):
        s = Sim()
        s.run(100, good=False)                 # paused at 30
        self.assertEqual(s.actions, [(30, 'pause')])
        s.run(300, good=True)                  # dawn: probe at 120 succeeds
        self.assertEqual(s.actions, [(30, 'pause'), (120, 'resume')])
        self.assertEqual(s.w.state, 'run')
        self.assertEqual(s.w.probes_ok, 1)
        self.assertTrue(s.running)

    def test_lost_again_after_recovery(self):
        s = Sim()
        s.run(100, good=False)
        s.run(60, good=True)                   # probe ok at 128
        s.run(100, good=False)
        self.assertEqual(s.actions[-1][1], 'pause')
        # not sooner than one full window after the probe ended
        self.assertGreaterEqual(s.actions[-1][0], 128 + 30)

    def test_probe_ignores_the_first_frames(self):
        w = di.VoLostWatch(probe_s=8.0)
        w.reset(0.0)
        w._to(w.PROBE, 100.0)
        w.on_msg(100.5, False)                 # init frames right after resume
        w.on_msg(101.0, False)
        for i in range(20):
            w.on_msg(102.5 + i * 0.2, True)
        self.assertIsNone(w.step(108.0))
        self.assertEqual(w.state, 'run')

    def test_probe_without_messages_pauses_again(self):
        w = di.VoLostWatch()
        w.reset(0.0)
        w._to(w.PROBE, 100.0)
        self.assertIsNone(w.step(107.0))
        self.assertEqual(w.step(108.0), 'pause')


class External(unittest.TestCase):
    def test_resumed_by_someone_else(self):
        s = Sim()
        s.run(40, good=False)
        self.assertEqual(s.w.state, 'paused')
        s.running = True                       # VO respawned / resumed elsewhere
        s.run(5, good=False)
        self.assertEqual(s.w.state, 'run')
        self.assertEqual(s.actions, [(30, 'pause')])
        s.run(40, good=False)                  # still lost -> paused again
        self.assertEqual(s.actions[-1][1], 'pause')

    def test_in_flight_messages_right_after_pause_do_not_count(self):
        w = di.VoLostWatch()
        w.reset(0.0)
        w._to(w.PAUSED, 100.0)
        for i in range(10):
            w.on_msg(100.1 + i * 0.1, False)
        self.assertIsNone(w.step(101.0))
        self.assertEqual(w.state, 'paused')

    def test_dock_reset(self):
        s = Sim()
        s.run(40, good=False)
        s.w.reset(s.t)                         # the dock pause took over
        self.assertEqual(s.w.state, 'run')
        s.running = True
        s.run(29, good=False)
        self.assertEqual(len(s.actions), 1)    # needs a full window again


class Status(unittest.TestCase):
    def test_json_fields(self):
        s = Sim()
        s.run(40, good=False)
        st = s.w.status()
        self.assertEqual(st['state'], 'paused')
        self.assertEqual(st['pauses'], 1)
        self.assertEqual(st['good_frac'], 0.0)


if __name__ == '__main__':
    unittest.main(verbosity=1)
