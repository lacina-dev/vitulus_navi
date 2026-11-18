#!/usr/bin/env python3
"""T39 A739: the DockIdle node object around the lost-VO watch, offline (rospy
stubbed, a stand-in VO behind the service calls, a manual clock).

  - start: the first tick resumes the VO and puts its logger back (a previous
    instance may have died holding the pause); under the dock pause it waits;
  - the watch's pause raises the VO logger to ERROR, the probe puts it back,
    a shutdown while paused resumes and puts it back;
  - NO LOCK ACROSS A SERVICE CALL: while the watch's pause call hangs (service
    wait), the dock pause and the wake-up on undocking go through at once, and
    when the hung call finally lands the VO is resumed again;
  - a probe's resume landing after the dock pause is paused again;
  - switch off: no VO call of any kind; dry run: no call, status not paused.

Run:  python3 tools/test_t39_vo_lost_node.py
"""
import importlib.machinery
import importlib.util
import json
import os
import sys
import threading
import time as real_time
import unittest

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _ros_env  # noqa: E402
_ros_env.ensure()

_PATH = os.path.join(os.path.dirname(HERE), 'nodes', 'dock_idle')
_loader = importlib.machinery.SourceFileLoader('dock_idle_node2', _PATH)
_spec = importlib.util.spec_from_loader('dock_idle_node2', _loader)
di = importlib.util.module_from_spec(_spec)
_loader.exec_module(di)

from nav_msgs.msg import Odometry  # noqa: E402
from std_msgs.msg import Int8  # noqa: E402


class Clock(object):
    t = 5000.0

    @classmethod
    def time(cls):
        return cls.t


class World(object):
    """The VO and lidar odometry behind the services."""

    def __init__(self):
        self.calls = []                 # names, in the order they LANDED
        self.vo_paused = False
        self.licp_paused = False
        self.level = 'warn'
        self.hang = {}                  # service name -> Event the call waits for
        self.entered = {}               # service name -> Event set when a call waits
        self.status = None

    def call(self, name, *args):
        if name in self.hang:
            self.entered.setdefault(name, threading.Event()).set()
            self.hang[name].wait(10.0)
        self.calls.append(name if not args else (name,) + args)
        if name == '/vo/pause_odom':
            self.vo_paused = True
        elif name == '/vo/resume_odom':
            self.vo_paused = False
        elif name == '/licp/pause_odom':
            self.licp_paused = True
        elif name == '/licp/resume_odom':
            self.licp_paused = False
        elif name.endswith('set_logger_level'):
            self.level = args[1]


class FakeRospy(object):
    def __init__(self, world, params):
        self.world, self.params = world, params

    def get_param(self, name, default=None):
        return self.params.get(name.lstrip('~'), default)

    def Publisher(self, *a, **k):
        world = self.world

        class Pub(object):
            def publish(self, msg):
                world.status = json.loads(msg.data)
        return Pub()

    def Subscriber(self, *a, **k):
        return None

    def Timer(self, *a, **k):
        return None

    def Duration(self, s):
        return s

    def on_shutdown(self, fn):
        pass

    def wait_for_service(self, name, timeout=None):
        pass

    def ServiceProxy(self, name, _type):
        return lambda *args: self.world.call(name, *args)

    def loginfo(self, *a, **k):
        pass

    logwarn = logwarn_throttle = loginfo


def lost():
    m = Odometry()
    c = [0.0] * 36
    c[0] = c[35] = 9999.0
    m.twist.covariance = c
    return m


class Base(unittest.TestCase):
    PARAMS = {'vo_lost_pause': True, 'vo_lost_window_s': 5.0, 'vo_lost_min_msgs': 10,
              'vo_lost_retry_s': 10.0, 'vo_lost_probe_s': 4.0, 'dock_hold_s': 20.0}

    def make(self, **over):
        self.world = World()
        params = dict(self.PARAMS)
        params.update(over)
        di.rospy = FakeRospy(self.world, params)
        di.time = Clock
        Clock.t = 5000.0
        return di.DockIdle()

    def run_lost(self, node, seconds, tick=True):
        """lost VO at 10 Hz while it is not paused; vo_tick every second"""
        for i in range(int(seconds * 10)):
            Clock.t += 0.1
            if not self.world.vo_paused:
                node.cb_vo_odom(lost())
            if tick and (i + 1) % 10 == 0:
                node.vo_tick(None)

    def vo_calls(self):
        return [c for c in self.world.calls if not isinstance(c, tuple)]

    def levels(self):
        return [c[2] for c in self.world.calls if isinstance(c, tuple)]


class Startup(Base):
    def test_first_tick_resumes_and_restores_the_logger(self):
        node = self.make()
        self.world.vo_paused, self.world.level = True, 'error'   # left by a dead instance
        node.vo_tick(None)
        self.assertEqual(self.world.calls,
                         ['/vo/resume_odom',
                          ('/vo_rgbd_odometry/set_logger_level', 'ros.rtabmap_odom', 'warn')])
        self.assertFalse(self.world.vo_paused)
        node.vo_tick(None)
        self.assertEqual(len(self.world.calls), 2)            # once

    def test_failed_startup_resume_is_tried_again(self):
        node = self.make()
        fail = [True]
        real = di.rospy.wait_for_service

        def wait(name, timeout=None):
            if fail[0]:
                raise RuntimeError('not yet')
        di.rospy.wait_for_service = wait
        node.vo_tick(None)
        self.assertEqual(self.world.calls, [])
        fail[0] = False
        di.rospy.wait_for_service = real
        node.vo_tick(None)
        self.assertEqual(self.vo_calls(), ['/vo/resume_odom'])
        self.assertEqual(self.levels(), ['warn'])

    def test_silent_vo_gets_one_resume(self):
        node = self.make()
        node.vo_tick(None)                                    # the start-up one
        self.world.vo_paused = True                           # paused behind our back
        for _ in range(12):
            Clock.t += 1.0
            node.vo_tick(None)
        self.assertEqual(self.vo_calls(), ['/vo/resume_odom', '/vo/resume_odom'])
        self.assertFalse(self.world.vo_paused)

    def test_under_the_dock_pause_nothing_is_resumed(self):
        node = self.make()
        node.cb_dock(Int8(0))
        Clock.t += 21.0
        node.tick(None)                                       # dock pause
        self.assertTrue(self.world.vo_paused and self.world.licp_paused)
        n = len(self.vo_calls())
        for _ in range(30):
            Clock.t += 1.0
            node.vo_tick(None)
        self.assertEqual(len(self.vo_calls()), n)
        self.assertTrue(self.world.vo_paused)


class QuietLog(Base):
    def test_pause_quiets_probe_restores(self):
        node = self.make()
        node.vo_tick(None)
        del self.world.calls[:]
        self.run_lost(node, 6)
        self.assertEqual(self.world.calls,
                         ['/vo/pause_odom',
                          ('/vo_rgbd_odometry/set_logger_level', 'ros.rtabmap_odom', 'error')])
        self.assertEqual(self.world.status['vo_lost']['paused'], True)
        self.run_lost(node, 10)                               # retry -> probe
        self.assertEqual(self.world.calls[2], '/vo/resume_odom')
        self.assertEqual(self.levels()[:2], ['error', 'warn'])
        self.run_lost(node, 5)                                # still lost -> pause again
        self.assertEqual(self.vo_calls(), ['/vo/pause_odom', '/vo/resume_odom', '/vo/pause_odom'])
        self.assertEqual(self.levels(), ['error', 'warn', 'error'])
        self.assertEqual(self.world.level, 'error')

    def test_shutdown_while_paused(self):
        node = self.make()
        node.vo_tick(None)
        self.run_lost(node, 6)
        self.assertTrue(self.world.vo_paused)
        node.on_shutdown()
        self.assertFalse(self.world.vo_paused)
        self.assertEqual(self.world.level, 'warn')

    def test_quiet_log_off(self):
        node = self.make(vo_lost_quiet_log=False)
        node.vo_tick(None)
        self.run_lost(node, 6)
        self.assertEqual(self.vo_calls(), ['/vo/resume_odom', '/vo/pause_odom'])
        self.assertEqual(self.levels(), [])

    def test_dock_pause_over_a_watch_pause_puts_the_logger_back(self):
        node = self.make()
        node.vo_tick(None)
        self.run_lost(node, 6)
        self.assertEqual(self.world.level, 'error')
        node.cb_dock(Int8(0))
        Clock.t += 21.0
        node.tick(None)
        node.vo_tick(None)
        self.assertEqual(self.world.level, 'warn')            # only the WATCH's pause is quiet
        self.assertTrue(self.world.vo_paused)


class NoLockAcrossCalls(Base):
    def test_undocking_is_not_delayed_by_a_hung_watch_call(self):
        node = self.make()
        node.vo_tick(None)
        node.cb_dock(Int8(0))                                 # docked, hold not over yet
        self.run_lost(node, 4.9, tick=False)
        Clock.t += 0.2
        self.world.hang['/vo/pause_odom'] = threading.Event()
        th = threading.Thread(target=node.vo_tick, args=(None,))
        th.start()                                            # the watch's pause call hangs
        self.assertTrue(self.world.entered.setdefault(
            '/vo/pause_odom', threading.Event()).wait(5.0))
        hung = self.world.hang.pop('/vo/pause_odom')          # later calls go straight through
        # 1) messages are not blocked
        t0 = real_time.monotonic()
        node.cb_vo_odom(lost())
        # 2) the dock pause is not blocked
        Clock.t += 21.0
        node.tick(None)
        self.assertTrue(node.paused and self.world.licp_paused)
        # 3) the wake-up on undocking is not blocked
        node.cb_dock(Int8(1))
        dt = real_time.monotonic() - t0
        self.assertLess(dt, 1.0)
        self.assertEqual(self.world.calls[-2:], ['/vo/resume_odom', '/licp/resume_odom'])
        self.assertFalse(self.world.vo_paused or self.world.licp_paused)
        # 4) the hung pause lands AFTER the undock resume ...
        hung.set()
        th.join(5.0)
        self.assertFalse(th.is_alive())
        self.assertIn('/vo/pause_odom', self.world.calls[-4:])
        # ... and the VO is running again, not quiet, and nobody holds a pause
        self.assertFalse(self.world.vo_paused)
        self.assertFalse(self.world.licp_paused)
        Clock.t += 1.0
        node.vo_tick(None)
        self.assertEqual(self.world.level, 'warn')
        self.assertFalse(node.vo_paused or node.paused)

    def test_probe_resume_landing_after_the_dock_pause_is_paused_again(self):
        node = self.make()
        node.vo_tick(None)
        self.run_lost(node, 6)                                # watch pause
        self.assertTrue(self.world.vo_paused)
        node.cb_dock(Int8(0))
        Clock.t += 10.0                                       # retry time: the probe's resume ...
        self.world.hang['/vo/resume_odom'] = threading.Event()
        th = threading.Thread(target=node.vo_tick, args=(None,))
        th.start()
        self.assertTrue(self.world.entered.setdefault(
            '/vo/resume_odom', threading.Event()).wait(5.0))
        hung = self.world.hang.pop('/vo/resume_odom')
        Clock.t += 21.0
        node.tick(None)                                       # ... the dock pause meanwhile
        self.assertTrue(node.paused)
        hung.set()
        th.join(5.0)
        self.assertFalse(th.is_alive())
        self.assertTrue(self.world.vo_paused)                 # paused for the dock
        self.assertTrue(self.world.licp_paused)

    def test_undock_during_the_dock_pause_calls(self):
        node = self.make(vo_lost_pause=False)
        node.cb_dock(Int8(0))
        Clock.t += 21.0
        self.world.hang['/licp/pause_odom'] = threading.Event()
        th = threading.Thread(target=node.tick, args=(None,))
        th.start()
        self.assertTrue(self.world.entered.setdefault(
            '/licp/pause_odom', threading.Event()).wait(5.0))
        hung = self.world.hang.pop('/licp/pause_odom')
        node.cb_dock(Int8(1))                                 # undocking right now
        hung.set()
        th.join(5.0)
        self.assertFalse(self.world.vo_paused or self.world.licp_paused)
        self.assertFalse(node.paused)


class ResumeOnStart(Base):
    """A742: a restarted dock_idle owes one resume of both odometries."""
    BOTH = ['/vo/resume_odom', '/licp/resume_odom']

    def left_paused(self):
        self.world.vo_paused = self.world.licp_paused = True   # by a dead instance

    def test_not_docked_at_start_resumes_once(self):
        node = self.make(vo_lost_pause=False)
        self.left_paused()
        node.cb_dock(Int8(3))
        self.assertEqual(self.world.calls, self.BOTH)
        self.assertFalse(self.world.vo_paused or self.world.licp_paused)
        node.cb_dock(Int8(1))
        node.cb_dock(Int8(3))
        self.assertEqual(self.world.calls, self.BOTH)          # once

    def test_docked_at_start_calls_nothing_then_undock_within_hold_resumes(self):
        node = self.make(vo_lost_pause=False)
        self.left_paused()
        node.cb_dock(Int8(0))
        Clock.t += 5.0
        node.tick(None)
        self.assertEqual(self.world.calls, [])                 # in the dock: as before
        node.cb_dock(Int8(1))                                  # undocking 5 s after the restart
        self.assertEqual(self.world.calls, self.BOTH)
        self.assertFalse(self.world.vo_paused or self.world.licp_paused)

    def test_own_dock_pause_settles_the_debt(self):
        node = self.make(vo_lost_pause=False)
        node.cb_dock(Int8(0))
        Clock.t += 21.0
        node.tick(None)
        node.cb_dock(Int8(1))
        self.assertEqual(self.world.calls, ['/vo/pause_odom', '/licp/pause_odom'] + self.BOTH)
        node.cb_dock(Int8(3))
        self.assertEqual(len(self.world.calls), 4)

    def test_no_dock_status_resumes_after_the_timeout(self):
        node = self.make(vo_lost_pause=False)
        self.left_paused()
        Clock.t += 9.0
        node.tick(None)
        self.assertEqual(self.world.calls, [])
        Clock.t += 2.0
        node.tick(None)
        self.assertEqual(self.world.calls, self.BOTH)
        Clock.t += 20.0
        node.tick(None)
        node.cb_dock(Int8(3))
        self.assertEqual(self.world.calls, self.BOTH)          # once

    def test_switch_off_is_the_old_behaviour(self):
        node = self.make(vo_lost_pause=False, resume_on_start=False)
        self.left_paused()
        node.cb_dock(Int8(0))
        Clock.t += 5.0
        node.tick(None)
        node.cb_dock(Int8(1))
        Clock.t += 30.0
        node.tick(None)
        self.assertEqual(self.world.calls, [])                 # the hole: both stay paused
        self.assertTrue(self.world.vo_paused and self.world.licp_paused)

    def test_enable_false_stays_a_no_op(self):
        node = self.make(vo_lost_pause=False, enable=False)
        node.cb_dock(Int8(3))
        Clock.t += 30.0
        node.tick(None)
        self.assertEqual(self.world.calls, [])

    def test_with_the_lost_watch_no_second_startup_resume(self):
        node = self.make()
        node.cb_dock(Int8(3))
        node.vo_tick(None)
        self.assertEqual(self.vo_calls(), self.BOTH)
        self.assertEqual(self.levels(), ['warn'])


class Off(Base):
    def test_switch_off_no_vo_watch(self):
        node = self.make(vo_lost_pause=False)
        self.assertIsNone(node.vo_watch)
        node.cb_dock(Int8(0))
        Clock.t += 21.0
        node.tick(None)
        node.cb_dock(Int8(1))
        self.assertEqual(self.world.calls, ['/vo/pause_odom', '/licp/pause_odom',
                                            '/vo/resume_odom', '/licp/resume_odom'])
        self.assertEqual(self.world.status, {'enabled': True, 'paused': False})
        node.on_shutdown()
        self.assertEqual(len(self.world.calls), 4)

    def test_dry_run(self):
        node = self.make(vo_lost_dry_run=True)
        node.vo_tick(None)
        self.run_lost(node, 6)
        self.assertEqual(self.world.calls, [])
        st = self.world.status['vo_lost']
        self.assertEqual((st['paused'], st['dry_run'], st['pauses']), (False, True, 1))


if __name__ == '__main__':
    unittest.main(verbosity=1)
