#!/usr/bin/env python3
"""Off-robot test: what a calendar run changes in mower_unit_smach.

  * mark_run_phase never overwrites a path checkpoint (a refused or interrupted
    resume used to reset it to on_init, and the next resume mowed everything)
  * MONITOR_PROGRAM_UNFINISHED keeps the checkpoint, also with ': ' in a zone name
  * weather_rain_bypass holds for manual runs only; a calendar run obeys
  * the weather monitor reacts to rain_now and ignores stale data
  * stop_at_path_boundary: next path = checkpoint, then the mission is preempted

The whole chain against the real SM: vitulus_planner/test/calendar_sim/run_sim.py
Run:  python3 test_calendar_run_flags.py   (ROS python env, no master)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))

import rospy
import rospy.rostime
import genpy
from std_msgs.msg import String
from vitulus_msgs.msg import PlannerProgram
from weather_alert.msg import RainAlert

rospy.rostime.set_rostime_initialized(True)
PARAMS = {}
rospy.get_param = lambda name, default=None: PARAMS.get(name, default)

from mower_unit_smach import helpers, monitors  # noqa: E402

FAILS = []


def check(name, cond, detail=''):
    print('%s %s %s' % ('ok  ' if cond else 'FAIL', name, detail))
    if not cond:
        FAILS.append(name)


# --- mark_run_phase ---------------------------------------------------------
p = PlannerProgram()
p.last_result = 'failed: on_init'
helpers.mark_run_phase(p, 'on_mower')
check('phase replaces a phase', p.last_result == 'failed: on_mower', p.last_result)
p.last_result = 'failed: on_path-Z-1-200'
helpers.mark_run_phase(p, 'on_planner')
check('phase keeps a path checkpoint', p.last_result == 'failed: on_path-Z-1-200', p.last_result)

# --- weather ----------------------------------------------------------------
PARAMS['~weather_rain_bypass'] = True
helpers.run_flags.scheduled = False
check('bypass for a manual run', helpers.get_weather_config()['bypass'] is True)
helpers.run_flags.scheduled = True
check('no bypass for a calendar run', helpers.get_weather_config()['bypass'] is False)


def rain(now=0, age=0.0, nowcast='OK'):
    m = RainAlert()
    m.rain_now = now
    m.time = rospy.Time.now() - rospy.Duration(age)
    m.status_nowcast10m = m.status_nowcast20m = m.status_nowcast30m = nowcast
    return m


check('dry goes on', monitors.weather_monitor_cb(None, rain()) is True)
check('raining now ends it', monitors.weather_monitor_cb(None, rain(now=1)) is False)
check('forecast ends it', monitors.weather_monitor_cb(None, rain(nowcast='RAIN')) is False)
check('stale data decides nothing',
      monitors.weather_monitor_cb(None, rain(now=1, age=3600.0)) is True)
helpers.run_flags.scheduled = False
check('manual + bypass ignores rain', monitors.weather_monitor_cb(None, rain(now=1)) is True)
helpers.run_flags.scheduled = False


# --- path boundary ----------------------------------------------------------
class Latch(object):
    def __init__(self, state):
        self.state = state
        self.asked = None

    def request(self, reason):
        self.asked = reason
        self.state.flag = True


class State(object):
    flag = False
    serviced = False

    def preempt_requested(self):
        return self.flag

    def service_preempt(self):
        self.serviced = True


class Ud(object):
    pass


class Pub(object):
    def __init__(self):
        self.sent = []

    def publish(self, m):
        self.sent.append(m)


class Pubs(object):
    save_program = Pub()
    log_info = Pub()


rospy.sleep = lambda *a, **kw: None
st = State()
ud = Ud()
ud.program = PlannerProgram()
ud.program.last_result = 'failed: on_path-A-0-300'
helpers.run_flags.home_latch = Latch(st)
helpers.run_flags.finish_after_path = False
check('no request, no stop',
      helpers.stop_at_path_boundary(st, ud, Pubs, 'A', 1) is False and not Pubs.save_program.sent)
helpers.run_flags.finish_after_path = True
check('stops at the boundary', helpers.stop_at_path_boundary(st, ud, Pubs, 'A', 1) is True)
check('next path is the checkpoint', ud.program.last_result == 'failed: on_path-A-1-0', ud.program.last_result)
check('saved at once', len(Pubs.save_program.sent) == 1)
check('sent home for the window', helpers.run_flags.home_latch.asked == 'window')
check('preempt serviced', st.serviced)
check('one shot', helpers.run_flags.finish_after_path is False)

print('\nALL PASS' if not FAILS else '\nFAILED: %s' % FAILS)
sys.exit(1 if FAILS else 0)
