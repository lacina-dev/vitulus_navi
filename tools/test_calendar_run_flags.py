#!/usr/bin/env python3
"""Off-robot test: what a calendar run changes in mower_unit_smach.

  * mark_run_phase never overwrites a path checkpoint (a refused or interrupted
    resume used to reset it to on_init, and the next resume mowed everything)
  * MONITOR_PROGRAM_UNFINISHED keeps the checkpoint, also with ': ' in a zone name
  * weather_rain_mode: who obeys the rain; ignore_rain_once lifts it for one run
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
from std_msgs.msg import Bool, String
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
def bypass(mode, scheduled):
    PARAMS['~weather_rain_mode'] = mode
    helpers.run_flags.scheduled = scheduled
    return helpers.get_weather_config()['bypass']


check('default: manual obeys', helpers.get_weather_config()['bypass'] is False)
check('always: manual obeys', bypass('always', False) is False)
check('always: calendar obeys', bypass('always', True) is False)
check('calendar: manual does not', bypass('calendar', False) is True)
check('calendar: calendar obeys', bypass('calendar', True) is False)
check('ignore: nobody', bypass('ignore', True) is True and bypass('ignore', False) is True)

flags = helpers.run_flags
PARAMS['~weather_rain_mode'] = 'always'
flags.scheduled = False
flags._on_ignore_rain(Bool(True))
check('once lifts it', helpers.get_weather_config()['bypass'] is True)
flags.scheduled = True
check('once never reaches a calendar run', helpers.get_weather_config()['bypass'] is False)
flags.scheduled = False
flags.mission_busy(True)
flags._ignore_rain_t -= 1000.0
check('once holds during the mission', flags.rain_ignored() is True)
flags.mission_busy(False)
check('once ends with the mission', flags.rain_ignored() is False)
flags._on_ignore_rain(Bool(True))
flags._ignore_rain_t -= 1000.0
check('once unused expires', flags.rain_ignored() is False)
flags._on_ignore_rain(Bool(True))
flags._on_ignore_rain(Bool(False))
check('once disarmed', flags.rain_ignored() is False)
PARAMS['~weather_rain_mode'] = 'always'


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
PARAMS['~weather_rain_mode'] = 'calendar'
helpers.run_flags.scheduled = False
check('manual + calendar mode ignores rain', monitors.weather_monitor_cb(None, rain(now=1)) is True)
PARAMS['~weather_rain_mode'] = 'always'
PARAMS['~weather_return_home'] = False
check('rain_return_home off: the run goes on', monitors.weather_monitor_cb(None, rain(now=1)) is True)
PARAMS['~weather_return_home'] = True

# --- the rule itself, as the mission and master_controller use it -----------
from mower_unit_smach import rain_rule  # noqa: E402


def verdict(recent=60, steps=3, **fields):
    m = RainAlert()
    m.status_nowcast10m = m.status_nowcast20m = m.status_nowcast30m = 'OK'
    for k, v in fields.items():
        setattr(m, k, v)
    return rain_rule.start_verdict(m, {'recent_min': recent, 'n_steps': steps,
                                       'reject_statuses': ('RAIN',)})


check('dry', verdict() is None)
check('raining', verdict(rain_now=1) == 'rain')
check('status now', verdict(status_now='RAIN') == 'recent_rain')
check('60 min back counts', verdict(status_past60m='RAIN') == 'recent_rain')
check('recent 30: 60 back does not', verdict(30, status_past60m='RAIN') is None)
check('recent 30: 30 back does', verdict(30, status_past30m='RAIN') == 'recent_rain')
check('recent 0: 10 back does not', verdict(0, status_past10m='RAIN') is None)
check('forecast', verdict(status_nowcast30m='RAIN') == 'forecast')
check('horizon 10 ends at 10', verdict(steps=1, status_nowcast30m='RAIN') is None)


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
