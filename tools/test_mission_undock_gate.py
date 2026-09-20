#!/usr/bin/env python3
"""Off-robot test of the mission undock gate (mower_unit_smach).

Builds the REAL mission state machine (smach) against a FAKE ROS world: no
master, no init_node, simulated clock. Topics are scripted per scenario.

Covers
  * docked + no UNDOCK program     -> fast 'rejected', LOAD_MAP never entered
  * undock Failed (dock_status 4)  -> fast 'aborted' UNDOCK_FAILED, no LOAD_MAP
  * undock never starts (stays 0)  -> 'aborted' UNDOCK_FAILED, no LOAD_MAP
  * UNDOCK program runs as docking -> 'aborted' + dock_smach cancelled
  * undock hangs                   -> 'aborted' UNDOCK_TIMEOUT + cancel
  * no dock_status, charger ONLINE / unknown -> 'rejected'
  * no dock_status, charger offline -> LOAD_MAP (old behaviour)
  * normal undock (0 -> 1 -> 3)    -> LOAD_MAP (unchanged)
  * already undocked (3), motors ON -> LOAD_MAP (unchanged)
  * already undocked (3), motors OFF / no motor state -> fast 'rejected',
    and the mission NEVER publishes /base/motor_power
  * site-native program ('SITE'): planner_loaded == served site -> continues,
    no legacy /navi_manager/load_map request; no served site -> 'rejected'
    BEFORE undocking; site un-served meanwhile -> 'rejected' in WAIT_FOR_PLANNER
  * legacy program (name***env*ENV): planner_loaded == map_name, unchanged
  * MISSION_CONCURRENCE maps 'rejected' -> 'mission_rejected'
  * CRITICAL_ERROR routes UNDOCK_* to terminal (no docking attempt)
  * STOPPED / TERMINAL_ERROR report a Run request instead of dropping it
  * WaitForTopic publishes periodic "Waiting for ..." status

Run:  python3 test_mission_undock_gate.py   (needs the ROS python env: rospy,
      smach, vitulus_msgs ... importable; does NOT need a ROS master)
"""
import os
import sys
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, '..', 'src'))

import rospy
import smach
import actionlib
import json
from std_msgs.msg import Int8, Bool, String
from vitulus_msgs.msg import DockProgram, PlannerProgram, Power_status, Navi_transform


# ---------------------------------------------------------------------------
# Fake ROS world
# ---------------------------------------------------------------------------

class World(object):
    """Simulated clock + scripted topics. script[topic] = f(t) -> msg | None."""

    def __init__(self):
        self.t = 100.0
        self.t0 = self.t
        self.script = {}
        self.subs = []          # (topic, cb)
        self.published = []     # (topic, msg)
        self.timers = []        # [t_fire, cb]
        self.params = {}        # rosparam overrides
        self.dead_after = {}    # topic -> t: publisher gone (0 connections) from t on
        self.hooks = []         # f(t) called on every clock advance
        self.lock = threading.RLock()

    def elapsed(self):
        return self.t - self.t0

    def value(self, topic):
        f = self.script.get(topic)
        return f(self.elapsed()) if f else None

    def advance(self, dt):
        with self.lock:
            self.t += dt
            for hook in list(self.hooks):
                hook(self.elapsed())
            for topic, cb in list(self.subs):
                msg = self.value(topic)
                if msg is not None:
                    cb(msg)
            for timer in list(self.timers):
                if self.t >= timer[0]:
                    self.timers.remove(timer)
                    timer[1](None)

    def texts(self, topic):
        return [m.data for (t, m) in self.published if t == topic]


W = World()
EVER_PUBLISHED = set()


class FakePub(object):
    def __init__(self, topic, *a, **kw):
        self.topic = topic

    def publish(self, msg):
        W.published.append((self.topic, msg))
        EVER_PUBLISHED.add(self.topic)


class FakeSub(object):
    def __init__(self, topic, msg_type, cb=None, callback_args=None, **kw):
        if cb is not None and callback_args is not None:
            real_cb = cb
            cb = lambda msg: real_cb(msg, callback_args)          # noqa: E731
        self.entry = (topic, cb)
        if cb is not None:
            W.subs.append(self.entry)

    def unregister(self):
        if self.entry in W.subs:
            W.subs.remove(self.entry)

    def get_num_connections(self):
        dead = W.dead_after.get(self.entry[0])
        return 0 if (dead is not None and W.elapsed() >= dead) else 1


class FakeTimer(object):
    def __init__(self, period, cb, oneshot=False):
        W.timers.append([W.t + period.to_sec(), cb])

    def shutdown(self):
        pass


def fake_wait_for_message(topic, msg_type, timeout=None):
    msg = W.value(topic)
    if msg is None:
        W.advance(timeout if timeout else 1.0)
        raise rospy.ROSException('timeout')
    W.advance(0.05)
    return msg


class FakeActionClient(object):
    def __init__(self, *a, **kw):
        pass

    def wait_for_server(self, *a, **kw):
        return True


rospy.Publisher = FakePub
rospy.Subscriber = FakeSub
rospy.ServiceProxy = lambda *a, **kw: None
rospy.wait_for_message = fake_wait_for_message
rospy.get_param = lambda name, default=None: W.params.get(name, default)
rospy.Timer = FakeTimer
rospy.sleep = lambda d: W.advance(d.to_sec() if hasattr(d, 'to_sec') else float(d))
rospy.is_shutdown = lambda: False
rospy.Time.now = staticmethod(lambda: rospy.Time.from_sec(W.t))
rospy.get_rostime = lambda: rospy.Time.from_sec(W.t)
for _name in ('logdebug', 'loginfo', 'logwarn', 'logerr', 'loginfo_throttle'):
    setattr(rospy, _name, lambda *a, **kw: None)
actionlib.SimpleActionClient = FakeActionClient
smach.set_loggers(lambda m: None, lambda m: None, lambda m: None, lambda m: None)

from mower_unit_smach.publishers import MowerPublishers          # noqa: E402
from mower_unit_smach import mission, top_level, states          # noqa: E402


def status_script(steps):
    """steps = [(t_from, value), ...] -> dock_status as a function of time."""
    def f(t):
        cur = None
        for t_from, v in steps:
            if t >= t_from:
                cur = v
        return None if cur is None else Int8(data=cur)
    return f


MAPSTAT = '/mapping_manager/status'
PLANNER = '/web_plan/planner_loaded'
GPS = '/nav_tf/odom_status'


def serving(site):
    return String(data=json.dumps({'serving': {'site': site} if site else None}))


def run_mission(script, map_name='SITE', params=None, dead_after=None, preempt_in=None):
    """Run MISSION_CHILD to its end. Past LOAD_MAP nothing is scripted, so a
    run that gets there ends in the (simulated) 600 s GPS-fix timeout."""
    global W
    W.__init__()
    W.script = dict(script)
    W.params = dict(params or {})
    W.dead_after = dict(dead_after or {})
    if map_name == 'SITE':
        W.script.setdefault(MAPSTAT, lambda t: serving('Nmap'))   # a site is served
    pubs = MowerPublishers()
    sm = mission.build_mission_child_sm(pubs)
    sm.userdata.program = PlannerProgram(name='T1', map_name=map_name)
    visited = []

    sm.register_transition_cb(lambda ud, active: visited.extend(active), cb_args=[])
    sm.register_start_cb(lambda ud, init: visited.extend(init), cb_args=[])
    if preempt_in is not None:
        # What a firing monitor does: preempt the child while <state> runs.
        def hook(t, fired=[False]):
            if not fired[0] and sm.is_running() and preempt_in in sm.get_active_states():
                fired[0] = True
                sm.request_preempt()
        W.hooks.append(hook)
    outcome = sm.execute()
    LAST['sm'] = sm
    return outcome, visited, sm.userdata.error_reason, W.elapsed()


FAILS = []
LAST = {}


def check(name, cond, detail=''):
    print('%-4s %s %s' % ('ok' if cond else 'FAIL', name, '' if cond else detail))
    if not cond:
        FAILS.append(name)


DOCK = '/dock_smach/dock_status'
UNDOCK_PRG = '/dock_manager/undock_program'
PM = '/pm/power_status'
MOTORS = '/base/motor_power_state'
MOTORS_ON = lambda t: Bool(data=True)                             # noqa: E731
MOTORS_OFF = lambda t: Bool(data=False)                           # noqa: E731
PRG = lambda t: DockProgram()                                     # noqa: E731


def test_mission():
    # 1. docked, no undock program (P13, fresh install)
    out, vis, reason, dur = run_mission({DOCK: status_script([(0, 0)])})
    check('no undock program -> rejected', out == 'rejected', out)
    check('no undock program: LOAD_MAP never entered', 'LOAD_MAP' not in vis, vis)
    check('no undock program: fast (<30 s)', dur < 30.0, '%.1fs' % dur)
    check('no undock program: user message',
          any('no UNDOCK program saved' in s for s in W.texts('/nextion/log_info')))
    check('no undock program: stop_reason',
          'rejected:no_undock_program' in W.texts('/mower_smach/stop_reason'))
    check('no undock program: rejection does not block the SM', dur < 12.0, '%.1fs' % dur)
    check('no undock program: status shows the rejection first',
          W.texts('/mower_smach/status')[-1] == 'Rejected: no undock program')
    W.advance(6.0)
    check('no undock program: status back to Ready after the hold',
          W.texts('/mower_smach/status')[-1] == 'Ready')

    # 2. undock fails: 0 -> 1 -> 4 (1 s) -> 0
    out, vis, reason, dur = run_mission({
        DOCK: status_script([(0, 0), (2, 1), (20, 4), (21, 0)]), UNDOCK_PRG: PRG})
    check('undock failed -> aborted', out == 'aborted', out)
    check('undock failed: reason', reason == 'UNDOCK_FAILED', reason)
    check('undock failed: LOAD_MAP never entered', 'LOAD_MAP' not in vis, vis)
    check('undock failed: fast (<30 s)', dur < 30.0, '%.1fs' % dur)
    check('undock failed: user message',
          'Undocking failed' in W.texts('/nextion/log_info'))

    # 2b. failed, but robot ends OFF the charger (4 then 3): still a failure
    out, vis, reason, dur = run_mission({
        DOCK: status_script([(0, 0), (2, 1), (20, 4), (21, 3)]), UNDOCK_PRG: PRG})
    check('undock failed then off-charger -> aborted',
          out == 'aborted' and 'LOAD_MAP' not in vis, (out, vis))

    # 3. dock_smach never starts the program (stays docked)
    out, vis, reason, dur = run_mission({DOCK: status_script([(0, 0)]), UNDOCK_PRG: PRG})
    check('undock never starts -> aborted', out == 'aborted' and reason == 'UNDOCK_FAILED',
          (out, reason))
    check('undock never starts: LOAD_MAP never entered', 'LOAD_MAP' not in vis, vis)
    check('undock never starts: fast (<30 s)', dur < 30.0, '%.1fs' % dur)

    # 4. saved UNDOCK program is evaluated as DOCKING by dock_smach (P17)
    out, vis, reason, dur = run_mission({
        DOCK: status_script([(0, 0), (1, 2)]), UNDOCK_PRG: PRG})
    check('undock runs as docking -> aborted', out == 'aborted' and 'LOAD_MAP' not in vis,
          (out, vis))
    check('undock runs as docking: dock_smach cancelled',
          any(t == '/dock_smach/stop' and m.data for t, m in W.published))

    # 5. undock hangs in 'undocking'
    out, vis, reason, dur = run_mission({
        DOCK: status_script([(0, 0), (1, 1)]), UNDOCK_PRG: PRG})
    check('undock hangs -> aborted UNDOCK_TIMEOUT',
          out == 'aborted' and reason == 'UNDOCK_TIMEOUT', (out, reason))
    check('undock hangs: LOAD_MAP never entered', 'LOAD_MAP' not in vis, vis)
    check('undock hangs: dock_smach cancelled',
          any(t == '/dock_smach/stop' and m.data for t, m in W.published))
    check('undock hangs: progress shown',
          any(s.startswith('Undocking (') for s in W.texts('/mower_smach/status')))

    # 6. no dock_status at all
    out, vis, reason, dur = run_mission({PM: lambda t: Power_status(supply_status='ONLINE')})
    check('no dock_status + charger ONLINE -> rejected',
          out == 'rejected' and 'LOAD_MAP' not in vis, (out, vis))
    out, vis, reason, dur = run_mission({})
    check('no dock_status + no power status -> rejected',
          out == 'rejected' and 'LOAD_MAP' not in vis, (out, vis))
    out, vis, reason, dur = run_mission({PM: lambda t: Power_status(supply_status='OFFLINE'),
                                         MOTORS: MOTORS_ON})
    check('no dock_status + charger offline -> LOAD_MAP (as before)', 'LOAD_MAP' in vis, vis)

    # 6b. dock state unknown / busy forever
    out, vis, reason, dur = run_mission({DOCK: status_script([(0, 5)])})
    check('dock status unknown forever -> rejected',
          out == 'rejected' and 'LOAD_MAP' not in vis, (out, vis))

    # 7. normal paths unchanged
    out, vis, reason, dur = run_mission({
        DOCK: status_script([(0, 0), (2, 1), (40, 3)]), UNDOCK_PRG: PRG})
    check('normal undock -> LOAD_MAP', 'LOAD_MAP' in vis and 'WAIT_FOR_UNDOCKED' in vis, vis)
    check('normal undock: undock program sent',
          any(t == '/dock_smach/start_docking' for t, m in W.published))
    check('normal undock: dock_smach NOT cancelled',
          not any(t == '/dock_smach/stop' for t, m in W.published))
    out, vis, reason, dur = run_mission({DOCK: status_script([(0, 3)]), MOTORS: MOTORS_ON})
    check('already undocked, motors on -> LOAD_MAP, no undock',
          'LOAD_MAP' in vis and 'GET_UNDOCK_PROGRAM' not in vis, vis)

    # 8. started off the dock with motors OFF (boot default) / no motor state
    for label, script, text, why in (
            ('motors OFF', {DOCK: status_script([(0, 3)]), MOTORS: MOTORS_OFF},
             'motors are OFF', 'rejected:motors_off'),
            ('no motor state', {DOCK: status_script([(0, 3)])},
             'drive base not ready', 'rejected:base_not_ready')):
        out, vis, reason, dur = run_mission(script)
        check('off dock, %s -> rejected' % label,
              out == 'rejected' and 'LOAD_MAP' not in vis, (out, vis))
        check('off dock, %s: fast (<10 s)' % label, dur < 10.0, '%.1fs' % dur)
        check('off dock, %s: user message' % label,
              any(text in s for s in W.texts('/nextion/log_info')), W.texts('/nextion/log_info'))
        check('off dock, %s: stop_reason' % label,
              why in W.texts('/mower_smach/stop_reason'))
    out, vis, reason, dur = run_mission({DOCK: status_script([(0, 3)]), MOTORS: MOTORS_OFF},
                                        params={'~require_motor_power_state': False})
    check('~require_motor_power_state:=false disables the gate', 'LOAD_MAP' in vis, vis)
    check('mission never publishes /base/motor_power',
          '/base/motor_power' not in EVER_PUBLISHED, sorted(EVER_PUBLISHED))


def test_site_native_planner():
    off_dock = {DOCK: status_script([(0, 3)]), MOTORS: MOTORS_ON,
                GPS: lambda t: Navi_transform(status='SAT', info='0.5s,0.5s')}

    # site-native: planner_loaded carries the SERVED SITE, program says 'SITE'
    out, vis, reason, dur = run_mission(dict(off_dock, **{PLANNER: lambda t: String(data='Nmap')}))
    check('site-native: planner_loaded == served site -> continues',
          'SET_PROGRAM_SPEED' in vis, vis)
    check('site-native: fast (no 120 s planner wait)', dur < 60.0, '%.1fs' % dur)
    check('site-native: no legacy load_map request',
          not any(t.startswith('/navi_manager/load_map') for t, m in W.published))

    # planner still holds another site -> not accepted
    out, vis, reason, dur = run_mission(dict(off_dock, **{PLANNER: lambda t: String(data='Other')}))
    check('site-native: planner on another site -> timeout, aborted',
          out == 'aborted' and 'SET_PROGRAM_SPEED' not in vis, (out, vis))

    # no site served, robot docked: refused BEFORE undocking
    for label, stat in (('serving null', lambda t: serving(None)), ('no status', lambda t: None)):
        out, vis, reason, dur = run_mission({DOCK: status_script([(0, 0)]), UNDOCK_PRG: PRG,
                                             MAPSTAT: stat})
        check('no active map (%s) -> rejected' % label, out == 'rejected', out)
        check('no active map (%s): never undocks' % label,
              vis == ['CHECK_ACTIVE_MAP'] and
              not any(t == '/dock_smach/start_docking' for t, m in W.published), vis)
        check('no active map (%s): fast (<20 s)' % label, dur < 20.0, '%.1fs' % dur)
        check('no active map (%s): user message' % label,
              any('no map is active' in s for s in W.texts('/nextion/log_info')))
        check('no active map (%s): stop_reason' % label,
              'rejected:no_active_map' in W.texts('/mower_smach/stop_reason'))

    # mapping_manager respawn: serving:null for a while, then the site is back
    out, vis, reason, dur = run_mission({DOCK: status_script([(0, 0)]),
                                         MAPSTAT: lambda t: serving('Nmap' if t >= 5.0 else None)})
    check('CHECK_ACTIVE_MAP tolerates a short serving:null',
          'CHECK_IF_DOCKED' in vis, vis)
    out, vis, reason, dur = run_mission(dict(off_dock, **{
        PLANNER: lambda t: String(data='Nmap'),
        MAPSTAT: lambda t: serving(None if 0.3 <= t < 40.0 else 'Nmap')}))
    check('WAIT_FOR_PLANNER waits out a serving:null gap',
          'SET_PROGRAM_SPEED' in vis, vis)

    # Site un-served for good AFTER the robot left the dock: never a quiet
    # 'rejected' (robot stranded, status Ready) - the normal failure path.
    out, vis, reason, dur = run_mission(dict(off_dock, **{
        PLANNER: lambda t: String(data='Nmap'),
        MAPSTAT: lambda t: serving('Nmap' if t < 0.3 else None)}))
    check('site un-served in the field -> aborted (return to dock), not rejected',
          out == 'aborted' and vis[-1] == 'WAIT_FOR_PLANNER', (out, vis))
    check('site un-served in the field: user message',
          any('No map is active' in s for s in W.texts('/nextion/log_info')))
    check('site un-served in the field: no rejected stop_reason',
          not any(r.startswith('rejected') for r in W.texts('/mower_smach/stop_reason')))

    # legacy program: unchanged (no mapping status needed at all)
    legacy = 'GARDEN_2026***env*OUTDOOR'
    out, vis, reason, dur = run_mission(dict(off_dock, **{PLANNER: lambda t: String(data=legacy)}),
                                        map_name=legacy)
    check('legacy: planner_loaded == map_name -> continues', 'SET_PROGRAM_SPEED' in vis, vis)
    check('legacy: load_map requested with the map name',
          any(t == '/navi_manager/load_map' and m.data == legacy for t, m in W.published))
    out, vis, reason, dur = run_mission(dict(off_dock, **{PLANNER: lambda t: String(data='Nmap')}),
                                        map_name=legacy)
    check('legacy: other planner data -> timeout, aborted (as before)',
          out == 'aborted' and vis[-1] == 'WAIT_FOR_PLANNER', (out, vis))


def cancelled():
    return any(t == '/dock_smach/stop' and m.data for t, m in W.published)


def test_review_findings():
    healthy = {DOCK: status_script([(0, 3)]), MOTORS: MOTORS_ON}

    # F1. sticky preempt: a monitor fires while a rejecting state runs; the
    # NEXT, healthy mission on the SAME state machine must run normally.
    W.__init__()
    pubs = MowerPublishers()
    parent = make_parent()
    cc = mission.build_mission_concurrence(pubs, parent)
    child = cc.get_children()['MISSION_CHILD']
    fired = [False]

    def motors(t):
        if not fired[0]:
            fired[0] = True
            child.request_preempt()
        return Bool(data=False)
    W.script = {DOCK: status_script([(0, 3)]), MOTORS: motors, MAPSTAT: lambda t: serving('Nmap')}
    out1 = child.execute(parent.userdata)
    check('F1: preempt during CHECK_MOTORS_ON is serviced', out1 == 'preempted', out1)
    W.__init__()
    W.script = dict(healthy, **{MAPSTAT: lambda t: serving('Nmap')})
    vis = []
    child.register_transition_cb(lambda ud, active: vis.extend(active), cb_args=[])
    child.execute(parent.userdata)
    check('F1: next mission runs normally', 'LOAD_MAP' in vis, vis)
    # even an UNSERVICED leftover flag is recalled at the next mission start
    child.request_preempt()
    child._states['CHECK_MOTORS_ON'].request_preempt()
    W.__init__()
    W.script = dict(healthy, **{MAPSTAT: lambda t: serving('Nmap'), '/mower_smach/stop':
                                lambda t: Bool(data=True) if t > 300 else None})
    del vis[:]
    out = cc.execute(parent.userdata)
    check('F1: leftover preempt recalled at mission start (real Concurrence)',
          'LOAD_MAP' in vis and out == 'stop_preempt', (out, vis))

    # F1/F9. preemption inside every new state is serviced, flags are clean
    docked = {DOCK: status_script([(0, 0), (12, 1)]), UNDOCK_PRG: PRG}
    for state, script in (('CHECK_ACTIVE_MAP', {DOCK: status_script([(0, 0)]),
                                                MAPSTAT: lambda t: serving(None)}),
                          ('CHECK_IF_DOCKED', {DOCK: status_script([(0, 5)])}),
                          ('CHECK_MOTORS_ON', {DOCK: status_script([(0, 3)]), MOTORS: MOTORS_OFF}),
                          ('WAIT_FOR_UNDOCKED', docked),
                          ('WAIT_FOR_PLANNER', dict(healthy, **{
                              GPS: lambda t: Navi_transform(status='SAT', info='0.5s,0.5s')}))):
        out, vis, reason, dur = run_mission(script, preempt_in=state)
        sm = LAST['sm']
        check('preempt in %s -> preempted' % state,
              out == 'preempted' and vis[-1] == state, (out, vis))
        check('preempt in %s: state flag serviced' % state,
              not sm._states[state].preempt_requested())

    # F2. preempt mid-undock cancels dock_smach and waits for it to stop
    script = {DOCK: status_script([(0, 0), (2, 1)]), UNDOCK_PRG: PRG}
    out, vis, reason, dur = run_mission(script, preempt_in='WAIT_FOR_UNDOCKED')
    check('F2: preempt mid-undock cancels dock_smach', out == 'preempted' and cancelled(), out)
    check('F2: waited for dock_smach to stop (~5 s, it never confirmed)', 5.0 <= dur < 15.0,
          '%.1fs' % dur)
    # dock_smach confirms the cancel (4): no need to sit out the whole wait
    out, vis, reason, dur = run_mission(
        {DOCK: lambda t: Int8(data=4 if cancelled() else (1 if t >= 1 else 0)), UNDOCK_PRG: PRG},
        preempt_in='WAIT_FOR_UNDOCKED')
    check('F2: returns as soon as dock_smach confirms', out == 'preempted' and dur < 4.0,
          (out, '%.1fs' % dur))

    # F3. slow but healthy undock (25 min) is NOT killed; the cap is a param
    out, vis, reason, dur = run_mission({
        DOCK: status_script([(0, 0), (2, 1), (1500, 3)]), UNDOCK_PRG: PRG, MOTORS: MOTORS_ON})
    check('F3: 1500 s undock still succeeds', 'LOAD_MAP' in vis and not cancelled(), vis)
    out, vis, reason, dur = run_mission({DOCK: status_script([(0, 0), (2, 1)]), UNDOCK_PRG: PRG})
    check('F3: absolute cap 1800 s', reason == 'UNDOCK_TIMEOUT' and 1800 <= dur < 1830,
          (reason, dur))
    out, vis, reason, dur = run_mission({DOCK: status_script([(0, 0), (2, 1)]), UNDOCK_PRG: PRG},
                                        params={'~undock_timeout': 100.0})
    check('F3: ~undock_timeout honoured', reason == 'UNDOCK_TIMEOUT' and 100 <= dur < 130,
          (reason, dur))
    out, vis, reason, dur = run_mission({DOCK: status_script([(0, 0), (2, 1)]), UNDOCK_PRG: PRG},
                                        dead_after={DOCK: 60.0})
    check('F3: dock_smach died mid-undock -> fast failure',
          out == 'aborted' and reason == 'UNDOCK_FAILED' and dur < 120, (out, reason, dur))

    # F5. a charger glitch (3 without 1) is not an undock
    out, vis, reason, dur = run_mission({
        DOCK: status_script([(0, 0), (11.3, 3), (11.6, 0)]), UNDOCK_PRG: PRG})
    check('F5: status 3 without 1 is not accepted',
          out == 'aborted' and 'LOAD_MAP' not in vis, (out, vis))

    # F6. dock_smach silent after the program was sent
    out, vis, reason, dur = run_mission({
        DOCK: lambda t: Int8(data=0) if t < 0.5 else None, UNDOCK_PRG: PRG})
    check('F6: no dock status at all -> "not responding"',
          out == 'aborted' and reason == 'UNDOCK_FAILED' and dur < 40 and
          any('not responding' in s for s in W.texts('/nextion/log_info')),
          (out, reason, dur, W.texts('/nextion/log_info')))

    # F7. PreStartCheck rejections: same strings as before, then "Ready"
    from vitulus_msgs.msg import Power_status as PS
    W.__init__()
    pubs = MowerPublishers()
    st = states.PreStartCheck(pubs)
    W.script = {PM: lambda t: PS(battery_capacity=20, supply_status='OFFLINE')}
    ud = smach.UserData()
    ud.program = PlannerProgram(name='T1')
    out = st.execute(ud)
    check('F7: low battery -> rejected', out == 'rejected', out)
    check('F7: stop_reason string unchanged (master_controller parses it)',
          W.texts('/mower_smach/stop_reason') == ['rejected:battery'])
    check('F7: status string unchanged', W.texts('/mower_smach/status') == ['Rejected: low battery'])
    W.advance(6.0)
    check('F7: then Ready (master_controller can close the program)',
          W.texts('/mower_smach/status')[-1] == 'Ready')

    # F7. Run pressed during the "Rejected" hold: not dropped, not overwritten
    W.__init__()
    pubs = MowerPublishers()
    states.publish_mission_rejected(pubs, 'x', 'Rejected: x', 'x')
    wait_cc = mission.build_wait_for_program(pubs)
    mon = wait_cc.get_children()['MONITOR_PROGRAM_ACTIVE']
    ud = smach.UserData()
    for k in wait_cc.get_registered_output_keys():
        setattr(ud, k, None)
    mon._cond_cb(ud, PlannerProgram(name='B1'))
    W.advance(10.0)
    status = W.texts('/mower_smach/status')
    check('F7: new mission status survives the hold', status[-1] == 'Initiating', status)
    check('F7: stale stop_reason cleared at mission start',
          W.texts('/mower_smach/stop_reason')[-1] == '')


def make_parent():
    parent = smach.StateMachine(outcomes=['x'])
    parent.userdata.program = PlannerProgram(name='T1', map_name='SITE')
    for k in ('prg_start_time', 'unfinished_active', 'unfinished_zone', 'unfinished_path',
              'unfinished_window', 'path_window_start_index', 'path_chunk',
              'restore_height_pending', 'consecutive_nav_failures'):
        setattr(parent.userdata, k, None)
    parent.userdata.error_reason = ''
    parent.userdata.skipped_spans = []
    return parent


def test_real_concurrence_and_top_level():
    # The REAL Concurrence: rejection comes out as 'mission_rejected' ...
    W.__init__()
    W.script = {DOCK: status_script([(0, 0)]), MAPSTAT: lambda t: serving('Nmap')}
    parent = make_parent()
    cc = mission.build_mission_concurrence(MowerPublishers(), parent)
    out = cc.execute(parent.userdata)
    check('real Concurrence: no undock program -> mission_rejected', out == 'mission_rejected', out)
    # ... and a STOP during the undock wins and cancels dock_smach.
    W.__init__()
    W.script = {DOCK: status_script([(0, 0), (2, 1)]), UNDOCK_PRG: PRG,
                MAPSTAT: lambda t: serving('Nmap'),
                '/mower_smach/stop': lambda t: Bool(data=True) if t > 30 else None}
    out = cc.execute(parent.userdata)
    check('real Concurrence: STOP mid-undock -> stop_preempt + dock cancel',
          out == 'stop_preempt' and cancelled(), out)

    # The top-level SM from nodes/mower_unit_smach builds and is consistent.
    import importlib.machinery
    import importlib.util
    import smach_ros
    from mower_unit_smach import helpers
    built = {}

    def fake_execute(self, parent_ud=None):
        self.check_consistency()
        built['sm'] = self
        return 'shutdown'

    class FakeSis(object):
        def __init__(self, *a):
            pass
        start = stop = lambda self: None

    path = os.path.join(HERE, '..', 'nodes', 'mower_unit_smach')
    spec = importlib.util.spec_from_loader(
        'mower_node', importlib.machinery.SourceFileLoader('mower_node', path))
    node = importlib.util.module_from_spec(spec)
    real = (rospy.init_node, helpers.wait_for_mbf, smach_ros.IntrospectionServer,
            smach.StateMachine.execute)
    try:
        rospy.init_node = lambda *a, **kw: None
        helpers.wait_for_mbf = lambda *a, **kw: True
        smach_ros.IntrospectionServer = FakeSis
        spec.loader.exec_module(node)
        smach.StateMachine.execute = fake_execute
        W.__init__()
        node.main()
    finally:
        (rospy.init_node, helpers.wait_for_mbf, smach_ros.IntrospectionServer,
         smach.StateMachine.execute) = real
    top = built.get('sm')
    check('top-level SM builds and is consistent', top is not None)
    trans = top._transitions['MISSION_CONCURRENCE'] if top else {}
    check("top level: 'mission_rejected' -> WAIT_FOR_PROGRAM",
          trans.get('mission_rejected') == 'WAIT_FOR_PROGRAM', trans)
    check('top level: every MISSION_CONCURRENCE outcome is mapped',
          top is not None and set(top._states['MISSION_CONCURRENCE'].get_registered_outcomes())
          <= set(trans), trans)


def test_concurrence():
    W.__init__()
    W.script = {DOCK: status_script([(0, 0)])}
    pubs = MowerPublishers()
    parent = make_parent()
    cc = mission.build_mission_concurrence(pubs, parent)
    check("concurrence has 'mission_rejected'",
          'mission_rejected' in cc.get_registered_outcomes())
    out = cc._outcome_cb({'MISSION_CHILD': 'rejected'})
    check("child 'rejected' -> 'mission_rejected'", out == 'mission_rejected', out)
    out = cc._outcome_cb({'MISSION_CHILD': 'rejected', 'STOP_MONITOR': 'invalid'})
    check('STOP still wins over rejected', out == 'stop_preempt', out)
    node = open(os.path.join(HERE, '..', 'nodes', 'mower_unit_smach')).read()
    check("top level: 'mission_rejected' -> WAIT_FOR_PROGRAM",
          "'mission_rejected': 'WAIT_FOR_PROGRAM'" in node)


def test_critical_routing():
    W.__init__()
    st = top_level.CriticalErrorState(MowerPublishers())
    for reason, expect in (('UNDOCK_FAILED', 'terminal'), ('UNDOCK_TIMEOUT', 'terminal'),
                           ('DOCKING_FAILED', 'terminal'), ('NAVIGATION_ABORTED', 'try_dock'),
                           ('', 'try_dock')):
        ud = smach.UserData()
        ud.error_reason = reason
        out = st.execute(ud)
        check('CRITICAL_ERROR %-20r -> %s' % (reason, expect), out == expect, out)


def test_run_request_notice():
    for cls, reset_out, needle in ((top_level.StoppedState, 'resume', 'Mission is stopped'),
                                   (top_level.TerminalErrorState, 'reset',
                                    'Mission ended with an error')):
        W.__init__()
        # Run pressed repeatedly for 5 min (the safe blade shutdown at state
        # entry takes simulated time), Reset afterwards.
        W.script = {
            '/web_plan/program_active':
                lambda t: PlannerProgram(name='B1') if t < 300.0 else None,
            '/mower_smach/reset': lambda t: Bool(data=True) if t >= 320.0 else None,
        }
        st = cls(MowerPublishers())
        ud = smach.UserData()
        ud.error_reason = 'UNDOCK_FAILED'
        out = st.execute(ud)
        logs = W.texts('/nextion/log_info')
        check('%s: Run reported' % cls.__name__,
              any(needle in s and 'press Reset' in s for s in logs), logs)
        check('%s: leaves only on Reset' % cls.__name__, out == reset_out, out)
        check('%s: nothing started' % cls.__name__,
              not any(t in ('/dock_smach/start_docking', '/navi_manager/load_map')
                      for t, m in W.published))
        check('%s: no developer-speak' % cls.__name__,
              not any('/mower_smach/reset' in s for s in logs), logs)
        check('%s: subscribers released' % cls.__name__, not W.subs, W.subs)


def test_wait_status():
    W.__init__()
    pubs = MowerPublishers()
    st = states.WaitForTopic('/nav_tf/odom_status', Int8, predicate=lambda m: True,
                             timeout=35.0, pubs=pubs, wait_label='GPS fix')
    out = st.execute(smach.UserData())
    status = W.texts('/mower_smach/status')
    check('WaitForTopic: timeout', out == 'timeout', out)
    check('WaitForTopic: periodic status (3x in 35 s)',
          len(status) == 3 and status[0].startswith('Waiting for GPS fix (10 s / 35 s)'),
          status)
    check('WaitForTopic: timeout reported',
          'Timed out waiting for GPS fix (35 s)' in W.texts('/nextion/log_info'))
    W.__init__()
    st = states.WaitForTopic('/x', Int8, predicate=lambda m: True, timeout=35.0)
    st.execute(smach.UserData())
    check('WaitForTopic without label stays silent', not W.published, W.published)


if __name__ == '__main__':
    test_mission()
    test_site_native_planner()
    test_review_findings()
    test_real_concurrence_and_top_level()
    test_concurrence()
    test_critical_routing()
    test_run_request_notice()
    test_wait_status()
    print('\n%s' % ('ALL PASSED' if not FAILS else 'FAILED: %s' % FAILS))
    sys.stdout.flush()
    os._exit(1 if FAILS else 0)   # smach_ros action states leave helper threads
