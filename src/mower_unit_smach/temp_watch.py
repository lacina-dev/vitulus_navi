"""Firmware TEMP during a mission (1. 11. 2026).

The firmware stops the blade by itself when one 1 s window reads >= 70 C, so a TEMP
never needs the mission to protect the motor.  What it decides is the mission.
On 1. 11. the window read 90.7 C, the next one 20.9 C, the firmware said READY a
second later and the program went home all the same.  The motor cools at about
1 C/min even from 70 C, so a motor that was really hot cannot drop to the firmware's
55 C release in seconds.  That is the test, DEFAULTS in seconds:

    TEMP            the mission stands (hold, as for a mux hold), wait_s at most
    released        status left TEMP, reading <= cool_c: a spike.  The firmware does
                    not restart the blade: True here, only if the blade was running or
                    commanded on, the last command is not False and no map rule
                    holds it
    up              hold released, the mission goes on from where it was
    TEMP stays      'temp', sleep and resume as always
    spin-up         a state waiting for the blade does not fail on TEMP either, see
                    TempGrace: this monitor alone decides, inside its window
    spike_max spikes in spike_window_s, or no blade in spinup_s
                    'temp_spike': home, a short wait, resume
    silence         no status in the wait: 'temp', in the restart: 'temp_spike'.  The
                    mission does not go on with a blade nobody can vouch for.
    safety_node says overtemp or sensor_fault (/safety/mower_temp)
                    'temp' or 'temp_sensor', whatever the firmware says, also with
                    no status at all (the home latch).  It may have stopped the
                    blade with no TEMP.  safety_node gone: no verdict, 'unknown'.
    True            not while the mux holds or the rules limit is stale: it waits,
                    and the spin-up clock with it.  stale_blade's last command False
                    cancels it.

TempWatch is pure.  TempWatchShell is the ROS side, called from the monitor state.
"""
import collections
import json
import threading
import time

import rospy
from std_msgs.msg import Bool, String

from .rule_gate import RULES

DEFAULTS = {
    'wait_s': 10.0, 'cool_c': 55.0, 'spinup_s': 30.0, 'spike_max': 3,
    'spike_window_s': 1800.0, 'on_rpm': 300, 'rpm_tol': 100, 'rpm_max': 7000, 'silent_s': 3.0,
}
STOP_REASONS = {'temp': 'interrupted:temp', 'temp_spike': 'interrupted:temp_spike',
                'temp_sensor': 'interrupted:temp_sensor'}
NO_BLADE = ('OFF', 'INIT')
GRACE_MARGIN_S = 3.0        # the monitor ends a real TEMP first: interrupted:temp
MISSION_FAULTS = ('ERR', 'BLOCKED')


class TempGrace(object):
    """One wait state's patience with TEMP.  Longer than the monitor's window, so
    a real TEMP ends as interrupted:temp through the monitor, not as this state's
    error.  A state outside the monitor still gets its error, a little later."""

    def __init__(self, wait_s=DEFAULTS['wait_s']):
        self.grace_s = wait_s + GRACE_MARGIN_S
        self.since = None

    def waits(self, now, status):
        """True: this FW error state is no failure yet, keep waiting."""
        if status != 'TEMP':
            return False
        if self.since is None:
            self.since = now
        return now - self.since < self.grace_s


class TempWatch(object):

    def __init__(self, params=None):
        self.p = dict(DEFAULTS)
        self.p.update(params or {})
        self.phase = None           # None | 'wait' | 'restart'
        self.since = None
        self.was_run = False
        self.commanded = None       # the last /mower/set_motor_on
        self.reason = None          # why the mission must end
        self.last_status = None
        self.safety = 'ok'
        self.spikes = collections.deque()

    @property
    def hold(self):
        return self.phase is not None

    def verdict(self):
        return {'overtemp': 'temp', 'sensor_fault': 'temp_sensor'}.get(self.safety)

    def reset(self):
        """A new mission.  The spikes stay: they count by time."""
        self.phase = None
        self.reason = None
        self.last_status = None

    def on_safety(self, state):
        self.safety = state

    def on_command(self, value):
        """/mower/set_motor_on: a False from anyone cancels the restart.  A True
        may have been refused by the firmware during TEMP: the restart stands."""
        self.commanded = bool(value)
        if not value and self.phase == 'restart':
            self.phase = None

    def _end(self, reason):
        self.phase = None
        self.reason = reason
        return False, []

    def _spike(self, now):
        self.spikes.append(now)
        while self.spikes and now - self.spikes[0] > self.p['spike_window_s']:
            self.spikes.popleft()
        return len(self.spikes) >= self.p['spike_max']

    def _up(self, rpm, setpoint):
        if not 0 <= rpm <= self.p['rpm_max']:
            return False            # the firmware's stall artefact, not a speed
        if setpoint > 0:
            return abs(rpm - setpoint) < self.p['rpm_tol']
        return rpm >= self.p['on_rpm']

    def on_status(self, now, status, temp, rpm, setpoint, blade_held=False):
        """One /mower/status.  Returns (keep_monitoring, actions)."""
        if self.reason is not None:
            return False, []
        if self.verdict():
            return self._end(self.verdict())
        last, self.last_status = self.last_status, status

        if status == 'TEMP':
            if self.phase != 'wait':
                # The status comes at 2 Hz: a spin-up can reach TEMP without a RUN.
                self.was_run = (last == 'RUN' or self.phase == 'restart'
                                or self.commanded is True)
                self.phase = 'wait'
                self.since = now
                return True, ['waiting']
            if now - self.since >= self.p['wait_s']:
                return self._end('temp')
            return True, []

        if self.phase == 'wait':
            return self._after_temp(now, status, temp, blade_held)
        if self.phase == 'restart':
            if status == 'RUN' and self._up(rpm, setpoint):
                self.phase = None
                return True, ['blade_up']
            if status in MISSION_FAULTS:
                self.phase = None           # the mission's own BLOCKED/ERR handling
            elif now - self.since >= self.p['spinup_s']:
                return self._end('temp_spike')
        return True, []

    def _after_temp(self, now, status, temp, blade_held):
        if status in MISSION_FAULTS:
            self.phase = None
            return True, []
        if status in NO_BLADE or not 0.0 < temp <= self.p['cool_c']:
            # No proof of a spike, or a board that cannot start the blade.  NaN,
            # 0.0 (no window yet) and -273 C (open NTC) prove nothing.
            if now - self.since >= self.p['wait_s']:
                return self._end('temp')
            return True, []
        if self._spike(now):
            return self._end('temp_spike')
        if self.was_run and self.commanded is not False and not blade_held:
            self.phase = 'restart'
            self.since = now
            return True, ['spike', 'blade_on']
        self.phase = None
        return True, ['spike']

    def silent(self):
        """The monitor stopped asking, or the mower fell silent: no hold of ours
        may outlive that, and the mission may not go on as if nothing happened."""
        if self.phase == 'wait':
            self._end('temp')
        elif self.phase == 'restart':
            self._end('temp_spike')
        return self.reason


class TempWatchShell(object):

    def __init__(self):
        self.lock = threading.Lock()
        self.core = None
        self.pubs = None
        self.hold = None
        self.latch = None
        self.blade_watch = None     # StaleBladeWatch, set by the node
        self.safety_sub = None
        self.blade_due = False      # a True not sent yet
        self.last_seen = None

    def attach(self, pubs, hold, latch):
        if self.core is not None:
            return
        self.pubs = pubs
        self.hold = hold
        self.latch = latch
        params = rospy.get_param('~mower_temp_watch', {})
        self.core = TempWatch(params)
        self.safety_sub = rospy.Subscriber('/safety/mower_temp', String, self._safety,
                                           queue_size=5)
        rospy.Subscriber('/mower/set_motor_on', Bool, self._command, queue_size=5)
        rospy.Timer(rospy.Duration(1.0), self._tick)

    def _safety(self, msg):
        try:
            state = json.loads(msg.data).get('state', 'ok')
        except (ValueError, AttributeError):
            return
        with self.lock:
            self.core.on_safety(state)

    def _command(self, msg):
        with self.lock:
            self.core.on_command(msg.data)
            self.hold.set_temp_wait(self.core.hold)

    def _end_now(self, reason):
        """The monitor may hear nothing more: preempt through the home latch, which
        says no when no mission runs."""
        return self.latch.request(reason)

    def _tick(self, _evt):
        try:
            with self.lock:
                if self.safety_sub.get_num_connections() == 0:
                    self.core.on_safety('unknown')    # the latched verdict died with it
                end = None
                if (self.last_seen is not None and self.core.hold and
                        time.monotonic() - self.last_seen > self.core.p['silent_s']):
                    end = self.core.silent()
                    rospy.logwarn('[MOWER_TEMP_MONITOR] no status while waiting: %s', end)
                verdict = self.core.verdict() if self.core.reason is None else None
                self.hold.set_temp_wait(self.core.hold)
            if end is not None:
                self._end_now(end)
            if verdict is not None and self._end_now(verdict):
                with self.lock:
                    self.core._end(verdict)
                    self.hold.set_temp_wait(self.core.hold)
            if self.blade_due:
                self._try_blade()
        except Exception as exc:  # noqa: BLE001 - a dead timer would hide it for good
            rospy.logerr('[MOWER_TEMP_MONITOR] tick failed: %r', exc)

    def on_status(self, msg):
        """The monitor callback's answer: False ends the mission."""
        now = time.monotonic()
        with self.lock:
            if (self.last_seen is None or
                    now - self.last_seen > self.core.p['silent_s']) and not self.core.hold:
                self.core.reset()
            self.last_seen = now
            keep, actions = self.core.on_status(
                now, msg.status, msg.temp, msg.moto_rpm, msg.setpoint_rpm, RULES.blade_held())
            self.hold.set_temp_wait(self.core.hold)
            reason = self.core.reason
        self._act(actions, msg)
        if not keep:
            rospy.logwarn('[MOWER_TEMP_MONITOR] Motor overtemperature detected (%s)', reason)
        return keep

    def _act(self, actions, msg):
        for action in actions:
            if action == 'waiting':
                rospy.logwarn('[MOWER_TEMP_MONITOR] firmware TEMP at %.1fC: waiting', msg.temp)
                self.pubs.log_info.publish(String('Blade off: temperature reading, checking'))
            elif action == 'spike':
                rospy.logwarn('[MOWER_TEMP_MONITOR] sensor spike, back at %.1fC', msg.temp)
                self.pubs.log_info.publish(String(
                    'Temperature spike: not a real overheat, mission goes on'))
            elif action == 'blade_on':
                self.blade_due = True
                self._try_blade()
            elif action == 'blade_up':
                rospy.loginfo('[MOWER_TEMP_MONITOR] blade back up')

    def _try_blade(self):
        blocked = self.hold.mux_held() or self.hold.limit_stale()
        off = False
        if self.blade_watch is not None:
            with self.blade_watch.lock:
                off = self.blade_watch.core.commanded is False
        with self.lock:
            if off and self.core.phase == 'restart':
                self.core.phase = None
            if self.core.phase != 'restart':
                self.blade_due = False
                self.hold.set_temp_wait(self.core.hold)
                return
            if blocked:
                self.core.since = time.monotonic()    # spinup_s runs from the True
                return
            self.blade_due = False
        self.pubs.mower_set_motor_on.publish(Bool(True))

    def grace(self):
        return TempGrace(self.core.p['wait_s'] if self.core is not None else DEFAULTS['wait_s'])

    def stop_reason(self):
        with self.lock:
            reason = self.core.reason if self.core is not None else None
        return STOP_REASONS.get(reason, STOP_REASONS['temp'])


TEMP_WATCH = TempWatchShell()
