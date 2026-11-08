"""Mission side of the map rules (vitulus_safety map_rules_guard, 1. 11. 2026).

The mux caps the speed behind TEB's back and the guard holds the blade off, and
neither of them may read as a failure:

  RULES.capped_below(v, reverse)  the limit of the way the robot goes is under
                         v m/s, see StallAnchor
  RULES.blade_held()     the guard holds the blade off (noblade area, or outside
                         the zone being mowed): a state waiting for the blade
                         goes on, the guard gives it back behind the area
  RULES.blade_failed()   the blade did not come back after a noblade area, or a
                         guard restart cannot say: the mission must not go on
                         as if it were mowing, ExecutePathWithFeedback takes it
                         as BLOCKED, whose recovery starts the blade itself

  ZoneSpinUp             back in the zone after the zone fence stopped the blade
                         (guard `restoring`, `held_by` zone): HOLD stands, so the
                         robot waits at the edge until the blade is up to speed
                         (WaitForRpmReached's test: setpoint +-100, > 1000 rpm
                         without one).  The start of a path and a zone start from a
                         pose behind the edge are mowed too, when the transit to
                         it is the mission's own path (EXE_PATH_TO_BEGIN/_START and
                         RemowSkippedSpans drive with the blade as they always did).
                         The blade is never started ahead outside the zone: the
                         guard does that, in it.  Going home (stop_reason, the home
                         latch) or a dropped debt ends the wait at once.

No guard, or no message for MAX_AGE_S: both False, the mission is as it was.
"""
import json
import math
import threading
import time

import rospy
from std_msgs.msg import String

MOWER_FAULTS = ('ERR', 'BLOCKED', 'TEMP')


class RuleGate(object):
    TOPIC = '/map_rules/state'
    MAX_AGE_S = 2.0

    def __init__(self):
        self._lock = threading.Lock()
        self._sub = None
        self._state = None
        self._t = None
        self._last_said = 0.0
        self._local_failed = False    # stale_blade.py: the blade did not spin up

    def start(self, hold=None):
        if self._sub is None:
            self._sub = rospy.Subscriber(self.TOPIC, String, self._cb, queue_size=1)
            if hold is not None:
                from vitulus_msgs.msg import Mower
                self._spin = ZoneSpinUp()
                self._hold = hold
                rospy.Subscriber('/mower/status', Mower, self._mower, queue_size=1)
                rospy.Subscriber('/mower_smach/stop_reason', String, self._stop_reason,
                                 queue_size=1)
                rospy.Timer(rospy.Duration(0.2), self._tick)

    def _stop_reason(self, msg):
        with self._lock:
            self._reason = msg.data or ''

    def _leaving(self):
        """The mission goes home or ends: no blade to wait for."""
        with self._lock:
            if getattr(self, '_reason', '').strip():
                return True
        try:
            from mower_unit_smach.helpers import run_flags
            latch = run_flags.home_latch
            return bool(latch is not None and latch.reason)
        except Exception:  # noqa: BLE001 - unknown is not leaving
            return False

    def _mower(self, msg):
        with self._lock:
            self._status = (msg.status, int(msg.moto_rpm), int(msg.setpoint_rpm))

    def _tick(self, _evt):
        try:
            with self._lock:
                status = getattr(self, '_status', None)
            was = self._spin.waiting
            waiting = self._spin.tick(time.monotonic(), self._fresh(), status,
                                      leaving=self._leaving())
            self._hold.set_rule_wait(waiting)
            if waiting != was:
                rospy.logwarn('[RULE_GATE] %s', 'back in the zone: waiting for the blade'
                              if waiting else 'blade up (or no longer waited for): go on')
        except Exception as exc:  # noqa: BLE001 - a dead timer would hide it for good
            rospy.logerr('[RULE_GATE] tick failed: %r', exc)

    def _cb(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        with self._lock:
            self._state = state
            self._t = time.monotonic()

    def _fresh(self):
        with self._lock:
            if self._t is None or time.monotonic() - self._t > self.MAX_AGE_S:
                return None
            return self._state

    def capped_below(self, speed, reverse=False):
        state = self._fresh()
        limit = (state or {}).get('limit') or {}
        if not limit.get('rules'):
            return False
        return limit.get('v_back' if reverse else 'v_fwd', speed) < speed

    def blade_held(self):
        return (self._fresh() or {}).get('blade') == 'stopped_by_rule'

    def noblade_here(self):
        """The footprint is on a noblade area, outside the zone being mowed (the
        guard's zone fence), or the guard says it still is."""
        state = self._fresh() or {}
        return (state.get('blade') == 'stopped_by_rule'
                or bool((state.get('zone') or {}).get('outside'))
                or any(i.get('type') == 'noblade' for i in state.get('in') or []))

    def set_local_failed(self, failed):
        self._local_failed = failed

    def blade_failed(self):
        return self._local_failed or (self._fresh() or {}).get('blade') == 'restore_failed'


class ZoneSpinUp(object):
    """Hold the robot at the zone edge until the blade the guard gives back is up
    to speed.  Pure.  `tick` -> waiting."""
    RPM_TOL = 100          # WaitForRpmReached
    RPM_ANY = 1000         # ... without a setpoint
    MAX_S = 30.0           # the guard gives up (restore_failed) well before

    def __init__(self):
        self.waiting = False
        self.since = None
        self.gave_up = False      # this restore timed out: not again until it ends

    @classmethod
    def up(cls, rpm, setpoint):
        if setpoint and setpoint > 0:
            return setpoint - cls.RPM_TOL < rpm < setpoint + cls.RPM_TOL
        return rpm > cls.RPM_ANY

    def _stop(self):
        self.waiting = False
        self.since = None
        return False

    def tick(self, now, state, status, leaving=False):
        """`state` the guard's fresh /map_rules/state or None, `status`
        (fw status, rpm, setpoint) or None, `leaving` the mission goes home."""
        if state is None or leaving:
            return self._stop()       # no guard: stale_blade.py has the blade
        blade = state.get('blade')
        fw, rpm, setpoint = status if status is not None else (None, 0, 0)
        if not self.waiting:
            if blade != 'restoring':
                self.gave_up = False
            if blade == 'restoring' and state.get('held_by') == 'zone' and not self.gave_up:
                self.waiting = True
                self.since = now
            else:
                return False
        if fw in MOWER_FAULTS or blade in ('restore_failed', 'stopped_by_rule'):
            return self._stop()       # the mission's handling, or out again: drive
        if self.up(rpm, setpoint) and fw == 'RUN':
            return self._stop()
        if blade != 'restoring' and state.get('held_by') is None and fw != 'RUN':
            return self._stop()       # no debt, nobody spins it up (the mission said False)
        if now - self.since > self.MAX_S:
            self.gave_up = True
            return self._stop()
        return True


class StallAnchor(object):
    """The stall test's distance under a cap TEB does not know.  One 0.5 s sample
    is too short there: 0.05 m/s makes 2.5 cm of the 8 cm the test wants, a false
    stall.  Measured from the last move it is the same 8 cm.  Uncapped the sample
    itself goes through, exactly as before."""

    def __init__(self):
        self.anchor = None

    def reset(self):
        self.anchor = None

    def moved(self, last_xy, cur_xy, sample, capped):
        if not capped:
            self.anchor = None
            return sample
        if self.anchor is None:
            self.anchor = last_xy
        return math.hypot(cur_xy[0] - self.anchor[0], cur_xy[1] - self.anchor[1])


RULES = RuleGate()
