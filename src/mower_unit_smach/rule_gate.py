"""Mission side of the map rules (vitulus_safety map_rules_guard, 1. 11. 2026).

The mux caps the speed behind TEB's back and the guard holds the blade off, and
neither of them may read as a failure:

  RULES.capped_below(v, reverse)  the limit of the way the robot goes is under
                         v m/s, see StallAnchor
  RULES.blade_held()     the guard holds the blade off: a state waiting for the
                         blade goes on, the guard gives it back behind the area
  RULES.blade_failed()   the blade did not come back after a noblade area, or a
                         guard restart cannot say: the mission must not go on
                         as if it were mowing, ExecutePathWithFeedback takes it
                         as BLOCKED, whose recovery starts the blade itself

No guard, or no message for MAX_AGE_S: both False, the mission is as it was.
"""
import json
import math
import threading
import time

import rospy
from std_msgs.msg import String


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

    def start(self):
        if self._sub is None:
            self._sub = rospy.Subscriber(self.TOPIC, String, self._cb, queue_size=1)

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
        """The footprint is on a noblade area, or the guard says it still is."""
        state = self._fresh() or {}
        return (state.get('blade') == 'stopped_by_rule'
                or any(i.get('type') == 'noblade' for i in state.get('in') or []))

    def set_local_failed(self, failed):
        self._local_failed = failed

    def blade_failed(self):
        return self._local_failed or (self._fresh() or {}).get('blade') == 'restore_failed'


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
