"""Mission side of the cmd_vel_mux hold (vitulus_base, 31. 10. 2026).

While /cmd_vel_mux/held is True, or the map rules limit is stale
(/cmd_vel_mux/status limit.stale), the mux drops every software command, so the
robot stands whatever the mission does.  Without this the mission would read
that as a failure: stall -> recovery -> detour -> skipped lines, timeouts ->
critical error.  With it the mission simply waits and goes on from where it was.

  HOLD.wrap_tree(sm)  no state of the tree STARTS during a hold (a state that
                      is already running is covered by HOLD.held / .stretch)
  HOLD.wait(state)    block until the hold ends; False = preempted (STOP works)
  HOLD.stretch(t, k)  deadline t moved by the time spent held since k's last call
  HOLD.seen_since(t)  a hold touched [t, now]: a failure there means nothing
"""
import json
import threading

import rospy
from std_msgs.msg import Bool, String

from smach import Container


class HoldGate(object):
    TOPIC = '/cmd_vel_mux/held'
    STATUS_TOPIC = '/cmd_vel_mux/status'

    def __init__(self):
        self._lock = threading.Lock()
        self._sub = None
        self._status_sub = None
        self._held = False
        self._mux_held = False
        self._stale = False
        self._blade_wait = False   # stale_blade.py: the blade comes back before the mission
        self._temp_wait = False    # temp_watch.py: firmware TEMP, spike or real?
        self._last_end = None      # rospy.Time the last hold ended
        self._since = None         # rospy.Time the running hold started
        self._stretch = {}         # id(waiter) -> rospy.Time of its last stretch()

    def start(self):
        if self._sub is None:
            self._sub = rospy.Subscriber(self.TOPIC, Bool, self._cb, queue_size=5)
            self._status_sub = rospy.Subscriber(self.STATUS_TOPIC, String, self._status_cb,
                                                queue_size=1)

    def _cb(self, msg):
        with self._lock:
            self._mux_held = bool(msg.data)
            self._update(rospy.Time.now())

    def _status_cb(self, msg):
        try:
            limit = json.loads(msg.data).get('limit') or {}
            stale = bool(limit.get('stale'))
        except (ValueError, AttributeError):
            return
        with self._lock:
            self._stale = stale
            self._update(rospy.Time.now())

    def _update(self, now):
        held = self._mux_held or self._stale or self._blade_wait or self._temp_wait
        if held and not self._held:
            self._since = now
        elif self._held and not held:
            self._last_end = now
            self._since = None
        self._held = held

    def limit_stale(self):
        with self._lock:
            return self._stale

    def mux_held(self):
        with self._lock:
            return self._mux_held

    def set_blade_wait(self, waiting):
        with self._lock:
            if waiting != self._blade_wait:
                self._blade_wait = waiting
                self._update(rospy.Time.now())

    def set_temp_wait(self, waiting):
        with self._lock:
            if waiting != self._temp_wait:
                self._temp_wait = waiting
                self._update(rospy.Time.now())

    def held(self):
        self.start()
        with self._lock:
            return self._held

    def seen_since(self, t):
        with self._lock:
            return self._held or (self._last_end is not None and self._last_end >= t)

    def _why(self, text):
        if self._mux_held:
            return text
        return 'Held: map rules stop' if self._stale else 'Held: blade coming back'

    def wait(self, state=None, status_pub=None, text='Held: waiting for GPS'):
        """Block while held.  True = the hold is over, False = preempted."""
        said = False
        while self.held() and not rospy.is_shutdown():
            if state is not None and state.preempt_requested():
                return False
            if status_pub is not None and not said:
                said = True
                status_pub.publish(self._why(text))
            rospy.sleep(0.2)
        return True

    def stretch(self, deadline, key):
        """In a wait loop: `deadline = HOLD.stretch(deadline, self)`.  The deadline
        moves by the time since this key's previous call, while held."""
        now = rospy.Time.now()
        with self._lock:
            held, last = self._held, self._stretch.get(id(key))
            self._stretch[id(key)] = now
        if not held or last is None or (now - last).to_sec() > 5.0:
            return deadline
        return deadline + (now - last)

    def wrap(self, state):
        original = state.execute

        def execute(userdata):
            if self.held() and not self.wait(state):
                if 'preempted' in state.get_registered_outcomes():
                    state.service_preempt()
                    return 'preempted'
            return original(userdata)
        state.execute = execute

    def wrap_tree(self, container):
        """Gate every leaf state below *container*."""
        self.start()
        for child in container.get_children().values():
            if isinstance(child, Container):
                self.wrap_tree(child)
            else:
                self.wrap(child)


HOLD = HoldGate()
