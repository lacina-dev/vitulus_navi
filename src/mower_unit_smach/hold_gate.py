"""Mission side of the cmd_vel_mux hold (vitulus_base, 31. 10. 2026).

While /cmd_vel_mux/held is True the mux drops every software command, so the
robot stands whatever the mission does.  Without this the mission would read
that as a failure: stall -> recovery -> detour -> skipped lines, timeouts ->
critical error.  With it the mission simply waits and goes on from where it was.

  HOLD.wrap_tree(sm)  no state of the tree STARTS during a hold (a state that
                      is already running is covered by HOLD.held / .stretch)
  HOLD.wait(state)    block until the hold ends; False = preempted (STOP works)
  HOLD.stretch(t, k)  deadline t moved by the time spent held since k's last call
  HOLD.seen_since(t)  a hold touched [t, now]: a failure there means nothing
"""
import threading

import rospy
from std_msgs.msg import Bool

from smach import Container


class HoldGate(object):
    TOPIC = '/cmd_vel_mux/held'

    def __init__(self):
        self._lock = threading.Lock()
        self._sub = None
        self._held = False
        self._last_end = None      # rospy.Time the last hold ended
        self._since = None         # rospy.Time the running hold started
        self._stretch = {}         # id(waiter) -> rospy.Time of its last stretch()

    def start(self):
        if self._sub is None:
            self._sub = rospy.Subscriber(self.TOPIC, Bool, self._cb, queue_size=5)

    def _cb(self, msg):
        now = rospy.Time.now()
        with self._lock:
            if msg.data and not self._held:
                self._since = now
            elif self._held and not msg.data:
                self._last_end = now
                self._since = None
            self._held = bool(msg.data)

    def held(self):
        self.start()
        with self._lock:
            return self._held

    def seen_since(self, t):
        with self._lock:
            return self._held or (self._last_end is not None and self._last_end >= t)

    def wait(self, state=None, status_pub=None, text='Held: waiting for GPS'):
        """Block while held.  True = the hold is over, False = preempted."""
        said = False
        while self.held() and not rospy.is_shutdown():
            if state is not None and state.preempt_requested():
                return False
            if status_pub is not None and not said:
                said = True
                status_pub.publish(text)
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
