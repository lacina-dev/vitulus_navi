"""Global /mower_smach/stop handling - immediate, idempotent, never blocking.

18. 10. 2026: the old handler ran safe_blade_shutdown INSIDE the subscriber
callback (up to 5 + 5 + 30 s for a mower without power) and only then sent
/dock_smach/stop.  rospy delivers one topic's messages one after another, so
every further STOP queued behind it and each one cancelled dock_smach ~40 s
after it was sent - it killed a docking the agent had just started, and the
cycle fed itself for minutes.  The node could not even be killed while a
callback hung in the blocking wait.

Now, per STOP message:
  * at once, in the callback: set_motor_on(False) (the safety part);
  * at once, and only while a mowing program runs (`busy_fn`): /dock_smach/stop
    and the mission preempt (UserStopLatch) - never later, never from the
    worker, and at most once per `coalesce_s` for a burst of STOPs;
  * the slow part (safe_blade_shutdown: wait for the motor, park the blade) runs
    on one worker thread; STOPs arriving meanwhile merge into one more run.
An idle mower is not a reason to cancel anybody's docking.
"""
import threading
import time

import rospy
from std_msgs.msg import Bool

from .helpers import safe_blade_shutdown


class GlobalStop(object):
    def __init__(self, pubs, stop_latch, busy_fn, shutdown_fn=safe_blade_shutdown,
                 clock=time.monotonic, coalesce_s=3.0, start_worker=True):
        self.pubs = pubs
        self.stop_latch = stop_latch
        self.busy_fn = busy_fn
        self.shutdown_fn = shutdown_fn
        self.clock = clock
        self.coalesce_s = float(coalesce_s)
        self.lock = threading.Lock()
        self.event = threading.Event()
        self.pending_busy = False
        self.last_propagated = None
        self.received = 0
        self.propagated = 0
        self.shutdown_runs = 0
        self.worker = None
        if start_worker:
            self.worker = threading.Thread(target=self._work, name='global-stop',
                                           daemon=True)
            self.worker.start()

    # -- the subscriber callback: returns at once ---------------------------
    def on_stop(self, msg):
        if not msg.data:
            return
        self.received += 1
        try:
            self.pubs.mower_set_motor_on.publish(Bool(False))
        except Exception as exc:  # noqa: BLE001 - keep going, the rest matters too
            rospy.logwarn('[GLOBAL_STOP] set_motor_on(False) failed: %s', exc)
        try:
            busy = bool(self.busy_fn())
        except Exception:  # noqa: BLE001 - unknown counts as a running program
            busy = True
        now = self.clock()
        if busy:
            with self.lock:
                fresh = (self.last_propagated is None
                         or now - self.last_propagated >= self.coalesce_s)
                if fresh:
                    self.last_propagated = now
            if fresh:
                self.propagated += 1
                rospy.logwarn('[GLOBAL_STOP] program running: blade off, '
                              'dock_smach stop, mission preempt')
                self.pubs.dock_cancel.publish(Bool(data=True))
            # Every STOP re-preempts a running mission (vitulus-field#53);
            # request() is idempotent and returns at once.
            if self.stop_latch is not None and self.stop_latch.request():
                rospy.logwarn('[GLOBAL_STOP] Mission preempt requested')
        else:
            rospy.loginfo('[GLOBAL_STOP] no mowing program runs: blade off only, '
                          'nothing else is stopped')
        with self.lock:
            self.pending_busy = self.pending_busy or busy
        self.event.set()

    # -- the slow part -------------------------------------------------------
    def run_pending(self):
        """One merged safe_blade_shutdown for every STOP since the last run."""
        with self.lock:
            busy = self.pending_busy
            self.pending_busy = False
        self.shutdown_runs += 1
        try:
            return self.shutdown_fn(source='GLOBAL_STOP', pubs=self.pubs,
                                    program_running=busy)
        except Exception as exc:  # noqa: BLE001 - the worker must survive
            rospy.logwarn('[GLOBAL_STOP] blade shutdown failed: %s', exc)
            return False

    def _work(self):
        while not rospy.is_shutdown():
            if not self.event.wait(0.5):
                continue
            self.event.clear()
            if rospy.is_shutdown():
                break
            self.run_pending()
