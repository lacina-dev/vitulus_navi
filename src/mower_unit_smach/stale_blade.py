"""Blade off while the map rules limit is stale (1. 11. 2026).

The mux stops all software when map_rules_guard falls silent, and the mission
waits as in a hold (hold_gate.py).  The guard may be dead, so the mission itself
must not leave a standing robot with a spinning blade: the same outcome as a
gnss_watchdog hold, here on the mission's side.

Stale here also means a fresh limit that stops everything (the mux reports
tf_stale and nogo_unparsable as limit.stale).  Times are DEFAULTS, seconds:

    stale grace_s       blade off if the firmware has it on, /mower/set_motor_on
                        only, re-asserted at most 1 Hz (recovery and the firmware's
                        retry can start it).  The grace ends only after 1 s of
                        fresh limit, a short respawn inside it touches nothing.
    limit back          stable for stable_s, not during a mux hold, then True and
                        the hold stands until the blade is up to speed.  Every
                        restart in the last window_s makes the next wait backoff
                        times longer, so a guard in a crash loop never cycles the
                        blade: it stays off, the robot stands, a healthy guard ends it.
    up                  hold released, the mission goes on from where it was.
    not up in spinup_s  `failed`: ExecutePathWithFeedback takes it as BLOCKED.
    ERR/BLOCKED         the debt ends, that is the mission's own handling.  TEMP waits.
    on a noblade area   no True, and the hold ends, only the mission can drive out.
                        Behind the area: TEMP waits, ERR/BLOCKED ends it, a blade
                        already on ends it, else stable_s off the area, then True
                        with the hold and the rpm check as above.
    escalate_s          stale that long during a program, flapping included: the
                        mission goes to TERMINAL_ERROR (buzzer, Reset), blade off.
                        Needs a running mission child: on the way to the dock
                        dock_smach waits in its hold and this is asked again.

True only with proof the mission wants it: the firmware reported RUN when the
blade was stopped, nobody but the guard has said False since (its False keeps
the debt, it stops what it sees on an area, and a mission False is the mission's
word), the same program still runs and no stop reason exists.  A False arriving within
1 s of our True may have crossed it on the wire: it goes out once more.

StaleBlade is pure.  StaleBladeWatch is the ROS shell: the mission's own commands
are seen through a thin wrapper on pubs.mower_set_motor_on, because on the topic
they carry the same callerid as this module's.
"""
import threading
import time

import rospy
from std_msgs.msg import Bool, String
from vitulus_msgs.msg import Mower

DEFAULTS = {
    'grace_s': 2.0, 'period_s': 1.0, 'stable_s': 5.0, 'backoff': 4.0, 'stable_max_s': 300.0,
    'window_s': 600.0, 'spinup_s': 30.0, 'on_rpm': 300, 'rpm_tol': 100, 'fresh_s': 1.0,
    'escalate_s': 600.0,
}
MOWER_FAULTS = ('ERR', 'BLOCKED', 'TEMP')
GUARD = '/map_rules_guard'
TERMINAL_REASON = 'map_rules_frozen'     # home_latch reason, mission.rules_terminal


class StaleBlade(object):

    def __init__(self, params=None):
        self.p = dict(DEFAULTS)
        self.p.update(params or {})
        self.status, self.rpm, self.setpoint = None, 0, 0
        self.busy, self.program, self.stop_reason = None, '', ''
        self.commanded = None         # the last command from anybody but this class
        self.owed = False             # stopped by this class, a True is due: hold
        self.deferred = False         # ... behind the noblade area: no hold
        self.owed_program = ''        # to this program, '' = its name not known yet
        self.failed = False
        self.since = None             # stale since
        self.frozen_t = None          # stale during a program since
        self.fresh_t = None           # fresh since, within a grace
        self.back_t = None            # limit fresh since
        self.spin_t = None            # True sent at
        self.last_off = None
        self.last_true = None
        self.restarts = []

    def on_mower(self, status, rpm, setpoint):
        self.status, self.rpm, self.setpoint = status, int(rpm), int(setpoint)

    def on_program(self, busy=None, program=None, stop_reason=None):
        if busy is not None:
            self.busy = bool(busy)
        if program is not None:
            self.program = program
            name = program.strip()
            if self.owed_program and name != self.owed_program:
                self._drop()          # a True belongs to the program it was taken from
            elif self.owed or self.deferred:
                self.owed_program = name
        if stop_reason is not None:
            self.stop_reason = stop_reason

    def on_command(self, value, now=None, guard=False):
        """The mission or another node commanded the blade.  True = send False
        once more, it may have crossed our True."""
        if guard and not value:
            return False
        self.commanded = bool(value)
        if value:
            self.failed = False
            self.deferred = False
            return False
        self._drop()
        crossed = (now is not None and self.last_true is not None
                   and now - self.last_true < 1.0)
        if crossed:
            self.last_true = None
        return crossed

    def _drop(self):
        self.owed = self.deferred = False
        self.owed_program = ''
        self.spin_t = None

    def escalate_due(self, now):
        return self.frozen_t is not None and now - self.frozen_t >= self.p['escalate_s']

    def _runs(self):
        return bool(self.busy) or bool(self.program.strip())

    def _up(self):
        if self.status != 'RUN':
            return False
        if self.setpoint > 0:
            return abs(self.rpm - self.setpoint) < self.p['rpm_tol']
        return self.rpm >= self.p['on_rpm']

    def _settle_s(self, now):
        self.restarts = [t for t in self.restarts if now - t < self.p['window_s']]
        return min(self.p['stable_max_s'],
                   self.p['stable_s'] * self.p['backoff'] ** len(self.restarts))

    def tick(self, now, stale, mux_held=False, noblade=False):
        """The commands to publish now."""
        if stale:
            return self._stale(now)
        if self.fresh_t is None:
            self.fresh_t = now
        if now - self.fresh_t >= self.p['fresh_s']:
            self.since = None
            self.frozen_t = None
        if not (self.owed or self.deferred):
            return []
        if not self._runs() or self.stop_reason.strip():
            self._drop()
            return []
        if mux_held:
            self.back_t = None
            return []
        if self.deferred:
            return self._behind_the_area(now, noblade)
        if noblade:
            # A True sent already was taken by the guard: then the debt is its own.
            self.deferred = self.spin_t is None
            self.owed = False
            self.spin_t = None
            self.back_t = None
            if not self.deferred:
                self.owed_program = ''
            return []
        return self._resume(now)

    def _behind_the_area(self, now, noblade):
        if noblade or self.status == 'TEMP':
            self.back_t = None
            return []
        if self.status in MOWER_FAULTS or self.status == 'RUN' or self.rpm >= self.p['on_rpm']:
            self._drop()              # the mission's handling, or it is on already
            self.back_t = None
            return []
        if self.back_t is None:
            self.back_t = now
        if now - self.back_t < self._settle_s(now):
            return []
        self.back_t = None
        self.deferred = False
        self.owed = True
        return self._start(now)

    def _start(self, now):
        self.restarts.append(now)
        self.spin_t = self.last_true = now
        return [True]

    def _stale(self, now):
        self.fresh_t = None
        self.back_t = None
        self.spin_t = None            # a restart under way is off again below
        if self.since is None:
            self.since = now
        if not self._runs():
            self.frozen_t = None
        elif self.frozen_t is None:
            self.frozen_t = now
        if now - self.since < self.p['grace_s'] or not self._runs():
            return []
        if self.status == 'RUN' and self.commanded is not False:
            if not self.owed:
                self.owed_program = self.program.strip()
            self.owed = True
        if self.status != 'RUN':
            return []                 # already off in the firmware, maybe coasting
        if self.last_off is not None and now - self.last_off < self.p['period_s']:
            return []
        self.last_off = now
        return [False]

    def _resume(self, now):
        if self.spin_t is not None:
            if self._up() or self.status in MOWER_FAULTS:
                self._drop()          # a fault is the mission's own BLOCKED handling
            elif now - self.spin_t >= self.p['spinup_s']:
                self._drop()
                self.failed = True
            return []
        if self.status == 'TEMP':
            self.back_t = None        # the firmware refuses a start
            return []
        if self.status in MOWER_FAULTS:
            self._drop()              # the mission's own BLOCKED handling
            self.back_t = None
            return []
        if self.back_t is None:
            self.back_t = now
        if now - self.back_t < self._settle_s(now):
            return []
        return self._start(now)


class _Tracked(object):
    """pubs.mower_set_motor_on: tells the watch what the mission commands."""

    def __init__(self, publisher, heard):
        self.publisher = publisher
        self.heard = heard

    def publish(self, msg):
        again = self.heard(msg.data)
        self.publisher.publish(msg)
        if again:
            self.publisher.publish(Bool(False))


class StaleBladeWatch(object):
    def __init__(self, pubs, hold, rules, params=None):
        self.core = StaleBlade(params)
        self.lock = threading.Lock()
        self.pubs = pubs
        self.hold = hold
        self.rules = rules
        self.raw = pubs.mower_set_motor_on
        pubs.mower_set_motor_on = _Tracked(self.raw, self.on_mission)
        self.own = rospy.get_name()
        self.guard = rospy.get_param('~guard_callerid', GUARD)
        self.latch = None             # ReturnHomeLatch, set by the node
        self.escalated = False
        rospy.Subscriber('/mower/status', Mower, self._mower, queue_size=1)
        rospy.Subscriber('/mower/set_motor_on', Bool, self._command, queue_size=5)
        rospy.Subscriber('/mower_smach/busy', Bool,
                         lambda m: self._program(busy=m.data), queue_size=1)
        rospy.Subscriber('/mower_smach/active_program', String,
                         lambda m: self._program(program=m.data or ''), queue_size=1)
        rospy.Subscriber('/mower_smach/stop_reason', String,
                         lambda m: self._program(stop_reason=m.data or ''), queue_size=1)
        hold.start()
        rospy.Timer(rospy.Duration(0.2), self.tick)

    def on_mission(self, value, guard=False):
        with self.lock:
            return self.core.on_command(value, time.monotonic(), guard)

    def _mower(self, msg):
        with self.lock:
            self.core.on_mower(msg.status, msg.moto_rpm, msg.setpoint_rpm)

    def _command(self, msg):
        hdr = getattr(msg, '_connection_header', None) or {}
        caller = hdr.get('callerid')
        if caller != self.own and self.on_mission(msg.data, caller == self.guard):
            self.raw.publish(Bool(False))     # this node's are seen by _Tracked

    def _program(self, **kw):
        with self.lock:
            self.core.on_program(**kw)

    def _escalate(self, due):
        if not due:
            self.escalated = False
        elif not self.escalated and self.latch is not None:
            self.escalated = self.latch.request(TERMINAL_REASON)
            if self.escalated:
                rospy.logerr('[STALE_BLADE] map rules stopped the robot for good: terminal')

    def tick(self, _evt):
        try:
            with self.lock:
                commands = self.core.tick(time.monotonic(), self.hold.limit_stale(),
                                          self.hold.mux_held(), self.rules.noblade_here())
                for value in commands:
                    self.raw.publish(Bool(value))
                owed, failed = self.core.owed, self.core.failed
                escalate = self.core.escalate_due(time.monotonic())
            self.hold.set_blade_wait(owed)
            self.rules.set_local_failed(failed)
            self._escalate(escalate)
            for value in commands:
                text = ('Blade back on, map rules guard is back' if value
                        else 'Blade off: map rules guard silent')
                rospy.logwarn('[STALE_BLADE] %s', text)
                self.pubs.log_info.publish(String(text))
        except Exception as exc:  # noqa: BLE001 - a dead timer would hide it for good
            rospy.logerr('[STALE_BLADE] tick failed: %r', exc)
