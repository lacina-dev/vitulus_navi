"""The mower board stops talking, or stops the blade, during a mission (1. 11. 2026).

Incident 21:35: /mower/status ended at 21:35:06.5, the blade's load left the battery
two seconds later, and the mission drove paths 2 and 3 of 7 for four minutes on a
stale mower=RUN.  The MCU never answered again, not even a new serial_node.

Armed: from the mission's first blade True until it ends or anybody sends
/mower/set_power (the board restarts on purpose then, 14 s of silence).  Rule: no
mowing path with a blade nobody can vouch for.

    silent      no /mower/status for silent_s, armed
    stopped     status fresh, last command True, spinup_s since it, rpm under the
                limit for rpm_low_s, and none of: a map rule, TEMP, ERR, BLOCKED, any
                other hold.  They have their own handling and stop the blade on
                purpose.  A False from anybody means off: no trigger, no True.

A gap first: after silent_s without status the robot only holds.  Status back within
gap_s of the last one (load, a slow bus): released, the blade was never touched and
nothing counts against max_recoveries.  Blade commands are not latched: a False sent
into the silence may never arrive.  So with the mission's last word False and a blade
that turns, the first status gets False again, at 1 Hz, and the hold stands until it
has stopped, attempt_s at most.  In a gap as in `board`.

Trigger: hold (HOLD and /cmd_vel_mux/hold, so all software stops, also a goal that
is already running; people still drive), blade False.  Then a ladder, DEFAULTS in
seconds, within attempt_s of the trigger; a silence counts from the start of its gap:

    resync      resync_s: serial_node asks the board every 15 s by itself, and an
                MCU reset by its watchdog (8 s) answers that.  Nothing else to do.
    serial      restart serial_node through /mower/link_cmd.  Proven not to cure what happened on 1. 11., cheap, kept for
                a dead node.  serial_s is short: a new node syncs within 2 s, and the
                next step is the only one that surely stops a blade on a hung MCU,
                whose PWM holds the last duty (1. 11., 17:20).  No log of this robot
                has ever shown FW_VERSION, so the firmware's own link stop and
                watchdog cannot be counted on.
    power       /pm/set_bat_out_switch False, repeated at rail_s until it reads
                False, at most rail_max times.  /mower/set_power cannot do this: it
                travels over the link that is gone.  The PM is another device on
                another port.  It is the one action here that does not depend on the
                MCU, and the blade has no power after it.  The rail stays off until
                status proves a board: a hung MCU holds its PWM, a True here would
                spin a blade on a standing robot.  A live MCU reports OFF with
                bat_out off, so the proof needs no power.  Silent through attempt_s:
                `failed` with the rail off.
    board       status back, whenever that happens above.  An MCU that has lost
                bat_out needs /mower/set_power True, the firmware's own restart.
                READY: height and rpm as they were at the last good RUN, then True.
                RUN at speed: released, the mission goes on from where it stands,
                the span is re-mowed (states.ExecutePathWithFeedback).
                After the link's False a RUN proves nothing (coasting, or the False
                not read yet): only a True of the link counts.
                Not up in blade_s: /mower/set_power True once (status is known, the
                blade stands: the known reset), then blade_s again.  False goes
                before every set_power: the rail comes up under a motor_on the MCU
                may still hold.  Nothing latches set_power: OFF after settle_s, sent
                once more.

attempt_s over, or max_recoveries used in window_s: `failed`.  Blade off, bat_out
off if the board is silent, home_latch 'mower_link' and then the hold released:
stop_reason interrupted:mower_link, RETURN_TO_DOCK, checkpoint kept, master resumes
after a wait.

STOP, or any end of the mission, ends the ladder where it stands and releases the
hold; with no status at that moment the blade is unknown, so bat_out goes off first.
/mower/set_power from anybody else (safety_node's stop) ends it too: released with
status, `failed` without, the mission must not drive on over an unknown blade.
A mission that ends in the first silent_s of a silence is watched on: the status
comes back, or the blade loses its power.
bat_out going False that this class did not order is a person: no set_power over it,
the mission goes home without the blade, stop_reason interrupted:mower_power_off, and
master does not resume: that power stays off until a person starts a program.  own_cut
ends with the set_power that brings the rail back, never on a PM report: one sent
before the cut arrives after it.  ERR/BLOCKED/TEMP during `board`: released, that is
the mission's own handling.

MowerLink is pure.  MowerLinkShell is the ROS side.
"""
import collections
import json
import threading
import time

import rospy
from std_msgs.msg import Bool, Int16, String
from vitulus_msgs.msg import Mower, Power_status

from .rule_gate import RULES

DEFAULTS = {
    'silent_s': 3.0, 'gap_s': 6.0, 'rpm_low_s': 6.0, 'spinup_s': 15.0, 'on_rpm': 300, 'rpm_tol': 100,
    'rpm_max': 7000, 'resync_s': 18.0, 'serial_s': 5.0, 'rail_s': 1.0, 'rail_max': 10,
    'attempt_s': 90.0, 'blade_s': 12.0, 'max_recoveries': 3, 'window_s': 1800.0,
    'height_tol': 3, 'settle_s': 2.5,
}
REASON = 'mower_link'           # home_latch reason, interrupted:mower_link
REASON_PERSON = 'mower_power_off'   # interrupted:mower_power_off: no resume
SOURCE = 'mower_link'           # /cmd_vel_mux/hold source
FAULTS = ('ERR', 'BLOCKED', 'TEMP')
REBOOTING = ('INIT', 'HOME')


class MowerLink(object):

    def __init__(self, params=None):
        self.p = dict(DEFAULTS)
        self.p.update(params or {})
        self.armed = False
        self.commanded = None       # last set_motor_on of anybody but this class
        self.true_t = None
        self.status, self.rpm, self.setpoint, self.height = None, 0, 0, 0
        self.seen_t = None          # last /mower/status
        self.good_t = None          # ... with the blade at speed
        self.good_height, self.good_setpoint = None, None
        self.low_t = None
        self.bat_out = None
        self.phase = None           # None | 'gap' | 'resync' | 'serial' | 'power_off' |
        #                             'board'
        self.why = None             # 'gap' | 'silent' | 'stopped'
        self.since = None           # trigger
        self.lost_t = None          # the blade was last known up
        self.stage_t = None
        self.powered = False        # bat_out was cycled in this attempt
        self.own_cut = False        # bat_out False is ours
        self.rail_t, self.rail_n = None, 0
        self.foreign_off = False    # bat_out went False and nobody here did that
        self.blade_cut = False      # the link sent False in this attempt
        self.span_due = False       # a span without blade waits for ExecutePath
        self.reset_sent = False
        self.reset_again = False    # set_power sent the second time
        self.reset_t = None
        self.off_t = None           # False repeated at, the mission's False was lost
        self.watch_seen = None      # mission ended in a silence: seen_t then
        self.height_sent = None
        self.spin_t = None          # True sent at
        self.failed = False
        self.cancelled = False
        self.history = collections.deque()    # triggers, by time

    @property
    def active(self):
        return self.phase is not None

    def fresh(self, now):
        return self.seen_t is not None and now - self.seen_t <= self.p['silent_s']

    def reset(self):
        """A new mission.  The history stays: it counts by time."""
        self.armed = False
        self.commanded = None
        self.true_t = None
        self.low_t = None
        self.failed = False
        self.good_t = None
        self.watch_seen = None

    def on_status(self, now, status, rpm, setpoint, height):
        self.seen_t = now
        self.status, self.rpm, self.setpoint, self.height = status, rpm, setpoint, height
        if self._up():
            self.good_t = now
            self.good_height, self.good_setpoint = height, setpoint

    def on_command(self, value, now):
        """/mower/set_motor_on by the mission or anybody else."""
        self.commanded = bool(value)
        if value:
            self.armed = True
            self.true_t = now
        self.low_t = None

    def on_bat_out(self, value):
        if value is False and self.bat_out is True and not self.own_cut:
            self.foreign_off = True
        self.bat_out = value

    def on_power(self):
        """/mower/set_power by somebody else: what the board does now is on purpose.
        safety_node's tilt and overtemperature stop goes this way: it ends a ladder
        too, or its OFF would be answered with a True."""
        self.armed = False
        self.low_t = None
        self.own_cut = False
        self.cancelled = self.active

    def _plausible(self):
        return 0 <= self.rpm <= self.p['rpm_max']

    def _up(self):
        if self.status != 'RUN' or not self._plausible():
            return False
        if self.setpoint > 0:
            return abs(self.rpm - self.setpoint) < self.p['rpm_tol']
        return self.rpm >= self.p['on_rpm']

    def _limit(self):
        half = self.setpoint / 2.0 if self.setpoint > 0 else 0.0
        return max(self.p['on_rpm'], half)

    def _stopped(self, now, free):
        """The blade should turn and it does not, for rpm_low_s."""
        if (not free or self.commanded is not True or self.status in FAULTS
                or self.true_t is None or now - self.true_t < self.p['spinup_s']
                or not self._plausible() or self.rpm >= self._limit()):
            self.low_t = None
            return False
        if self.low_t is None:
            self.low_t = now
        return now - self.low_t >= self.p['rpm_low_s']

    def abort(self, now):
        """The mission ended (STOP, rain, battery ...) or was never there.  Nobody
        looks after the blade any more: with no status it loses its power."""
        actions = []
        if self.active:
            if not self.fresh(now):
                actions += ['blade_off'] + self._cut(now)
            actions.append('release')
        elif self.armed and self.seen_t is not None:
            if self.fresh(now):
                self.watch_seen = self.seen_t   # maybe the start of a silence: _watch
            else:
                actions += ['blade_off'] + self._cut(now)
        self.foreign_off = False
        self.phase = None
        self.armed = False
        self.low_t = None
        return actions

    def tick(self, now, running, free=True, rule_held=False, blocked=False):
        """Actions due now, in order.

        running    the mission child runs
        free       no rule, TEMP or other hold stops the blade on purpose
        rule_held  a map rule holds the blade: no True, the guard gives it back
        blocked    a foreign hold or stale limit: no True yet, the clock waits
        """
        return self._rail(now) + self._step(now, running, free, rule_held, blocked)

    def _cut(self, now):
        self.own_cut = True
        self.rail_t, self.rail_n = now, 1
        return ['bat_off']

    def _rail(self, now):
        """The PM may have missed it.  Bounded: it may also be gone."""
        if self.rail_t is None:
            return []
        if self.bat_out is False or self.rail_n >= self.p['rail_max']:
            self.rail_t = None
            return []
        if now - self.rail_t < self.p['rail_s']:
            return []
        self.rail_t = now
        self.rail_n += 1
        return ['bat_off']

    def _watch(self, now):
        """After a mission that ended inside silent_s: nobody else will look."""
        if self.watch_seen is None:
            return []
        if self.seen_t != self.watch_seen:
            self.watch_seen = None
            return []
        if self.fresh(now):
            return []
        self.watch_seen = None
        return ['blade_off'] + self._cut(now)

    def _step(self, now, running, free, rule_held, blocked):
        if not running:
            return self.abort(now) + self._watch(now)
        if self.foreign_off:
            self.foreign_off = False
            if self.armed or self.active:
                return self._person_off()
        if self.cancelled:
            self.cancelled = False
            return self._done() if self.fresh(now) else self._fail(now)
        if self.phase is not None:
            return self._recover(now, rule_held, blocked)
        if self.failed or not self.armed:
            return []
        if self.seen_t is None or not self.fresh(now):
            if not free and self.status == 'TEMP':
                return []           # temp_watch decides a silence during TEMP
            if self.seen_t is None:
                return self._trigger(now, 'silent')
            return self._gap(now)
        if self._stopped(now, free):
            return self._trigger(now, 'stopped')
        return []

    def _gap(self, now):
        self.phase, self.why = 'gap', 'gap'
        self.since = self.stage_t = now
        self.lost_t = self.good_t if self.good_t is not None else self.seen_t
        return ['hold']

    def _trigger(self, now, why):
        start = self.since if self.phase == 'gap' else now
        self.why = why
        self.lost_t = self.good_t if self.good_t is not None else self.seen_t
        self.low_t = None
        while self.history and now - self.history[0] > self.p['window_s']:
            self.history.popleft()
        self.history.append(now)
        self.since = self.stage_t = start
        self.powered = self.reset_sent = self.reset_again = False
        self.blade_cut = why == 'silent'
        self.span_due = True
        self.reset_t = self.off_t = None
        self.height_sent = None
        self.spin_t = None
        self.phase = 'resync' if why == 'silent' else 'board'
        actions = ['hold', 'blade_off'] if why == 'silent' else ['hold']
        if len(self.history) > self.p['max_recoveries']:
            return actions + self._fail(now)
        return actions

    def _fail(self, now):
        # A blade that turns against the False is not released to the way home.
        cut = not self.fresh(now) or self._turns_against_order()
        self.phase = None
        self.failed = True
        self.armed = False
        return ['blade_off'] + (self._cut(now) if cut else []) + ['home', 'release']

    def _person_off(self):
        """Somebody took the power away during mowing: it stays away."""
        active = self.active
        self.phase = None
        self.failed = True
        self.armed = False
        return ['blade_off', 'home_off'] + (['release'] if active else [])

    def _recover(self, now, rule_held, blocked):
        if self.phase == 'gap':
            if self.fresh(now):
                if self._turns_against_order():
                    if now - self.since >= self.p['attempt_s']:
                        return self._fail(now)
                    return self._off_again(now)
                return self._done()
            return self._trigger(now, 'silent') if now - self.seen_t >= self.p['gap_s'] else []
        if now - self.since >= self.p['attempt_s']:
            return self._fail(now)
        if self.fresh(now):
            self.phase = 'board'
            return self._board(now, rule_held, blocked)
        if self.phase == 'board':
            self.phase = 'resync'               # gone again, the ladder from the top
            self.stage_t = now
        return self._ladder(now)

    def _ladder(self, now):
        waited = now - self.stage_t
        if self.phase == 'resync' and waited >= self.p['resync_s']:
            self.phase, self.stage_t = 'serial', now
            return ['blade_off', 'restart_serial']
        if self.phase == 'serial' and waited >= self.p['serial_s']:
            self.phase, self.stage_t = 'power_off', now
            self.powered = True
            return ['blade_off'] + self._cut(now)
        return []

    def _turns_against_order(self):
        """The mission's last word is False and the board says the blade turns."""
        return self.commanded is False and (self.status == 'RUN' or (
            self._plausible() and self.rpm >= self.p['on_rpm']))

    def _off_again(self, now):
        if self.off_t is not None and now - self.off_t < self.p['rail_s']:
            return []
        self.off_t = now
        return ['blade_off']

    def _done(self):
        self.phase = None
        self.low_t = self.off_t = None
        return ['release']

    def _board(self, now, rule_held, blocked):
        if self.powered and not self.reset_sent:
            return self._reset(now)     # the rail is off and the board has proved itself
        if self.status in FAULTS:
            return self._done()         # the mission's own handling
        if self.reset_t is not None and now - self.reset_t < self.p['settle_s']:
            return []                   # the old status, the restart is on its way
        if self.status == 'OFF':
            if not self.reset_sent:     # the firmware's own start, it is listening
                return self._reset(now)
            if not self.reset_again:    # not latched, it may have gone nowhere
                self.reset_again = True
                return self._reset(now)
            return []
        if self.status in REBOOTING or self.status == 'CHANGE_HEIGHT':
            return []
        if self._turns_against_order():
            return self._off_again(now)
        if self.commanded is False or rule_held:
            return self._restore_height() or self._done()
        if self._up() and (not self.blade_cut or self.spin_t is not None):
            return self._done()
        if blocked:
            self.since += 0.5           # tick period: this wait is not ours
            return []
        if self.spin_t is not None:
            if now - self.spin_t < self.p['blade_s']:
                return []
            self.spin_t = None
            if self.reset_sent:
                return self._fail(now)
            return self._reset(now)     # status known, blade at rest: the known reset
        if self.status != 'READY' and self.status != 'RUN':
            return []
        again = self._restore_height()
        if again:
            return again
        if self.height_sent is not None and not self._at_height(self.height_sent):
            return []
        self.spin_t = now
        rpm = self.good_setpoint if self.good_setpoint else self.setpoint
        return ([('rpm', rpm)] if rpm else []) + ['blade_on']

    def _reset(self, now):
        self.reset_sent = True
        self.reset_t = now
        self.spin_t = None
        self.rail_t = None              # no more bat_off: it would undo this
        self.own_cut = False
        return ['blade_off', 'set_power']

    def _at_height(self, height):
        return abs(self.height - height) <= self.p['height_tol']

    def _restore_height(self):
        """A restarted board stands at max height."""
        height = self.good_height
        if height is None or self._at_height(height) or self.height_sent == height:
            return []
        self.height_sent = height
        return [('height', height)]

    def report(self, now):
        out = {'phase': self.phase, 'why': self.why, 'failed': self.failed,
               'armed': self.armed, 'recoveries': len(self.history)}
        if self.active:
            out['since_s'] = round(now - self.since, 1)
            out['left_s'] = max(0, int(self.p['attempt_s'] - (now - self.since)))
        return out


class _Heard(object):
    """pubs.mower_set_motor_on: what the mission commands.  On the topic it has the
    same callerid as this module.  A True in a ladder (a detour that ends, a recovery)
    is remembered and not sent: the board is not vouched for, `board` sends it."""

    def __init__(self, publisher, heard, busy):
        self.publisher = publisher
        self.heard = heard
        self.busy = busy

    def publish(self, msg):
        self.heard(msg.data)
        if msg.data and self.busy():
            rospy.logwarn('[MOWER_LINK] mission True kept back')
            return
        self.publisher.publish(msg)


class MowerLinkShell(object):

    PERIOD_S = 0.5

    def __init__(self):
        self.lock = threading.Lock()
        self.core = None
        self.pubs = None
        self.hold = None
        self.latch = None
        self.temp = None
        self.stale = None           # StaleBladeWatch: its True goes first
        self.raw = None             # the blade topic past the mission's wrappers
        self.tracked = None         # ... past ours only: a True stale_blade must know
        self.own = None
        self.held = False
        self.said = None
        self.path_xy = collections.deque(maxlen=120)     # (t, x, y) of the mission

    def attach(self, pubs, hold, latch, temp_watch):
        if self.core is not None:
            return
        self.pubs = pubs
        self.hold = hold
        self.latch = latch
        self.temp = temp_watch
        self.core = MowerLink(rospy.get_param('~mower_link', {}))
        self.own = rospy.get_name()
        self.tracked = pubs.mower_set_motor_on    # stale_blade hears what goes through
        self.raw = pubs.mower_set_motor_on
        while hasattr(self.raw, 'publisher'):
            self.raw = self.raw.publisher     # past stale_blade's wrapper
        pubs.mower_set_motor_on = _Heard(pubs.mower_set_motor_on, self.on_mission, self.ladder)
        self.pub_hold = rospy.Publisher('/cmd_vel_mux/hold', String, queue_size=5)
        self.pub_cmd = rospy.Publisher('/mower/link_cmd', String, queue_size=2)
        self.pub_bat = rospy.Publisher('/pm/set_bat_out_switch', Bool, queue_size=2)
        self.pub_state = rospy.Publisher('/mower_smach/link', String, latch=True,
                                         queue_size=1)
        rospy.Subscriber('/mower/status', Mower, self._status, queue_size=5)
        rospy.Subscriber('/mower/set_motor_on', Bool, self._command, queue_size=5)
        rospy.Subscriber('/mower/set_power', Bool, self._power, queue_size=5)
        rospy.Subscriber('/pm/power_status', Power_status, self._pm, queue_size=1)
        self._leftover_hold()
        rospy.Timer(rospy.Duration(self.PERIOD_S), self._tick)

    def _leftover_hold(self):
        """A hold never times out: a node restarted in one must take it back."""
        try:
            holds = rospy.get_param('/cmd_vel_mux/holds', {}) or {}
        except Exception:  # noqa: BLE001 - no parameter server, no hold to find
            return
        if SOURCE in holds:
            rospy.sleep(0.5)        # the mux needs to hear it
            self._send_hold(False)

    def reset(self):
        with self.lock:
            if self.core is not None:
                self.core.reset()
            self.path_xy.clear()

    def on_mission(self, value):
        with self.lock:
            self.core.on_command(value, time.monotonic())

    def _status(self, msg):
        with self.lock:
            self.core.on_status(time.monotonic(), msg.status, msg.moto_rpm,
                                msg.setpoint_rpm, msg.current_height)

    def _command(self, msg):
        hdr = getattr(msg, '_connection_header', None) or {}
        if hdr.get('callerid') != self.own:
            self.on_mission(msg.data)

    def _power(self, msg):
        hdr = getattr(msg, '_connection_header', None) or {}
        with self.lock:
            # Ours and the mission's have one callerid, and during a ladder only ours
            # can come: the mission waits in the hold.
            if hdr.get('callerid') != self.own or not self.core.active:
                self.core.on_power()

    def _pm(self, msg):
        with self.lock:
            self.core.on_bat_out(bool(msg.bat_out_switch))

    def _send_hold(self, on):
        self.pub_hold.publish(String(json.dumps(
            {'source': SOURCE, 'hold': on, 'reason': 'mower link lost', 'data': {}})))

    def _tick(self, _evt):
        try:
            running = self.latch.running()
            rule = RULES.noblade_here() or (self.stale is not None and self.stale.deferred())
            blocked = self.hold.foreign_hold() or self.hold.limit_stale()
            with self.temp.lock:
                temp_hold = self.temp.core is not None and self.temp.core.hold
            free = not (rule or blocked or temp_hold or self.hold.blade_wait())
            now = time.monotonic()
            with self.lock:
                actions = self.core.tick(now, running, free, rule, blocked)
                state = self.core.report(now)
            self._publish(state)        # before a False: map_rules_guard reads it
            for action in actions:
                self._do(action)
            if self.held:
                self._send_hold(True)   # the mux's keepalive
            elif self.hold.own_hold():
                self._send_hold(False)  # a release the mux has not heard
        except Exception as exc:  # noqa: BLE001 - a dead timer would hide it for good
            rospy.logerr('[MOWER_LINK] tick failed: %r', exc)

    def _publish(self, state):
        text = json.dumps(state, sort_keys=True)
        # Over and over while it matters: a latched state of a dead smach looks alive,
        # and the supervisor needs to hear this node before the first phase.
        if text != self.said or state.get('phase') or state.get('armed') or state.get('failed'):
            self.said = text
            self.pub_state.publish(String(text))

    def _do(self, action):
        if isinstance(action, tuple):
            kind, value = action
            rospy.logwarn('[MOWER_LINK] restoring %s=%s', kind, value)
            pub = self.pubs.mower_set_height if kind == 'height' else self.pubs.mower_set_rpm
            pub.publish(Int16(int(value)))
        elif action == 'hold':
            self.held = True
            self.hold.set_link_wait(True)
            self._send_hold(True)
            text = 'Mower link lost: robot stops, blade unknown'
            if self.core.why == 'gap':
                text = 'No mower status: robot waits'
            elif self.core.why == 'stopped':
                text = 'Blade stopped during mowing: robot stops'
            rospy.logerr('[MOWER_LINK] %s', text)
            self.pubs.log_info.publish(String(text))
            self.pubs.smach_status.publish(String('Held: mower link'))
        elif action == 'release':
            self.held = False
            self._send_hold(False)
            self.hold.set_link_wait(False)
            rospy.logwarn('[MOWER_LINK] hold released')
        elif action == 'blade_off':
            self.raw.publish(Bool(False))
        elif action == 'blade_on':
            rospy.logwarn('[MOWER_LINK] board answers: blade on')
            self.tracked.publish(Bool(True))
        elif action == 'restart_serial':
            rospy.logwarn('[MOWER_LINK] restarting serial_node')
            self.pub_cmd.publish(String('restart_serial'))
        elif action == 'bat_off':
            rospy.logwarn('[MOWER_LINK] bat_off')
            self.pub_bat.publish(Bool(False))
        elif action == 'set_power':
            rospy.logwarn('[MOWER_LINK] /mower/set_power True: restarting the board')
            self.pubs.mower_set_power.publish(Bool(True))
        elif action == 'home':
            rospy.logerr('[MOWER_LINK] recovery failed: mission goes home without blade')
            self.pubs.log_info.publish(String('Mower link not recovered: going home'))
            self.latch.request(REASON)
        elif action == 'home_off':
            rospy.logerr('[MOWER_LINK] mower power switched off by a person: going home')
            self.pubs.log_info.publish(String('Mower power switched off: going home'))
            self.latch.request(REASON_PERSON)

    # -- the mission's side ---------------------------------------------------
    def active(self):
        with self.lock:
            return self.core is not None and self.core.active

    def ladder(self):
        """A ladder, not the short wait of a gap."""
        with self.lock:
            return self.core is not None and self.core.active and self.core.phase != 'gap'

    def span_due(self):
        with self.lock:
            return self.core is not None and self.core.span_due

    def span_kept(self):
        with self.lock:
            if self.core is not None:
                self.core.span_due = False

    def silent(self):
        with self.lock:
            return (self.core is not None and self.core.armed
                    and self.core.seen_t is not None
                    and not self.core.fresh(time.monotonic()))

    def settle(self):
        """Until a gap has an answer: a healed one owes the mission nothing."""
        while self.in_gap() and not rospy.is_shutdown():
            rospy.sleep(0.1)

    def in_gap(self):
        with self.lock:
            return self.core is not None and self.core.phase == 'gap'

    def note_xy(self, x, y):
        with self.lock:
            self.path_xy.append((time.monotonic(), x, y))

    def lost_xy(self):
        """Where the robot was when the blade was last known up, None if not seen."""
        with self.lock:
            lost = self.core.lost_t if self.core is not None else None
            if lost is None:
                return None
            for t, x, y in reversed(self.path_xy):
                if t <= lost:
                    return x, y
            return (self.path_xy[0][1], self.path_xy[0][2]) if self.path_xy else None

    def shutdown(self):
        if self.held:
            self._send_hold(False)


LINK = MowerLinkShell()
