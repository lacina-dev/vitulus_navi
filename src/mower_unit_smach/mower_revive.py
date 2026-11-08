"""The mower board is silent when the mission switches it on (T24, 8. 11. 2026).

Test run 8. 11. 15:46: the board's MCU stopped talking 2 s into the undock (last
/mower/status 15:46:40.99) and stayed silent for 141 s.  POWER_ON_MOWER sent
/mower/set_power True, which travels over that very link (the firmware asks the PM
for bat_out itself), so nothing came on.  WAIT_FOR_MOWER_ON timed out, the mission
ended as critical with reason UNKNOWN, and the master's mower_link recovery did not
apply.

Only for a board that is silent (no status in mower_status_now) when it is to be
switched on:

    pm          the power module, another device on another port, switches bat_out:
                off for off_s first when it is on or unknown, then on (sent again
                until it reads on).  Up to revive_s for a status.  A status: the usual
                /mower/set_power True follows, the firmware's own reset, READY in
                ~18 s.  None: bat_out off again, no board vouches for the blade
                (mower_link.py's rule), and the mission goes to its retry.
    retry       once: the whole power-on again, WAIT_FOR_MOWER_ON included.
    home        still silent: home_latch 'mower_link' -> stop_reason
                interrupted:mower_link, RETURN_TO_DOCK, the checkpoint stays and the
                master's recovery applies (wait, resume, hw_reset where allowed).

The PM is used only for a blade that cannot turn: the board's last status is HOME,
READY or CHANGE_HEIGHT (SAFE; never OFF or INIT, see SAFE), nothing in this mission sent set_motor_on True and mower_link is not
armed.  A hung MCU holds its PWM (mower_link.py): otherwise the old way, unchanged.
A board that answers is switched on exactly as before.  A STOP inside the wait
ends it at once, bat_out off again when no board has answered.

Data 8. 11.: bat_out is not the MCU's supply (it runs with bat_out off, see
hw_reset.py).  The board answered 5.2 s after somebody's bat_out on and ~0.5 s after
the dock contact came back: which of the two woke it is not proven.  The retry and
the mower_link reason do not depend on it.
"""
import time

import rospy
import smach
from std_msgs.msg import Bool, Int16, String

from .chain import PROGRAM_CHAIN
from .helpers import (POWER_EVIDENCE_MAX_AGE_S, STATUS_SILENT_S, mark_run_phase,
                      mower_status_now, power_watch, run_flags)
from .mower_link import LINK, REASON as REASON_LINK

REVIVE_S = 20.0     # bat_out on -> status; 8. 11.: 5 s, a cold MCU boots in ~2.5 s
OFF_S = 2.0         # bat_out off before on, when it was on or unknown
RESEND_S = 1.0      # bat_out command again until it reads back
RESEND_MAX = 3
STEP_S = 0.2
# The blade does not turn.  Not OFF or INIT: the firmware reports them over a running
# motor too (motor_on is not cleared when bat_out goes off), so bat_out on could spin
# the blade at the dock.  Those keep the old way, the retry and mower_link.
SAFE = ('HOME', 'READY', 'CHANGE_HEIGHT')


def may_switch(last_status, commanded, link_armed):
    """bat_out from the mission only for a board that cannot turn the blade."""
    return not link_armed and commanded is not True and last_status in SAFE


def revive(io, revive_s=REVIVE_S, off_s=OFF_S):
    """Switch bat_out through the PM and wait for the board.

    io: now(), sleep(dt) -> False on preempt, status_t() (time of the last status or
    None), bat_out() (fresh read-back or None), set_bat(value).
    Returns 'alive', 'silent' or 'preempted'.
    """
    start = io.now()

    def answered():
        t = io.status_t()
        return t is not None and t > start

    def drive(value, budget):
        """Send value, again at RESEND_S until it reads back; status ends it early."""
        io.set_bat(value)
        sent, last, t0 = 1, io.now(), io.now()
        while io.now() - t0 < budget:
            if answered():
                return 'alive'
            if (io.bat_out() is not value and sent < RESEND_MAX
                    and io.now() - last >= RESEND_S):
                io.set_bat(value)
                sent, last = sent + 1, io.now()
            if not io.sleep(STEP_S):
                return 'preempted'
        return None

    switched = False
    try:
        if io.bat_out() is not False:
            out = drive(False, off_s)
            if out is not None:
                return out
        switched = True
        out = drive(True, revive_s)
        if out is not None:
            return out
        return 'silent'
    finally:
        if switched and not answered():
            io.set_bat(False)       # no board vouches for the blade: the rail stays off


class _RosIO(object):
    def __init__(self, state, watch, pub):
        self.state, self.watch, self.pub = state, watch, pub

    def now(self):
        return self.watch.clock()

    def sleep(self, dt):
        if self.state.preempt_requested():
            return False
        rospy.sleep(dt)
        return not self.state.preempt_requested()

    def status_t(self):
        with self.watch.lock:
            last = self.watch.last_status
        return last[0] if last is not None else None

    def bat_out(self):
        with self.watch.lock:
            bat = self.watch.last_bat_out
        if bat is None or self.watch.clock() - bat[0] > POWER_EVIDENCE_MAX_AGE_S:
            return None
        return bat[1]

    def set_bat(self, value):
        rospy.logwarn('[POWER_ON_MOWER] /pm/set_bat_out_switch %s', value)
        self.pub.publish(Bool(value))


def last_status(watch):
    if watch is None:
        return None
    with watch.lock:
        last = watch.last_status
    return last[1].status if last is not None else None


def link_state():
    """(armed or in a ladder, the last set_motor_on of the mission)."""
    core = LINK.core
    if core is None:
        return False, None
    with LINK.lock:
        return bool(core.armed or core.active), core.commanded


def board_silent(watch=None):
    """No /mower/status for STATUS_SILENT_S.  No watch: unknown, not silent."""
    watch = watch if watch is not None else power_watch()
    silent_s = watch.silent_s() if watch is not None else None
    return silent_s is not None and silent_s >= STATUS_SILENT_S


class PowerOnMower(smach.State):
    """POWER_ON_MOWER, and once more after a silent board (POWER_ON_MOWER_RETRY).

    done       the board answered, or the PM may not be used: /mower/set_power True
               sent as always -> WAIT_FOR_MOWER_ON
    silent     silent, the PM brought nothing: bat_out off again
    """

    def __init__(self, pubs, label='POWER_ON_MOWER', revive_s=REVIVE_S, off_s=OFF_S):
        smach.State.__init__(self, outcomes=['done', 'silent', 'preempted'],
                             input_keys=['program'], output_keys=['program'])
        self.pubs = pubs
        self.label = label
        self.revive_s = revive_s
        self.off_s = off_s
        self.pub_bat = rospy.Publisher('/pm/set_bat_out_switch', Bool, queue_size=2)

    def execute(self, ud):
        mark_run_phase(ud.program, 'on_mower')
        status = mower_status_now()
        if PROGRAM_CHAIN.board_stays_on(status):
            rospy.loginfo('[%s] chain: board stays on (%s)', self.label, status)
            return 'done'
        if status is None:
            out = self._revive()
            if out == 'preempted':
                self.service_preempt()
                return 'preempted'
            if out == 'silent':
                return 'silent'
        rospy.loginfo('[%s] Turning on mower', self.label)
        self.pubs.mower_set_power.publish(Bool(True))
        self.pubs.log_info.publish(String("Turn on mower"))
        self.pubs.smach_status.publish(String("Turn on mower"))
        self.pubs.pm_play_melody.publish(Int16(1))
        rospy.sleep(2.5)
        return 'done'

    def _revive(self):
        """'alive', 'silent', 'preempted', or 'refused': the old way goes on."""
        watch = power_watch()
        said = last_status(watch)
        armed, commanded = link_state()
        if watch is None or not may_switch(said, commanded, armed):
            rospy.logwarn('[%s] mower board silent, power module not used (last status '
                          '%s, blade command %s, link armed %s)', self.label, said,
                          commanded, armed)
            return 'refused'
        rospy.logwarn('[%s] mower board silent: bat_out through the power module, '
                      'up to %.0f s for its status', self.label, self.revive_s)
        # Lines the UI already translates (vt_autonomy.js LINES): no new text.
        self.pubs.log_info.publish(String("No mower status: robot waits"))
        self.pubs.smach_status.publish(String("Turn on mower"))
        t0 = time.monotonic()
        out = revive(_RosIO(self, watch, self.pub_bat), revive_s=self.revive_s,
                     off_s=self.off_s)
        rospy.logwarn('[%s] power module revive: %s after %.1f s', self.label, out,
                      time.monotonic() - t0)
        return out


class MowerOnTimeout(smach.State):
    """WAIT_FOR_MOWER_ON timed out: a silent board goes on (retry or home), a board
    that answers ends the mission as before (critical)."""

    def __init__(self, label):
        smach.State.__init__(self, outcomes=['silent', 'abort'])
        self.label = label

    def execute(self, ud):
        if board_silent():
            rospy.logwarn('[%s] no READY and the mower board is silent', self.label)
            return 'silent'
        rospy.logerr('[%s] no READY from a board that answers: mission aborts', self.label)
        return 'abort'


class MowerLinkHome(smach.State):
    """Silent after the retry: home as interrupted:mower_link, not critical."""

    def __init__(self, pubs):
        smach.State.__init__(self, outcomes=['preempted', 'aborted'],
                             input_keys=['error_reason'], output_keys=['error_reason'])
        self.pubs = pubs

    def execute(self, ud):
        rospy.logerr('[MOWER_LINK_HOME] mower board silent after the retry: going home '
                     '(%s)', REASON_LINK)
        self.pubs.log_info.publish(String("Mower link not recovered: going home"))
        ud.error_reason = REASON_LINK
        latch = run_flags.home_latch
        if latch is not None and latch.request(REASON_LINK):
            deadline = time.monotonic() + 2.0
            while time.monotonic() < deadline and not rospy.is_shutdown():
                if self.preempt_requested():
                    self.service_preempt()
                    return 'preempted'
                rospy.sleep(0.05)
        return 'aborted'    # mission_outcome still reads the latch reason first
