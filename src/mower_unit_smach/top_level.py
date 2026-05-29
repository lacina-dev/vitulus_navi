"""
Top-level SMACH states: CriticalError, TerminalError, and RETURN_TO_DOCK sub-SM.
"""
import rospy
import smach
import smach_ros

from std_msgs.msg import Bool, String, Int16, Int8
from vitulus_msgs.msg import DockProgram

from .helpers import safe_blade_shutdown
from .states import WaitForTopic, WaitForDockedState, CheckForDockPoint


# ===========================================================================
# CRITICAL_ERROR state (decision router)
# ===========================================================================

class CriticalErrorState(smach.State):
    """Route critical errors: either try docking or go directly to TERMINAL.

    If the error reason is NAVIGATION_ABORTED or DOCKING_* (meaning the robot
    can't navigate/dock reliably), skip docking and go terminal.
    For other reasons (e.g. BLOCKED_FAILED), attempt one more dock.
    """
    NO_DOCK_REASONS = ('NAVIGATION_ABORTED', 'DOCKING_TIMEOUT', 'DOCKING_FAILED')

    def __init__(self, pubs):
        smach.State.__init__(self,
                             outcomes=['try_dock', 'terminal', 'preempted'],
                             input_keys=['error_reason'],
                             output_keys=['error_reason'])
        self.pubs = pubs

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        reason = userdata.error_reason or 'UNKNOWN'
        rospy.logerr("[CRITICAL_ERROR] Reason: %s", reason)
        self.pubs.log_info.publish(String("CRITICAL ERROR: {}".format(reason)))
        self.pubs.smach_status.publish(String("Critical error"))
        self.pubs.stop_reason.publish(String("critical:{}".format(reason.lower())))
        self.pubs.pm_play_melody.publish(Int16(1))

        if reason in self.NO_DOCK_REASONS:
            rospy.logerr("[CRITICAL_ERROR] Cannot dock safely (reason=%s) -> TERMINAL", reason)
            return 'terminal'
        rospy.logwarn("[CRITICAL_ERROR] Attempting return to dock...")
        return 'try_dock'


# ===========================================================================
# TERMINAL_ERROR state (quarantine with buzzer + reset monitor)
# ===========================================================================

class TerminalErrorState(smach.State):
    """Quarantine state: robot stays put, beeps periodically, waits for reset.

    Listens to /mower_smach/reset (std_msgs/Bool). On True → returns 'reset'
    to re-enter WAIT_FOR_PROGRAM. The buzzer fires every BUZZER_INTERVAL seconds.
    """
    BUZZER_INTERVAL = 30.0

    def __init__(self, pubs):
        smach.State.__init__(self,
                             outcomes=['reset', 'preempted'],
                             input_keys=['error_reason'],
                             output_keys=['error_reason'])
        self.pubs = pubs

    def execute(self, userdata):
        reason = userdata.error_reason or 'UNKNOWN'
        rospy.logerr("[TERMINAL_ERROR] Quarantine. Reason: %s", reason)
        self.pubs.log_info.publish(String(
            "TERMINAL: {}. Reset via /mower_smach/reset".format(reason)))
        self.pubs.smach_status.publish(String("TERMINAL ERROR"))
        self.pubs.stop_reason.publish(String("terminal:{}".format(reason.lower())))

        # Ensure blade is safe
        safe_blade_shutdown(source='TERMINAL_ERROR', pubs=self.pubs)
        self.pubs.mower_set_power.publish(Bool(False))

        # Initial buzz
        self.pubs.pm_play_melody.publish(Int16(1))
        last_buzzer = rospy.Time.now()

        while not rospy.is_shutdown():
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            # Check for reset
            try:
                msg = rospy.wait_for_message('/mower_smach/reset', Bool, timeout=1.0)
                if msg.data:
                    rospy.loginfo("[TERMINAL_ERROR] Reset received")
                    self.pubs.log_info.publish(String("Reset accepted. Returning to idle."))
                    self.pubs.smach_status.publish(String("Ready"))
                    userdata.error_reason = ''
                    return 'reset'
            except rospy.ROSException:
                pass

            # Periodic buzzer
            if (rospy.Time.now() - last_buzzer).to_sec() >= self.BUZZER_INTERVAL:
                self.pubs.pm_play_melody.publish(Int16(1))
                last_buzzer = rospy.Time.now()

        # rospy shutdown
        return 'preempted'


# ===========================================================================
# RETURN_TO_DOCK sub-SM
# ===========================================================================

def build_return_to_dock_sm(pubs):
    """Build the RETURN_TO_DOCK sub-SM.

    Sequence:
      1. Safe blade shutdown + mower power off
      2. Check for DOCK point
      3. Get dock program
      4. Send dock program
      5. Wait for docked

    Outcomes: 'succeeded', 'failed', 'preempted'
    """
    sm = smach.StateMachine(
        outcomes=['succeeded', 'failed', 'preempted'],
        input_keys=['error_reason'],
        output_keys=['error_reason']
    )
    sm.userdata.dock_program = None
    sm.userdata.error_reason = ''

    with sm:

        # --- Safe shutdown ---
        @smach.cb_interface(outcomes=['done', 'preempted'])
        def safe_shutdown_cb(ud):
            rospy.loginfo("[RETURN_TO_DOCK] Safe blade shutdown")
            pubs.log_info.publish(String("Returning to dock..."))
            pubs.smach_status.publish(String("Returning to dock"))
            safe_blade_shutdown(source='RETURN_TO_DOCK', pubs=pubs)
            pubs.mower_set_power.publish(Bool(False))
            rospy.sleep(2.0)
            return 'done'

        smach.StateMachine.add('SAFE_SHUTDOWN', smach.CBState(safe_shutdown_cb),
                               transitions={'done': 'CHECK_DOCK_POINT',
                                            'preempted': 'preempted'})

        # --- Check for dock point ---
        smach.StateMachine.add('CHECK_DOCK_POINT', CheckForDockPoint(pubs),
                               transitions={
                                   'dock_point_found': 'GET_DOCK_PROGRAM',
                                   'no_dock_point': 'failed',
                                   'preempted': 'preempted'
                               })

        # --- Get dock program ---
        def _on_dock_program(userdata, msg):
            userdata.dock_program = msg
            pubs.log_info.publish(String("Received dock program."))

        smach.StateMachine.add('GET_DOCK_PROGRAM',
                               WaitForTopic('/dock_manager/dock_program',
                                            DockProgram,
                                            predicate=lambda m: True,
                                            timeout=30.0,
                                            output_keys=['dock_program'],
                                            on_match=_on_dock_program),
                               transitions={
                                   'received': 'SEND_DOCK_PROGRAM',
                                   'timeout': 'failed',
                                   'preempted': 'preempted'
                               })

        # --- Send dock program ---
        @smach.cb_interface(input_keys=['dock_program'], outcomes=['done', 'preempted'])
        def send_dock_cb(ud):
            pub_dock = rospy.Publisher('/dock_smach/start_docking',
                                       DockProgram, queue_size=1, latch=True)
            pub_dock.publish(ud.dock_program)
            pubs.log_info.publish(String("Sent dock program."))
            rospy.sleep(1.0)
            return 'done'

        smach.StateMachine.add('SEND_DOCK_PROGRAM', smach.CBState(send_dock_cb),
                               transitions={'done': 'WAIT_FOR_DOCKED',
                                            'preempted': 'preempted'})

        # --- Wait for docked ---
        smach.StateMachine.add('WAIT_FOR_DOCKED', WaitForDockedState(pubs),
                               transitions={
                                   'succeeded': 'succeeded',
                                   'failed': 'failed',
                                   'timeout': 'failed',
                                   'preempted': 'preempted'
                               })

    return sm


# ===========================================================================
# Wrapped RETURN_TO_DOCK with STOP monitor
# ===========================================================================

def build_return_to_dock_with_stop(pubs):
    """Wrap RETURN_TO_DOCK_SM in a Concurrence with a STOP monitor.

    If /mower_smach/stop fires during docking, the whole thing is preempted
    and the caller transitions to TERMINAL_ERROR.

    Outcomes: 'succeeded', 'failed', 'stop_preempt', 'preempted'
    """
    from .monitors import stop_monitor_cb

    dock_sm = build_return_to_dock_sm(pubs)

    def child_term_cb(outcome_map):
        return True  # terminate all on first finish

    def outcome_cb(outcome_map):
        if outcome_map.get('STOP_MON') == 'invalid':
            return 'stop_preempt'
        dock_outcome = outcome_map.get('DOCK_SM')
        if dock_outcome == 'succeeded':
            return 'succeeded'
        if dock_outcome == 'failed':
            return 'failed'
        return 'preempted'

    cc = smach.Concurrence(
        outcomes=['succeeded', 'failed', 'stop_preempt', 'preempted'],
        default_outcome='preempted',
        child_termination_cb=child_term_cb,
        outcome_cb=outcome_cb,
        input_keys=['error_reason'],
        output_keys=['error_reason']
    )

    with cc:
        smach.Concurrence.add('DOCK_SM', dock_sm)
        smach.Concurrence.add('STOP_MON',
                              smach_ros.MonitorState('/mower_smach/stop',
                                                     Bool, stop_monitor_cb))
    return cc
