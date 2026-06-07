"""
Recovery sub-state-machines for blocked motor and navigation failures.

Factory functions that return SMACH StateMachine objects ready to be
added to the process_path_sm.
"""
import rospy
import smach
import actionlib

from std_msgs.msg import Bool, String, Int16
from geometry_msgs.msg import Twist
from nav_msgs.msg import Path
from vitulus_msgs.msg import Mower
from mbf_msgs.msg import ExePathAction, ExePathGoal, RecoveryAction, RecoveryGoal

from .helpers import (MOWER_ERROR_STATES, get_robot_xy_in_map,
                      nearest_pose_index, mbf_outcome_str)
from .states import WaitForMowerStatus, RetryLimitedAction


# ===========================================================================
# BLOCKED motor recovery (4-phase cascade)
# ===========================================================================

def build_blocked_recovery_sm(pubs):
    """Build a sub-SM for recovering from BLOCKED motor status.

    Phases:
      1. In-place restart (2 attempts)
      2. Raise blade to max height + restart (2 attempts)
      3. Escape: move ~2m forward without blade, then try motor (1 attempt)
      4. Failure → critical_error

    Outcomes: 'recovered', 'critical_error', 'preempted'
    Input keys: zone_rpm, zone_cut_height, path, path_window_start_index
    Output keys: restore_height_pending
    """
    sm = smach.StateMachine(
        outcomes=['recovered', 'critical_error', 'preempted'],
        input_keys=['zone_rpm', 'zone_cut_height', 'path', 'path_window_start_index'],
        output_keys=['restore_height_pending']
    )
    sm.userdata.blocked_phase1_count = 0
    sm.userdata.blocked_phase2_count = 0
    sm.userdata.restore_height_pending = False

    with sm:

        # --- Phase 1: In-place motor restart (up to 2 attempts) ---

        @smach.cb_interface(outcomes=['done', 'preempted'])
        def phase1_stop_cb(ud):
            rospy.loginfo("[BLOCKED_RECOVERY] Phase 1: stopping motor")
            pubs.mower_set_motor_on.publish(Bool(False))
            rospy.sleep(1.0)
            return 'done'

        smach.StateMachine.add('PHASE1_STOP', smach.CBState(phase1_stop_cb),
                               transitions={'done': 'PHASE1_START',
                                            'preempted': 'preempted'})

        @smach.cb_interface(outcomes=['done', 'preempted'])
        def phase1_start_cb(ud):
            rospy.loginfo("[BLOCKED_RECOVERY] Phase 1: restarting motor")
            pubs.mower_set_motor_on.publish(Bool(True))
            rospy.sleep(1.0)
            return 'done'

        smach.StateMachine.add('PHASE1_START', smach.CBState(phase1_start_cb),
                               transitions={'done': 'PHASE1_WAIT_RUN',
                                            'preempted': 'preempted'})

        smach.StateMachine.add('PHASE1_WAIT_RUN',
                               WaitForMowerStatus('RUN', timeout=float(rospy.get_param(
                                   '~blocked_wait_run_timeout', 5.0))),
                               transitions={
                                   'reached': 'recovered',
                                   'error': 'PHASE1_RETRY',
                                   'timeout': 'PHASE1_RETRY',
                                   'preempted': 'preempted'
                               })

        smach.StateMachine.add('PHASE1_RETRY',
                               RetryLimitedAction('blocked_phase1_count',
                                                  max_retries=int(rospy.get_param(
                                                      '~blocked_phase1_retries', 2))),
                               transitions={
                                   'retry': 'PHASE1_STOP',
                                   'give_up': 'PHASE2_RAISE',
                                   'preempted': 'preempted'
                               })

        # --- Phase 2: Height raise + restart (up to 2 attempts) ---

        @smach.cb_interface(outcomes=['done', 'preempted'])
        def phase2_raise_cb(ud):
            rospy.loginfo("[BLOCKED_RECOVERY] Phase 2: raising blade to max height")
            pubs.mower_set_motor_on.publish(Bool(False))
            rospy.sleep(0.5)
            # Read actual max_height from FW (avoid hardcoded value)
            max_height = 80
            try:
                s = rospy.wait_for_message('/mower/status', Mower, timeout=3.0)
                if s.max_height:
                    max_height = int(s.max_height)
            except rospy.ROSException:
                rospy.logwarn("[BLOCKED_RECOVERY] Phase 2: cannot read max_height, using 80")
            pubs.mower_set_height.publish(Int16(max_height))
            rospy.sleep(1.0)
            return 'done'

        smach.StateMachine.add('PHASE2_RAISE', smach.CBState(phase2_raise_cb),
                               transitions={'done': 'PHASE2_WAIT_HEIGHT',
                                            'preempted': 'preempted'})

        smach.StateMachine.add('PHASE2_WAIT_HEIGHT',
                               WaitForMowerStatus('READY', timeout=float(rospy.get_param(
                                   '~blocked_wait_height_timeout', 60.0))),
                               transitions={
                                   'reached': 'PHASE2_START_MOTOR',
                                   'error': 'PHASE2_RETRY',
                                   'timeout': 'PHASE2_RETRY',
                                   'preempted': 'preempted'
                               })

        @smach.cb_interface(outcomes=['done', 'preempted'])
        def phase2_start_motor_cb(ud):
            rospy.loginfo("[BLOCKED_RECOVERY] Phase 2: starting motor at max height")
            pubs.mower_set_motor_on.publish(Bool(True))
            rospy.sleep(1.0)
            return 'done'

        smach.StateMachine.add('PHASE2_START_MOTOR', smach.CBState(phase2_start_motor_cb),
                               transitions={'done': 'PHASE2_WAIT_RUN',
                                            'preempted': 'preempted'})

        smach.StateMachine.add('PHASE2_WAIT_RUN',
                               WaitForMowerStatus('RUN', timeout=float(rospy.get_param(
                                   '~blocked_wait_run_timeout', 5.0))),
                               transitions={
                                   'reached': 'PHASE2_RECOVERED',
                                   'error': 'PHASE2_RETRY',
                                   'timeout': 'PHASE2_RETRY',
                                   'preempted': 'preempted'
                               })

        @smach.cb_interface(input_keys=['zone_cut_height'],
                            output_keys=['restore_height_pending'],
                            outcomes=['done'])
        def phase2_recovered_cb(ud):
            # Motor running at max height. Schedule height restore after some movement.
            rospy.loginfo("[BLOCKED_RECOVERY] Phase 2 success: motor running at max height. "
                          "Height will be restored after next chunk.")
            ud.restore_height_pending = True
            return 'done'

        smach.StateMachine.add('PHASE2_RECOVERED', smach.CBState(phase2_recovered_cb),
                               transitions={'done': 'recovered'})

        smach.StateMachine.add('PHASE2_RETRY',
                               RetryLimitedAction('blocked_phase2_count',
                                                  max_retries=int(rospy.get_param(
                                                      '~blocked_phase2_retries', 2))),
                               transitions={
                                   'retry': 'PHASE2_RAISE',
                                   'give_up': 'PHASE3_ESCAPE',
                                   'preempted': 'preempted'
                               })

        # --- Phase 3: Escape ~2m forward without blade (1 attempt) ---

        class Phase3Escape(smach.State):
            """Extract ~2m of path ahead and drive forward without cutting."""

            ESCAPE_POSES = 67  # ~2m at 3cm spacing
            MOTOR_WAIT_TIMEOUT = 10.0  # seconds to wait for motor RUN

            def __init__(self):
                smach.State.__init__(self,
                                     outcomes=['motor_ok', 'motor_failed', 'nav_failed', 'preempted'],
                                     input_keys=['path', 'path_window_start_index'])
                # Tunables (defaults = the class constants above).
                self.ESCAPE_POSES = int(rospy.get_param(
                    '~blocked_escape_poses', self.ESCAPE_POSES))
                self.MOTOR_WAIT_TIMEOUT = float(rospy.get_param(
                    '~blocked_escape_motor_wait_timeout', self.MOTOR_WAIT_TIMEOUT))
                self._client = actionlib.SimpleActionClient(
                    '/move_base_flex/exe_path', ExePathAction)
                self._client.wait_for_server(rospy.Duration(5.0))

            def execute(self, userdata):
                if self.preempt_requested():
                    self.service_preempt()
                    return 'preempted'

                rospy.loginfo("[BLOCKED_RECOVERY] Phase 3: escape forward ~2m")
                pubs.mower_set_motor_on.publish(Bool(False))
                rospy.sleep(0.5)

                # Find actual robot position via TF (M2 fix)
                full_path = userdata.path
                hint_idx = int(userdata.path_window_start_index or 0)
                robot_xy = get_robot_xy_in_map(timeout=0.5)
                if robot_xy is not None:
                    start_idx = nearest_pose_index(
                        full_path, robot_xy[0], robot_xy[1], hint_index=hint_idx)
                    start_idx = max(start_idx, hint_idx)
                else:
                    rospy.logwarn("[BLOCKED_RECOVERY] Phase 3: TF unavailable, using window index")
                    start_idx = hint_idx

                end_idx = min(start_idx + self.ESCAPE_POSES, len(full_path.poses))

                if end_idx <= start_idx:
                    rospy.logwarn("[BLOCKED_RECOVERY] Phase 3: no poses for escape")
                    return 'nav_failed'

                escape_path = Path()
                escape_path.header = full_path.header
                escape_path.poses = full_path.poses[start_idx:end_idx]

                # Drive forward
                goal = ExePathGoal()
                goal.path = escape_path
                self._client.send_goal(goal)

                # Wait for completion with preempt check
                while self._client.get_state() in [
                        actionlib.GoalStatus.PENDING, actionlib.GoalStatus.ACTIVE]:
                    if self.preempt_requested():
                        self._client.cancel_goal()
                        self.service_preempt()
                        return 'preempted'
                    rospy.sleep(0.2)

                result = self._client.get_result()
                if result is None or result.outcome != 0:
                    rospy.logwarn("[BLOCKED_RECOVERY] Phase 3: escape navigation failed")
                    return 'nav_failed'

                # Try motor after moving — wait properly for RUN status (M3 fix)
                rospy.loginfo("[BLOCKED_RECOVERY] Phase 3: trying motor after escape")
                pubs.mower_set_motor_on.publish(Bool(True))
                deadline = rospy.Time.now() + rospy.Duration(self.MOTOR_WAIT_TIMEOUT)
                while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                    if self.preempt_requested():
                        self.service_preempt()
                        return 'preempted'
                    try:
                        msg = rospy.wait_for_message('/mower/status', Mower, timeout=1.0)
                        if msg.status == 'RUN':
                            rospy.loginfo("[BLOCKED_RECOVERY] Phase 3: motor running after escape")
                            return 'motor_ok'
                        if msg.status in MOWER_ERROR_STATES:
                            rospy.logwarn("[BLOCKED_RECOVERY] Phase 3: motor error %s", msg.status)
                            return 'motor_failed'
                    except rospy.ROSException:
                        continue
                rospy.logwarn("[BLOCKED_RECOVERY] Phase 3: motor did not reach RUN in %.0fs",
                              self.MOTOR_WAIT_TIMEOUT)
                return 'motor_failed'

        smach.StateMachine.add('PHASE3_ESCAPE', Phase3Escape(),
                               transitions={
                                   'motor_ok': 'recovered',
                                   'motor_failed': 'PHASE4_FAIL',
                                   'nav_failed': 'PHASE4_FAIL',
                                   'preempted': 'preempted'
                               })

        # --- Phase 4: Terminal failure ---

        @smach.cb_interface(outcomes=['done'])
        def phase4_fail_cb(ud):
            rospy.logerr("[BLOCKED_RECOVERY] All phases failed. "
                         "Probable physical damage or tangled wire.")
            pubs.mower_set_motor_on.publish(Bool(False))
            pubs.log_info.publish(String("BLOCKED recovery failed - critical"))
            return 'done'

        smach.StateMachine.add('PHASE4_FAIL', smach.CBState(phase4_fail_cb),
                               transitions={'done': 'critical_error'})

    return sm


# ===========================================================================
# Navigation failure recovery (wait → skip → critical)
# ===========================================================================

def build_nav_recovery_sm(pubs):
    """Build a sub-SM for recovering from navigation failures.

    This handles the *dynamic* obstacle case (something that may clear by
    itself): blade off, wait for the self-decaying obstacle maps to clear,
    clear the costmap, optionally back up, and retry the SAME chunk.

    If the obstacle persists, it does NOT skip the whole line — it returns
    'detour_needed' so the caller can route AROUND the obstacle and rejoin the
    line downstream (DetourAroundObstacle).

    Outcomes: 'recovered', 'detour_needed', 'critical_error', 'preempted'
    Input keys: path_chunk, consecutive_nav_failures, zone_rpm
    Output keys: consecutive_nav_failures
    """
    sm = smach.StateMachine(
        outcomes=['recovered', 'detour_needed', 'critical_error', 'preempted'],
        input_keys=['path_chunk', 'consecutive_nav_failures', 'zone_rpm'],
        output_keys=['consecutive_nav_failures']
    )
    sm.userdata.nav_retry_count = 0

    with sm:

        # --- Phase 1: Blade off + wait for dynamic obstacle to clear ---

        class NavBladeOffWait(smach.State):
            """Stop blade and wait (preemptable) for a dynamic obstacle to clear.

            The obstacle source maps (lidar/cloud) self-decay within a few
            seconds, so a short wait lets a passing person/animal disappear from
            the costmap before we retry the same chunk.
            """

            WAIT_SECONDS = 6

            def __init__(self):
                smach.State.__init__(self, outcomes=['done', 'preempted'])
                self.WAIT_SECONDS = int(rospy.get_param(
                    '~nav_blade_off_wait_s', self.WAIT_SECONDS))

            def execute(self, userdata):
                rospy.loginfo("[NAV_RECOVERY] Stopping blade, waiting %ds", self.WAIT_SECONDS)
                pubs.mower_set_motor_on.publish(Bool(False))
                for _ in range(self.WAIT_SECONDS):
                    if self.preempt_requested():
                        self.service_preempt()
                        return 'preempted'
                    if rospy.is_shutdown():
                        return 'preempted'
                    rospy.sleep(1.0)
                return 'done'

        smach.StateMachine.add('NAV_BLADE_OFF_WAIT', NavBladeOffWait(),
                               transitions={'done': 'NAV_CLEAR_COSTMAP',
                                            'preempted': 'preempted'})

        # Clear costmap before retry
        class ClearCostmap(smach.State):
            def __init__(self):
                smach.State.__init__(self, outcomes=['done', 'preempted'])
                self._client = actionlib.SimpleActionClient(
                    'move_base_flex/recovery', RecoveryAction)
                self._client.wait_for_server(rospy.Duration(5.0))

            def execute(self, userdata):
                if self.preempt_requested():
                    self.service_preempt()
                    return 'preempted'
                goal = RecoveryGoal()
                goal.behavior = 'clear_costmap'
                self._client.send_goal(goal)
                self._client.wait_for_result(rospy.Duration(10.0))
                return 'done'

        smach.StateMachine.add('NAV_CLEAR_COSTMAP', ClearCostmap(),
                               transitions={'done': 'NAV_RETRY_EXE',
                                            'preempted': 'preempted'})

        # Retry the same path chunk
        class RetryPathChunk(smach.State):
            def __init__(self):
                smach.State.__init__(self,
                                     outcomes=['succeeded', 'aborted', 'preempted'],
                                     input_keys=['path_chunk'])
                self._client = actionlib.SimpleActionClient(
                    '/move_base_flex/exe_path', ExePathAction)
                self._client.wait_for_server(rospy.Duration(5.0))

            def execute(self, userdata):
                if self.preempt_requested():
                    self.service_preempt()
                    return 'preempted'
                goal = ExePathGoal()
                goal.path = userdata.path_chunk
                rospy.loginfo("[NAV_REC] retrying same chunk (%d poses)",
                              len(goal.path.poses) if goal.path.poses else 0)
                self._client.send_goal(goal)
                while self._client.get_state() in [
                        actionlib.GoalStatus.PENDING, actionlib.GoalStatus.ACTIVE]:
                    if self.preempt_requested():
                        self._client.cancel_goal()
                        self.service_preempt()
                        return 'preempted'
                    rospy.sleep(0.2)
                result = self._client.get_result()
                outcome = getattr(result, 'outcome', None) if result else None
                rospy.loginfo("[NAV_REC] chunk retry outcome=%s", mbf_outcome_str(outcome))
                if outcome == 0:
                    return 'succeeded'
                return 'aborted'

        smach.StateMachine.add('NAV_RETRY_EXE', RetryPathChunk(),
                               transitions={
                                   'succeeded': 'NAV_BLADE_ON',
                                   'aborted': 'NAV_RETRY_GATE',
                                   'preempted': 'preempted'
                               })

        @smach.cb_interface(input_keys=['consecutive_nav_failures'],
                            output_keys=['consecutive_nav_failures'],
                            outcomes=['done'])
        def blade_on_cb(ud):
            pubs.mower_set_motor_on.publish(Bool(True))
            rospy.sleep(1.0)
            # NOTE: consecutive_nav_failures is intentionally NOT reset here.
            # It is reset only after a *genuine* full-chunk completion in
            # ExecutePathWithFeedback. Resetting on every recovery retry let the
            # robot loop forever against a re-appearing map ghost (each tiny nudge
            # reset the escalation counter, so it never reached skip/critical).
            rospy.loginfo("[NAV_RECOVERY] Path retry succeeded, blade on")
            return 'done'

        smach.StateMachine.add('NAV_BLADE_ON', smach.CBState(blade_on_cb),
                               transitions={'done': 'recovered'})

        # Physical backup maneuver: reverse a short distance to break free of a
        # real obstacle and re-seed the local planner before retrying. Publishes
        # directly to /cmd_vel (the base command topic) — no MBF goal is active
        # during recovery, so the topic is free. Blind reverse, kept short/slow.
        class NavBackup(smach.State):
            BACKUP_SPEED = 0.1       # m/s magnitude (reverse)
            BACKUP_DISTANCE = 0.4    # m
            RATE_HZ = 10.0

            def __init__(self):
                smach.State.__init__(self, outcomes=['done', 'preempted'])
                # Tunables (defaults = the class constants above).
                self.BACKUP_SPEED = float(rospy.get_param(
                    '~nav_backup_speed', self.BACKUP_SPEED))
                self.BACKUP_DISTANCE = float(rospy.get_param(
                    '~nav_backup_distance', self.BACKUP_DISTANCE))
                self._cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

            def execute(self, userdata):
                if self.preempt_requested():
                    self.service_preempt()
                    return 'preempted'
                rospy.loginfo("[NAV_RECOVERY] Backing up %.2fm at %.2fm/s",
                              self.BACKUP_DISTANCE, self.BACKUP_SPEED)
                pubs.mower_set_motor_on.publish(Bool(False))
                twist = Twist()
                twist.linear.x = -abs(self.BACKUP_SPEED)
                duration = self.BACKUP_DISTANCE / self.BACKUP_SPEED
                rate = rospy.Rate(self.RATE_HZ)
                deadline = rospy.Time.now() + rospy.Duration(duration)
                while not rospy.is_shutdown() and rospy.Time.now() < deadline:
                    if self.preempt_requested():
                        self._cmd_pub.publish(Twist())  # stop
                        self.service_preempt()
                        return 'preempted'
                    self._cmd_pub.publish(twist)
                    rate.sleep()
                self._cmd_pub.publish(Twist())  # stop
                rospy.sleep(0.3)
                return 'done'

        smach.StateMachine.add('NAV_BACKUP', NavBackup(),
                               transitions={'done': 'NAV_CLEAR_COSTMAP',
                                            'preempted': 'preempted'})

        # When the quick retries are exhausted the obstacle is not clearing by
        # itself -> hand off to DETOUR (route around it and rejoin the line)
        # instead of abandoning the whole line here.
        smach.StateMachine.add('NAV_RETRY_GATE',
                               RetryLimitedAction('nav_retry_count',
                                                  max_retries=int(rospy.get_param(
                                                      '~nav_retry_max', 2))),
                               transitions={
                                   'retry': 'NAV_BACKUP',
                                   'give_up': 'detour_needed',
                                   'preempted': 'preempted'
                               })

    return sm
