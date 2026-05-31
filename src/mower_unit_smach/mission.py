"""
Mission construction: WAIT_FOR_PROGRAM, MISSION_CHILD_SM, and MISSION_CONCURRENCE.

This module contains the factory functions that build the main mission flow
and wrap it with concurrent monitors.
"""
import rospy
import smach
import smach_ros
import actionlib

from std_msgs.msg import Bool, String, Int16, Int8
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Quaternion
from vitulus_msgs.msg import (
    DockProgram, PlannerProgram, Mower, Device_icon_status, Power_status,
    Navi_transform)
from weather_alert.msg import RainAlert
from mbf_msgs.msg import (
    ExePathAction, ExePathGoal, GetPathAction, RecoveryAction, RecoveryGoal)
from mbf_msgs.srv import CheckPath, CheckPathRequest
from rtabmap_msgs.msg import Info

from .helpers import (
    classify_mbf_result, safe_blade_shutdown, wait_for_mbf, MOWER_ERROR_STATES)
from .states import (
    VerifyAtPose, WaitForMowerStatus, WaitForTopic, RetryLimitedAction,
    GetZoneData, GetPathData, CheckDistance, WindowPlannerPath, TrimAndRetry,
    ExecutePathWithFeedback, WaitForRpmReached, WaitForMowerOff,
    CheckIfDockedState, WaitForPlannerLoaded, WaitForDockedState,
    CheckForDockPoint, DetourAroundObstacle)
from .monitors import (
    weather_monitor_cb, battery_monitor_cb, mower_temp_monitor_cb, stop_monitor_cb)
from .recovery import build_blocked_recovery_sm, build_nav_recovery_sm


# ===========================================================================
# WAIT_FOR_PROGRAM (Concurrence with two MonitorStates)
# ===========================================================================

def build_wait_for_program(pubs):
    """Build the WAIT_FOR_PROGRAM Concurrence container.

    Monitors two topics:
      - /web_plan/program_active (new program)
      - /web_plan/program_active_unfinished (resume after crash)
    """

    def reinit_userdata(userdata):
        userdata.path_window_start_index = 0
        userdata.path_chunk = None
        userdata.path_chunk_trim_count = 0
        userdata.unfinished_active = False
        userdata.restore_height_pending = False
        userdata.consecutive_nav_failures = 0

    def callback_program(userdata, msg):
        reinit_userdata(userdata)
        userdata.program = msg
        userdata.program.last_result = 'failed: on_init'
        rospy.loginfo('[MONITOR_PROGRAM_ACTIVE] New program: %s', msg.name)
        pubs.active_program.publish(String(msg.name))
        pubs.log_info.publish(String("Executing program: {}".format(msg.name)))
        pubs.pm_play_melody.publish(Int16(2))
        pubs.smach_status.publish(String("Initiating"))
        rospy.sleep(1.0)
        userdata.prg_start_time = rospy.Time.now()
        return False  # 'invalid' outcome → triggers Concurrence exit

    def callback_program_unfinished(userdata, msg):
        reinit_userdata(userdata)
        userdata.program = msg
        rospy.loginfo('[MONITOR_PROGRAM_UNFINISHED] Resume: %s last_result=%s',
                      msg.name, msg.last_result)

        last_result = msg.last_result.split(': ')
        if last_result[0] == 'failed':
            failure_payload = last_result[1] if len(last_result) > 1 else ''
            tag, _, rest = failure_payload.partition('-')
            if tag == 'on_path' and rest:
                parts = rest.rsplit('-', 2)
                if len(parts) == 3:
                    zone_name, path_idx_str, window_str = parts
                elif len(parts) == 2:
                    zone_name, path_idx_str = parts
                    window_str = '0'
                else:
                    zone_name = parts[0]
                    path_idx_str = '0'
                    window_str = '0'
                userdata.unfinished_zone = zone_name
                userdata.unfinished_path = path_idx_str
                try:
                    userdata.unfinished_window = int(window_str)
                except (TypeError, ValueError):
                    userdata.unfinished_window = 0
                userdata.unfinished_active = True

        userdata.program.last_result = 'failed: on_init'
        pubs.active_program.publish(String(msg.name))
        pubs.log_info.publish(String("Resuming program: {}".format(msg.name)))
        pubs.pm_play_melody.publish(Int16(2))
        pubs.smach_status.publish(String("Initiating unfinished"))
        rospy.sleep(1.0)
        userdata.prg_start_time = rospy.Time.now()
        return False

    def concurrence_outcome_cb(outcome_map):
        if (outcome_map.get('MONITOR_PROGRAM_ACTIVE') == 'invalid' or
                outcome_map.get('MONITOR_PROGRAM_UNFINISHED') == 'invalid'):
            return 'program_received'
        return 'preempted'

    all_keys = ['program', 'prg_start_time', 'unfinished_active',
                'unfinished_zone', 'unfinished_path', 'path_window_start_index',
                'path_chunk', 'unfinished_window', 'restore_height_pending',
                'consecutive_nav_failures']

    cc = smach.Concurrence(
        outcomes=['program_received', 'preempted'],
        default_outcome='preempted',
        child_termination_cb=lambda so: True,
        output_keys=all_keys,
        input_keys=all_keys,
        outcome_cb=concurrence_outcome_cb
    )

    with cc:
        smach.Concurrence.add('MONITOR_PROGRAM_ACTIVE',
                              smach_ros.MonitorState(
                                  '/web_plan/program_active', PlannerProgram,
                                  callback_program,
                                  output_keys=all_keys,
                                  input_keys=all_keys))

        smach.Concurrence.add('MONITOR_PROGRAM_UNFINISHED',
                              smach_ros.MonitorState(
                                  '/web_plan/program_active_unfinished', PlannerProgram,
                                  callback_program_unfinished,
                                  output_keys=all_keys,
                                  input_keys=all_keys))

    return cc


# ===========================================================================
# MISSION_CHILD_SM (the main mission flow)
# ===========================================================================

def build_mission_child_sm(pubs):
    """Build the mission child SM: undock → map → mow all zones → power off.

    Outcomes:
      - 'succeeded': all zones mowed, mower powered off
      - 'aborted': critical error (error_reason set in userdata)
      - 'preempted': external preemption from monitors
    """
    sm = smach.StateMachine(
        outcomes=['succeeded', 'aborted', 'preempted'],
        input_keys=['program', 'prg_start_time', 'unfinished_active',
                    'unfinished_zone', 'unfinished_path', 'unfinished_window',
                    'path_window_start_index', 'path_chunk', 'error_reason',
                    'restore_height_pending', 'consecutive_nav_failures'],
        output_keys=['program', 'prg_start_time', 'error_reason']
    )

    # Shared userdata (will be linked to parent in build_mission_concurrence)
    sm.userdata.path_plan = None
    sm.userdata.path = None
    sm.userdata.path_planner = None
    sm.userdata.path_start_pose = None
    sm.userdata.zone_cut_height = None
    sm.userdata.zone_rpm = None
    sm.userdata.zone_name = None
    sm.userdata.zone_start_pose = None
    sm.userdata.paths = None
    sm.userdata.path_cost = None
    sm.userdata.final_pose = None
    sm.userdata.path_chunk_trim_count = 0
    sm.userdata.retry_get_path_to_start = 0
    sm.userdata.retry_get_path_to_begin = 0
    sm.userdata.retry_exe_to_begin = 0
    sm.userdata.retry_planner = 0
    sm.userdata.consecutive_nav_failures = 0
    sm.userdata.restore_height_pending = False
    sm.userdata.error_reason = ''

    with sm:

        # ==================== UNDOCK ====================

        smach.StateMachine.add('CHECK_IF_DOCKED', CheckIfDockedState(pubs),
                               transitions={
                                   'proceed_to_undock': 'GET_UNDOCK_PROGRAM',
                                   'skip_undocking': 'LOAD_MAP',
                                   'wait': 'CHECK_IF_DOCKED',
                                   'preempted': 'preempted'
                               })

        def _on_undock_program(userdata, msg):
            userdata.undock_program = msg
            pubs.log_info.publish(String("Received undock program."))
            pubs.smach_status.publish(String("Undocking"))

        smach.StateMachine.add('GET_UNDOCK_PROGRAM',
                               WaitForTopic('/dock_manager/undock_program',
                                            DockProgram,
                                            predicate=lambda m: True,
                                            timeout=30.0,
                                            output_keys=['undock_program'],
                                            on_match=_on_undock_program),
                               transitions={'received': 'SEND_UNDOCK_PROGRAM',
                                            'timeout': 'LOAD_MAP',
                                            'preempted': 'preempted'})

        @smach.cb_interface(input_keys=['undock_program'], outcomes=['done', 'preempted'])
        def send_undock_cb(ud):
            pub = rospy.Publisher('/dock_smach/start_docking',
                                  DockProgram, queue_size=1, latch=True)
            pub.publish(ud.undock_program)
            pubs.log_info.publish(String("Sent undock program."))
            rospy.sleep(1.0)
            return 'done'

        smach.StateMachine.add('SEND_UNDOCK_PROGRAM', smach.CBState(send_undock_cb),
                               transitions={'done': 'WAIT_FOR_UNDOCKED',
                                            'preempted': 'preempted'})

        def _undocked_pred(m):
            return m.data in (3, 4)  # Undocked or Failed

        smach.StateMachine.add('WAIT_FOR_UNDOCKED',
                               WaitForTopic('/dock_smach/dock_status', Int8,
                                            predicate=_undocked_pred, timeout=180.0),
                               transitions={'received': 'LOAD_MAP',
                                            'timeout': 'LOAD_MAP',
                                            'preempted': 'preempted'})

        # ==================== LOAD MAP ====================

        @smach.cb_interface(input_keys=['program'],
                            outcomes=['success_indoor', 'success_outdoor', 'preempted'])
        def load_map_cb(ud):
            # Determine environment type from map_name (format: "name***env*INDOOR" or "...OUTDOOR")
            indoor = False
            try:
                env_part = ud.program.map_name.split('***env*')[1]
                indoor = (env_part == 'INDOOR')
            except (IndexError, AttributeError):
                rospy.logwarn('[LOAD_MAP] Cannot parse env type from map_name=%r; defaulting to outdoor',
                              ud.program.map_name)
            if indoor:
                pub = rospy.Publisher('/navi_manager/load_map_rtabmap',
                                      String, latch=True, queue_size=1)
            else:
                pub = rospy.Publisher('/navi_manager/load_map',
                                      String, latch=True, queue_size=1)
            rospy.sleep(1.0)
            pub.publish(String(ud.program.map_name))
            rospy.loginfo('[LOAD_MAP] map=%s indoor=%s', ud.program.map_name, indoor)
            pubs.log_info.publish(String("Loading the map"))
            pubs.smach_status.publish(String("Loading map"))
            rospy.sleep(4.0)
            if indoor:
                return 'success_indoor'
            return 'success_outdoor'

        smach.StateMachine.add('LOAD_MAP', smach.CBState(load_map_cb),
                               transitions={
                                   'success_indoor': 'WAIT_FOR_RTABMAP',
                                   'success_outdoor': 'WAIT_FOR_GPS_FIX',
                                   'preempted': 'preempted'
                               })

        # ==================== WAIT FOR RTABMAP / GPS ====================

        def _rtabmap_pred(msg):
            if msg.proximityDetectionId > 0:
                pubs.log_info.publish(String("Rtabmap is ready"))
                pubs.pm_play_melody.publish(Int16(1))
                pubs.smach_status.publish(String("Rtabmap ready"))
                rospy.sleep(0.5)
                return True
            return False

        smach.StateMachine.add('WAIT_FOR_RTABMAP',
                               WaitForTopic('/rtabmap/info', Info,
                                            predicate=_rtabmap_pred, timeout=180.0),
                               transitions={
                                   'received': 'WAIT_FOR_PLANNER',
                                   'timeout': 'aborted',
                                   'preempted': 'preempted'
                               })

        def _gps_pred(msg):
            """Check both GPS are fresh: status=SAT and both fused times < 3s."""
            if msg.status != 'SAT':
                return False
            try:
                values_str = msg.info.split(',')
                parsed = []
                for val in values_str:
                    val = val.strip().replace('e', '').replace('s', '')
                    if val.lower() == 'inf':
                        parsed.append(float('inf'))
                    else:
                        parsed.append(float(val))
                if len(parsed) < 2:
                    return False
                fix_time = parsed[0]   # fused_fix_time (position GPS)
                nav_time = parsed[1]   # fused_nav_time (heading GPS)
                if fix_time < 3.0 and nav_time < 3.0:
                    rospy.loginfo('[WAIT_FOR_GPS_FIX] Both GPS ready: fix=%.1fs, nav=%.1fs',
                                  fix_time, nav_time)
                    pubs.log_info.publish(String("Both GPS ready"))
                    pubs.smach_status.publish(String("GPS fix acquired"))
                    rospy.sleep(0.5)
                    return True
                rospy.loginfo_throttle(5, '[WAIT_FOR_GPS_FIX] Waiting: fix=%.1fs, nav=%.1fs',
                                       fix_time, nav_time)
            except Exception as e:
                rospy.logwarn('[WAIT_FOR_GPS_FIX] Error parsing odom_status: %s', e)
            return False

        smach.StateMachine.add('WAIT_FOR_GPS_FIX',
                               WaitForTopic('/nav_tf/odom_status',
                                            Navi_transform,
                                            predicate=_gps_pred, timeout=600.0),
                               transitions={
                                   'received': 'WAIT_FOR_PLANNER',
                                   'timeout': 'aborted',
                                   'preempted': 'preempted'
                               })

        # ==================== WAIT FOR PLANNER ====================

        smach.StateMachine.add('WAIT_FOR_PLANNER', WaitForPlannerLoaded(pubs, timeout=120.0),
                               transitions={
                                   'received': 'SET_PROGRAM_SPEED',
                                   'timeout': 'aborted',
                                   'preempted': 'preempted'
                               })

        # ==================== PROGRAM SPEED ====================
        # Apply the program-level driving speed the same way the webui speed
        # buttons do (publish on /navi_manager/speed). 'mid' is the default.
        @smach.cb_interface(input_keys=['program'], outcomes=['done', 'preempted'])
        def set_program_speed_cb(ud):
            speed_map = {'slow': 'SLOW', 'mid': 'MEDIUM', 'fast': 'FAST'}
            raw = (getattr(ud.program, 'speed', '') or 'mid').lower()
            speed = speed_map.get(raw, 'MEDIUM')
            rospy.loginfo('[SET_PROGRAM_SPEED] program speed=%r -> %s', raw, speed)
            pubs.navi_speed.publish(String(speed))
            pubs.log_info.publish(String("Set speed: {}".format(speed)))
            pubs.smach_status.publish(String("Set speed {}".format(speed)))
            rospy.sleep(0.5)
            return 'done'

        smach.StateMachine.add('SET_PROGRAM_SPEED', smach.CBState(set_program_speed_cb),
                               transitions={'done': 'POWER_ON_MOWER',
                                            'preempted': 'preempted'})

        # ==================== MOWER POWER ====================

        @smach.cb_interface(input_keys=['program'], output_keys=['program'],
                            outcomes=['done', 'preempted'])
        def power_on_cb(ud):
            ud.program.last_result = 'failed: on_mower'
            rospy.loginfo('[POWER_ON_MOWER] Turning on mower')
            pubs.mower_set_power.publish(Bool(True))
            pubs.log_info.publish(String("Turn on mower"))
            pubs.smach_status.publish(String("Turn on mower"))
            pubs.pm_play_melody.publish(Int16(1))
            rospy.sleep(2.5)
            return 'done'

        smach.StateMachine.add('POWER_ON_MOWER', smach.CBState(power_on_cb),
                               transitions={'done': 'WAIT_FOR_MOWER_ON',
                                            'preempted': 'preempted'})

        def _on_mower_ready(_msg):
            pubs.log_info.publish(String("Mower is ready"))
            pubs.smach_status.publish(String("Mower ready"))
            pubs.pm_play_melody.publish(Int16(1))
            return True

        smach.StateMachine.add('WAIT_FOR_MOWER_ON',
                               WaitForMowerStatus('READY', timeout=30.0,
                                                  extra_check=_on_mower_ready),
                               transitions={
                                   'reached': 'ZONE_IT',
                                   'error': 'aborted',
                                   'timeout': 'aborted',
                                   'preempted': 'preempted'
                               })

        # ==================== ZONE ITERATOR ====================

        zone_it = smach.Iterator(
            outcomes=['succeeded', 'preempted', 'aborted', 'critical_failure'],
            input_keys=['program', 'zone_cut_height', 'zone_rpm', 'zone_name',
                        'zone_start_pose', 'path_plan', 'paths',
                        'path_window_start_index', 'unfinished_active',
                        'unfinished_zone', 'unfinished_path', 'unfinished_window',
                        'retry_get_path_to_start', 'retry_get_path_to_begin',
                        'retry_exe_to_begin', 'retry_planner',
                        'consecutive_nav_failures', 'restore_height_pending',
                        'error_reason'],
            output_keys=['zone_cut_height', 'zone_rpm', 'zone_name',
                         'zone_start_pose', 'path_plan', 'paths',
                         'path_window_start_index', 'error_reason',
                         'consecutive_nav_failures'],
            it=lambda: range(0, len(sm.userdata.program.zone_list)),
            it_label='index',
            exhausted_outcome='succeeded')
        zone_it.userdata = sm.userdata

        with zone_it:
            process_zone_sm = _build_process_zone_sm(pubs, zone_it)
            smach.Iterator.set_contained_state(
                'PROCESS_ZONE', process_zone_sm,
                loop_outcomes=['continue'])

        smach.StateMachine.add('ZONE_IT', zone_it,
                               transitions={
                                   'succeeded': 'POWER_OFF_MOWER',
                                   'aborted': 'POWER_OFF_MOWER',
                                   'critical_failure': 'SET_ERROR_AND_ABORT',
                                   'preempted': 'preempted'
                               })

        # Set error_reason before aborting from critical failure
        @smach.cb_interface(input_keys=['error_reason'], output_keys=['error_reason'],
                            outcomes=['done'])
        def set_error_abort_cb(ud):
            if not ud.error_reason:
                ud.error_reason = 'MISSION_CRITICAL_FAILURE'
            return 'done'

        smach.StateMachine.add('SET_ERROR_AND_ABORT', smach.CBState(set_error_abort_cb),
                               transitions={'done': 'aborted'})

        # ==================== POWER OFF ====================

        @smach.cb_interface(input_keys=['program'], output_keys=['program'],
                            outcomes=['done', 'preempted'])
        def power_off_cb(ud):
            rospy.loginfo('[POWER_OFF_MOWER] Shutting down mower')
            safe_blade_shutdown(source='MISSION_POWER_OFF', pubs=pubs)
            pubs.mower_set_power.publish(Bool(False))
            pubs.show_map_layer.publish(String("SMACH|MAP|FULL"))
            rospy.sleep(1.5)
            ud.program.last_result = 'succeeded'
            pubs.stop_reason.publish(String("succeeded"))
            return 'done'

        smach.StateMachine.add('POWER_OFF_MOWER', smach.CBState(power_off_cb),
                               transitions={'done': 'WAIT_MOWER_OFF',
                                            'preempted': 'preempted'})

        smach.StateMachine.add('WAIT_MOWER_OFF', WaitForMowerOff(pubs, timeout=30.0),
                               transitions={
                                   'off': 'succeeded',
                                   'timeout': 'succeeded',
                                   'preempted': 'preempted'
                               })

    return sm


# ===========================================================================
# Process Zone sub-SM (nested inside ZONE_IT)
# ===========================================================================

def _build_process_zone_sm(pubs, zone_it):
    """Build the sub-SM for processing a single zone."""

    process_zone_sm = smach.StateMachine(
        outcomes=['succeeded', 'preempted', 'aborted', 'continue', 'critical_failure'],
        input_keys=['program', 'zone_cut_height', 'zone_rpm', 'index',
                    'zone_name', 'zone_start_pose', 'path_plan', 'paths',
                    'path_window_start_index', 'unfinished_active',
                    'unfinished_zone', 'unfinished_path', 'unfinished_window',
                    'retry_get_path_to_start', 'retry_get_path_to_begin',
                    'retry_exe_to_begin', 'retry_planner',
                    'consecutive_nav_failures', 'restore_height_pending',
                    'error_reason'],
        output_keys=['zone_cut_height', 'zone_rpm', 'zone_name',
                     'zone_start_pose', 'path_plan', 'paths',
                     'path_window_start_index', 'error_reason',
                     'consecutive_nav_failures'])
    process_zone_sm.userdata = zone_it.userdata

    with process_zone_sm:

        # --- GET_ZONE_DATA ---
        smach.StateMachine.add('GET_ZONE_DATA', GetZoneData(pubs),
                               transitions={
                                   'available': 'GET_PATH_TO_START',
                                   'failed': 'aborted',
                                   'preempted': 'preempted',
                                   'skip_zone': 'continue'
                               })

        # --- GET_PATH_TO_START ---
        def _get_path_result_cb(userdata, status, result):
            cls = classify_mbf_result('GET_PATH_TO_START', status, result,
                                       log_pub=pubs.log_info)
            if cls == 'succeeded':
                pubs.log_info.publish(String("Go to zone begin"))
                pubs.smach_status.publish(String("Go to zone begin"))
                pubs.pm_play_melody.publish(Int16(2))
            return cls

        smach.StateMachine.add('GET_PATH_TO_START',
                               smach_ros.SimpleActionState(
                                   '/move_base_flex/get_path', GetPathAction,
                                   goal_slots=['target_pose'],
                                   result_slots=['path'],
                                   result_cb=_get_path_result_cb),
                               transitions={
                                   'succeeded': 'EXE_PATH_TO_START',
                                   'aborted': 'RG_GET_PATH_TO_START',
                                   'preempted': 'preempted'
                               },
                               remapping={
                                   'target_pose': 'zone_start_pose',
                                   'path': 'path_plan'
                               })

        smach.StateMachine.add('RG_GET_PATH_TO_START',
                               RetryLimitedAction('retry_get_path_to_start', max_retries=3),
                               transitions={'retry': 'GET_PATH_TO_START',
                                            'give_up': 'aborted',
                                            'preempted': 'preempted'})

        # --- EXE_PATH_TO_START ---
        def _exe_start_result_cb(userdata, status, result):
            return classify_mbf_result('EXE_PATH_TO_START', status, result,
                                        log_pub=pubs.log_info)

        smach.StateMachine.add('EXE_PATH_TO_START',
                               smach_ros.SimpleActionState(
                                   '/move_base_flex/exe_path', ExePathAction,
                                   goal_slots=['path'],
                                   result_cb=_exe_start_result_cb),
                               transitions={
                                   'succeeded': 'VERIFY_AT_ZONE_START',
                                   'aborted': 'RECOVERY_ZONE',
                                   'preempted': 'preempted'
                               },
                               remapping={'path': 'path_plan'})

        smach.StateMachine.add('VERIFY_AT_ZONE_START',
                               VerifyAtPose('zone_start_pose', tolerance=0.5,
                                            label='VERIFY_AT_ZONE_START'),
                               transitions={
                                   'arrived': 'SET_CUT_HEIGHT',
                                   'not_arrived': 'RECOVERY_ZONE',
                                   'tf_unavailable': 'SET_CUT_HEIGHT',
                                   'preempted': 'preempted'
                               })

        # Recovery for zone start navigation
        @smach.cb_interface(outcomes=['done', 'preempted'])
        def recovery_zone_cb(ud):
            pubs.log_info.publish(String("Recovery (zone start)"))
            client = actionlib.SimpleActionClient('move_base_flex/recovery', RecoveryAction)
            client.wait_for_server(rospy.Duration(5.0))
            goal = RecoveryGoal()
            goal.behavior = 'clear_costmap'
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(10.0))
            return 'done'

        smach.StateMachine.add('RECOVERY_ZONE', smach.CBState(recovery_zone_cb),
                               transitions={'done': 'RG_GET_PATH_TO_START',
                                            'preempted': 'preempted'})

        # --- SET_CUT_HEIGHT ---
        @smach.cb_interface(input_keys=['zone_cut_height'], outcomes=['done', 'preempted'])
        def set_height_cb(ud):
            rospy.loginfo('[SET_CUT_HEIGHT] height=%d', ud.zone_cut_height)
            pubs.mower_set_height.publish(Int16(ud.zone_cut_height))
            pubs.log_info.publish(String("Setting mower height"))
            pubs.smach_status.publish(String("Setting height"))
            pubs.pm_play_melody.publish(Int16(1))
            rospy.sleep(1.5)
            return 'done'

        smach.StateMachine.add('SET_CUT_HEIGHT', smach.CBState(set_height_cb),
                               transitions={'done': 'WAIT_FOR_SET_CUT_HEIGHT',
                                            'preempted': 'preempted'})

        smach.StateMachine.add('WAIT_FOR_SET_CUT_HEIGHT',
                               WaitForMowerStatus('READY', timeout=120.0),
                               transitions={
                                   'reached': 'SET_RPM',
                                   'error': 'aborted',
                                   'timeout': 'aborted',
                                   'preempted': 'preempted'
                               })

        # --- SET_RPM + MOTOR ON ---
        @smach.cb_interface(input_keys=['zone_rpm'], outcomes=['done', 'preempted'])
        def set_rpm_cb(ud):
            rospy.loginfo('[SET_RPM] rpm=%d', ud.zone_rpm)
            pubs.mower_set_rpm.publish(Int16(ud.zone_rpm))
            pubs.smach_status.publish(String("Set rpm"))
            return 'done'

        smach.StateMachine.add('SET_RPM', smach.CBState(set_rpm_cb),
                               transitions={'done': 'SET_MOTOR_ON',
                                            'preempted': 'preempted'})

        @smach.cb_interface(outcomes=['done', 'preempted'])
        def set_motor_on_cb(ud):
            rospy.loginfo('[SET_MOTOR_ON] Turning motor on')
            pubs.mower_set_motor_on.publish(Bool(True))
            return 'done'

        smach.StateMachine.add('SET_MOTOR_ON', smach.CBState(set_motor_on_cb),
                               transitions={'done': 'WAIT_FOR_SET_RPM',
                                            'preempted': 'preempted'})

        smach.StateMachine.add('WAIT_FOR_SET_RPM', WaitForRpmReached(pubs, timeout=30.0),
                               transitions={
                                   'reached': 'PATH_IT',
                                   'error': 'aborted',
                                   'timeout': 'aborted',
                                   'preempted': 'preempted'
                               })

        # ==================== PATH ITERATOR ====================

        path_it = smach.Iterator(
            outcomes=['succeeded', 'preempted', 'aborted', 'critical_failure'],
            input_keys=['program', 'path_plan', 'zone_cut_height', 'zone_rpm',
                        'zone_name', 'zone_start_pose', 'paths',
                        'path_window_start_index', 'unfinished_active',
                        'unfinished_zone', 'unfinished_path', 'unfinished_window',
                        'retry_get_path_to_begin', 'retry_exe_to_begin',
                        'retry_planner', 'consecutive_nav_failures',
                        'restore_height_pending', 'error_reason'],
            output_keys=['path_plan', 'zone_cut_height', 'zone_rpm', 'zone_name',
                         'zone_start_pose', 'index_path', 'path_window_start_index',
                         'consecutive_nav_failures', 'error_reason'],
            it=lambda: range(0, len(process_zone_sm.userdata.paths)),
            it_label='index_path',
            exhausted_outcome='succeeded')
        path_it.userdata = process_zone_sm.userdata

        with path_it:
            process_path_sm = _build_process_path_sm(pubs, path_it)
            smach.Iterator.set_contained_state(
                'PROCESS_PATH', process_path_sm,
                loop_outcomes=['continue_path', 'aborted_path'])

        smach.StateMachine.add('PATH_IT', path_it,
                               transitions={
                                   'succeeded': 'SET_MOTOR_OFF_AND_HOME',
                                   'aborted': 'SET_MOTOR_OFF_AND_HOME',
                                   'critical_failure': 'EMERGENCY_BLADE_OFF',
                                   'preempted': 'preempted'
                               })

        # Emergency blade off on critical failure (before propagating up)
        @smach.cb_interface(outcomes=['done'])
        def emergency_blade_off_cb(ud):
            rospy.logwarn("[EMERGENCY_BLADE_OFF] Critical failure, safe shutdown")
            safe_blade_shutdown(source='EMERGENCY_CRITICAL', pubs=pubs)
            return 'done'

        smach.StateMachine.add('EMERGENCY_BLADE_OFF',
                               smach.CBState(emergency_blade_off_cb),
                               transitions={'done': 'critical_failure'})

        # --- SET_MOTOR_OFF_AND_HOME ---
        @smach.cb_interface(input_keys=['program'], output_keys=['program'],
                            outcomes=['done', 'preempted'])
        def motor_off_home_cb(ud):
            rospy.loginfo('[SET_MOTOR_OFF_AND_HOME] Raising blade')
            ud.program.last_result = 'succeeded'
            pubs.show_map_layer.publish(String("SMACH|MAP|FULL"))
            pubs.smach_status.publish(String("Raising blade to max"))
            safe_blade_shutdown(source='SET_MOTOR_OFF_AND_HOME', pubs=pubs)
            return 'done'

        smach.StateMachine.add('SET_MOTOR_OFF_AND_HOME', smach.CBState(motor_off_home_cb),
                               transitions={'done': 'WAIT_SET_MOTOR_OFF',
                                            'preempted': 'preempted'})

        smach.StateMachine.add('WAIT_SET_MOTOR_OFF',
                               WaitForMowerStatus('READY', timeout=60.0),
                               transitions={
                                   'reached': 'continue',
                                   'error': 'continue',
                                   'timeout': 'continue',
                                   'preempted': 'preempted'
                               })

    return process_zone_sm


# ===========================================================================
# Process Path sub-SM (nested inside PATH_IT)
# ===========================================================================

def _build_process_path_sm(pubs, path_it):
    """Build the sub-SM for processing a single path within a zone."""

    process_path_sm = smach.StateMachine(
        outcomes=['succeeded_path', 'preempted', 'aborted_path',
                  'continue_path', 'critical_failure'],
        input_keys=['program', 'zone_cut_height', 'zone_rpm', 'index',
                    'zone_name', 'zone_start_pose', 'path_plan', 'index_path',
                    'paths', 'path_window_start_index', 'path',
                    'unfinished_active', 'unfinished_zone', 'unfinished_path',
                    'unfinished_window',
                    'retry_get_path_to_begin', 'retry_exe_to_begin',
                    'retry_planner', 'consecutive_nav_failures',
                    'restore_height_pending', 'error_reason'],
        output_keys=['zone_cut_height', 'zone_rpm', 'zone_name', 'program',
                     'zone_start_pose', 'path_plan', 'path_window_start_index',
                     'consecutive_nav_failures', 'error_reason'])
    process_path_sm.userdata = path_it.userdata

    with process_path_sm:

        # --- GET_PATH_DATA ---
        smach.StateMachine.add('GET_PATH_DATA', GetPathData(pubs),
                               transitions={
                                   'available': 'GET_PATH_TO_BEGIN',
                                   'preempted': 'preempted',
                                   'skip_path': 'continue_path'
                               })

        # --- GET_PATH_TO_BEGIN ---
        def _get_path_begin_cb(userdata, status, result):
            pubs.log_info.publish(String("{}: go to path {}/{}".format(
                userdata.zone_name, userdata.index_path + 1, len(userdata.paths))))
            pubs.smach_status.publish(String("{}: go to path {}/{}".format(
                userdata.zone_name, userdata.index_path + 1, len(userdata.paths))))
            cls = classify_mbf_result('GET_PATH_TO_BEGIN', status, result,
                                       log_pub=pubs.log_info)
            if cls == 'succeeded':
                userdata.path_cost = result.cost
            return cls

        smach.StateMachine.add('GET_PATH_TO_BEGIN',
                               smach_ros.SimpleActionState(
                                   '/move_base_flex/get_path', GetPathAction,
                                   goal_slots=['target_pose'],
                                   result_slots=['path'],
                                   input_keys=['path_start_pose', 'path', 'path_planner',
                                               'path_cost', 'zone_name', 'index_path',
                                               'paths', 'program'],
                                   output_keys=['path_cost', 'program'],
                                   result_cb=_get_path_begin_cb),
                               transitions={
                                   'succeeded': 'CHECK_DISTANCE',
                                   'aborted': 'RG_GET_PATH_TO_BEGIN',
                                   'preempted': 'preempted'
                               },
                               remapping={
                                   'target_pose': 'path_start_pose',
                                   'path': 'path_plan'
                               })

        smach.StateMachine.add('RG_GET_PATH_TO_BEGIN',
                               RetryLimitedAction('retry_get_path_to_begin', max_retries=3),
                               transitions={'retry': 'GET_PATH_TO_BEGIN',
                                            'give_up': 'continue_path',
                                            'preempted': 'preempted'})

        # --- CHECK_DISTANCE ---
        smach.StateMachine.add('CHECK_DISTANCE', CheckDistance(),
                               transitions={
                                   'on_place': 'CHECK_PLANNER_PATH',
                                   'exe_path': 'EXE_PATH_TO_BEGIN',
                                   'preempted': 'preempted'
                               })

        # --- EXE_PATH_TO_BEGIN ---
        def _exe_begin_cb(userdata, status, result):
            pubs.log_info.publish(String("{}: go to path {}/{}".format(
                userdata.zone_name, userdata.index_path + 1, len(userdata.paths))))
            return classify_mbf_result('EXE_PATH_TO_BEGIN', status, result,
                                        log_pub=pubs.log_info)

        smach.StateMachine.add('EXE_PATH_TO_BEGIN',
                               smach_ros.SimpleActionState(
                                   '/move_base_flex/exe_path', ExePathAction,
                                   goal_slots=['path'],
                                   result_cb=_exe_begin_cb,
                                   input_keys=['paths', 'zone_name', 'index_path']),
                               transitions={
                                   'succeeded': 'VERIFY_AT_PATH_START',
                                   'aborted': 'RECOVERY_BEGIN',
                                   'preempted': 'preempted'
                               },
                               remapping={'path': 'path_plan'})

        smach.StateMachine.add('VERIFY_AT_PATH_START',
                               VerifyAtPose('path_start_pose', tolerance=0.5,
                                            label='VERIFY_AT_PATH_START'),
                               transitions={
                                   'arrived': 'CHECK_PLANNER_PATH',
                                   'not_arrived': 'RECOVERY_BEGIN',
                                   'tf_unavailable': 'CHECK_PLANNER_PATH',
                                   'preempted': 'preempted'
                               })

        @smach.cb_interface(outcomes=['done', 'preempted'])
        def recovery_begin_cb(ud):
            pubs.log_info.publish(String("Recovery (path begin)"))
            client = actionlib.SimpleActionClient('move_base_flex/recovery', RecoveryAction)
            client.wait_for_server(rospy.Duration(5.0))
            goal = RecoveryGoal()
            goal.behavior = 'clear_costmap'
            client.send_goal(goal)
            client.wait_for_result(rospy.Duration(10.0))
            return 'done'

        smach.StateMachine.add('RECOVERY_BEGIN', smach.CBState(recovery_begin_cb),
                               transitions={'done': 'RG_EXE_TO_BEGIN',
                                            'preempted': 'preempted'})

        smach.StateMachine.add('RG_EXE_TO_BEGIN',
                               RetryLimitedAction('retry_exe_to_begin', max_retries=3),
                               transitions={'retry': 'EXE_PATH_TO_BEGIN',
                                            'give_up': 'continue_path',
                                            'preempted': 'preempted'})

        # --- CHECK_PLANNER_PATH (costmap validation) ---
        @smach.cb_interface(input_keys=['path', 'zone_name', 'index_path', 'paths'])
        def check_path_request_cb(userdata, request):
            pubs.log_info.publish(String("{}: exe path {}/{}".format(
                userdata.zone_name, userdata.index_path + 1, len(userdata.paths))))
            pubs.smach_status.publish(String("{}: exe path {}/{}".format(
                userdata.zone_name, userdata.index_path + 1, len(userdata.paths))))
            request.path = userdata.path
            request.safety_dist = 0.1
            request.lethal_cost_mult = 0
            request.inscrib_cost_mult = 0
            request.unknown_cost_mult = 0
            request.costmap = CheckPathRequest.GLOBAL_COSTMAP
            request.skip_poses = 0
            request.use_padded_fp = False
            request.path_cells_only = False
            return request

        smach.StateMachine.add('CHECK_PLANNER_PATH',
                               smach_ros.ServiceState(
                                   '/move_base_flex/check_path_cost', CheckPath,
                                   request_cb=check_path_request_cb,
                                   response_cb=lambda ud, resp: 'succeeded',
                                   input_keys=['path', 'zone_name', 'index_path', 'paths']),
                               transitions={
                                   'succeeded': 'WINDOW_PLANNER_PATH',
                                   'aborted': 'TRIM_AND_RETRY',
                                   'preempted': 'preempted'
                               })

        # --- TRIM_AND_RETRY ---
        smach.StateMachine.add('TRIM_AND_RETRY', TrimAndRetry(),
                               transitions={
                                   'retry': 'CHECK_PLANNER_PATH',
                                   'bypass_check': 'EXE_PLANNER_PATH',
                                   'preempted': 'preempted'
                               })

        # --- WINDOW_PLANNER_PATH ---
        # 'detour_needed' fires when the costmap goal guard sees an obstacle on
        # the mowing line right ahead: route around it globally rather than
        # handing TEB a blocked chunk goal (avoids the dancing at chunk ends).
        smach.StateMachine.add('WINDOW_PLANNER_PATH', WindowPlannerPath(pubs),
                               transitions={
                                   'available': 'EXE_PLANNER_PATH',
                                   'finished': 'continue_path',
                                   'detour_needed': 'DETOUR',
                                   'preempted': 'preempted'
                               })

        # --- EXE_PLANNER_PATH (with BLOCKED detection) ---
        # 'detour' = a stall/abort where the live costmap confirms an obstacle on
        # the line ahead -> go straight around it (skip the slow NAV_RECOVERY
        # wait/retry that is meant for dynamic obstacles, not a standing person).
        smach.StateMachine.add('EXE_PLANNER_PATH', ExecutePathWithFeedback(pubs),
                               transitions={
                                   'succeeded': 'WINDOW_PLANNER_PATH',
                                   'replan_needed': 'WINDOW_PLANNER_PATH',
                                   'detour': 'DETOUR',
                                   'aborted': 'NAV_RECOVERY',
                                   'blocked': 'BLOCKED_RECOVERY',
                                   'preempted': 'preempted'
                               })

        # --- BLOCKED RECOVERY (4-phase cascade) ---
        blocked_sm = build_blocked_recovery_sm(pubs)
        smach.StateMachine.add('BLOCKED_RECOVERY', blocked_sm,
                               transitions={
                                   'recovered': 'WINDOW_PLANNER_PATH',
                                   'critical_error': 'SET_BLOCKED_ERROR',
                                   'preempted': 'preempted'
                               })

        @smach.cb_interface(input_keys=['error_reason'], output_keys=['error_reason'],
                            outcomes=['done'])
        def set_blocked_error_cb(ud):
            ud.error_reason = 'BLOCKED_FAILED'
            return 'done'

        smach.StateMachine.add('SET_BLOCKED_ERROR', smach.CBState(set_blocked_error_cb),
                               transitions={'done': 'critical_failure'})

        # --- NAV RECOVERY (wait + retry same chunk for dynamic obstacles) ---
        # On persistent obstacle it returns 'detour_needed' instead of skipping
        # the whole line.
        nav_sm = build_nav_recovery_sm(pubs)
        smach.StateMachine.add('NAV_RECOVERY', nav_sm,
                               transitions={
                                   'recovered': 'WINDOW_PLANNER_PATH',
                                   'detour_needed': 'DETOUR',
                                   'critical_error': 'SET_NAV_ERROR',
                                   'preempted': 'preempted'
                               })

        # --- DETOUR (route around the obstacle, rejoin the line downstream) ---
        smach.StateMachine.add('DETOUR', DetourAroundObstacle(pubs),
                               transitions={
                                   'resumed': 'WINDOW_PLANNER_PATH',
                                   'no_detour': 'DETOUR_ESCALATE',
                                   'preempted': 'preempted'
                               })

        # --- DETOUR_ESCALATE ---
        # Reached only when no detour around the obstacle is feasible. As a last
        # resort skip THIS line (not the whole zone) and move on. Only after many
        # consecutive un-mowable lines with no successful mowing in between (the
        # counter is reset on every genuine full-chunk completion) do we give up
        # and raise a navigation error — which now tries a return to dock first.
        MAX_CONSECUTIVE_UNMOWABLE = 5

        @smach.cb_interface(input_keys=['consecutive_nav_failures', 'zone_name',
                                        'index_path', 'paths'],
                            output_keys=['consecutive_nav_failures'],
                            outcomes=['skip', 'critical'])
        def detour_escalate_cb(ud):
            count = int(ud.consecutive_nav_failures or 0) + 1
            ud.consecutive_nav_failures = count
            if count >= MAX_CONSECUTIVE_UNMOWABLE:
                rospy.logerr("[DETOUR_ESCALATE] %d consecutive un-mowable lines - "
                             "navigation error", count)
                pubs.log_info.publish(String(
                    "Navigation: {} lines blocked, giving up".format(count)))
                return 'critical'
            rospy.logwarn("[DETOUR_ESCALATE] line %s blocked, no detour - skipping "
                          "(%d consecutive)", ud.index_path, count)
            pubs.log_info.publish(String(
                "Path skipped: obstacle, no detour possible"))
            # Blade back on for the next line.
            pubs.mower_set_motor_on.publish(Bool(True))
            rospy.sleep(1.0)
            return 'skip'

        smach.StateMachine.add('DETOUR_ESCALATE', smach.CBState(detour_escalate_cb),
                               transitions={
                                   'skip': 'continue_path',
                                   'critical': 'SET_NAV_ERROR'
                               })

        @smach.cb_interface(input_keys=['error_reason'], output_keys=['error_reason'],
                            outcomes=['done'])
        def set_nav_error_cb(ud):
            ud.error_reason = 'NAVIGATION_ABORTED'
            return 'done'

        smach.StateMachine.add('SET_NAV_ERROR', smach.CBState(set_nav_error_cb),
                               transitions={'done': 'critical_failure'})

    return process_path_sm


# ===========================================================================
# MISSION_CONCURRENCE (mission child + monitors)
# ===========================================================================

def build_mission_concurrence(pubs, parent_sm):
    """Wrap the mission child SM with concurrent monitors.

    Outcomes:
      - 'mission_complete': mission finished successfully
      - 'weather_preempt': rain detected in forecast
      - 'battery_preempt': battery <= 25%
      - 'temp_preempt': motor overtemperature
      - 'stop_preempt': external stop signal
      - 'critical_error': critical failure from within mission
      - 'preempted': generic preemption
    """

    mission_child = build_mission_child_sm(pubs)

    def child_term_cb(outcome_map):
        # If ANY child finishes/triggers, terminate all others
        return True

    def outcome_cb(outcome_map):
        # Priority: STOP > TEMP > BATTERY > WEATHER > mission
        if outcome_map.get('STOP_MONITOR') == 'invalid':
            pubs.stop_reason.publish(String("terminal:stop_signal"))
            return 'stop_preempt'
        if outcome_map.get('MOWER_TEMP_MONITOR') == 'invalid':
            pubs.stop_reason.publish(String("interrupted:temp"))
            return 'temp_preempt'
        if outcome_map.get('BATTERY_MONITOR') == 'invalid':
            pubs.stop_reason.publish(String("interrupted:battery"))
            return 'battery_preempt'
        if outcome_map.get('WEATHER_MONITOR') == 'invalid':
            pubs.stop_reason.publish(String("interrupted:weather"))
            return 'weather_preempt'
        mission_out = outcome_map.get('MISSION_CHILD')
        if mission_out == 'succeeded':
            return 'mission_complete'
        if mission_out == 'aborted':
            pubs.stop_reason.publish(String("critical:mission_aborted"))
            return 'critical_error'
        return 'preempted'

    all_keys = ['program', 'prg_start_time', 'unfinished_active',
                'unfinished_zone', 'unfinished_path', 'unfinished_window',
                'path_window_start_index', 'path_chunk', 'error_reason',
                'restore_height_pending', 'consecutive_nav_failures']

    cc = smach.Concurrence(
        outcomes=['mission_complete', 'weather_preempt', 'battery_preempt',
                  'temp_preempt', 'stop_preempt', 'critical_error', 'preempted'],
        default_outcome='preempted',
        child_termination_cb=child_term_cb,
        outcome_cb=outcome_cb,
        input_keys=all_keys,
        output_keys=['program', 'prg_start_time', 'error_reason']
    )
    cc.userdata = parent_sm.userdata

    with cc:
        smach.Concurrence.add('MISSION_CHILD', mission_child)

        smach.Concurrence.add('WEATHER_MONITOR',
                              smach_ros.MonitorState(
                                  '/weather_alert/rain_alert', RainAlert,
                                  weather_monitor_cb))

        smach.Concurrence.add('BATTERY_MONITOR',
                              smach_ros.MonitorState(
                                  '/pm/power_status', Power_status,
                                  battery_monitor_cb))

        smach.Concurrence.add('MOWER_TEMP_MONITOR',
                              smach_ros.MonitorState(
                                  '/mower/status', Mower,
                                  mower_temp_monitor_cb))

        smach.Concurrence.add('STOP_MONITOR',
                              smach_ros.MonitorState(
                                  '/mower_smach/stop', Bool,
                                  stop_monitor_cb))

    return cc
