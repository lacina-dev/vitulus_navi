"""
All SMACH state classes for the mower state machine.

Moved from the monolithic mower_unit_smach script.
Bug fixes applied:
  - Publisher leak: states receive MowerPublishers, no per-execute publisher creation
  - Removed dead 'pub_status is None' checks
  - Unified direction_angle (removed GetPathData.direction duplicate)
  - Fixed duplicate input_keys
"""
import math
import rospy
import smach
import shapely
from shapely import geometry
import PyKDL as kdl
import actionlib

from nav_msgs.msg import Path
from std_msgs.msg import Bool, String, Int16, Int8
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from vitulus_msgs.msg import (
    DockProgram, StringList, PlannerProgram, Mower,
    Device_icon_status, Power_status)
from weather_alert.msg import RainAlert
from mbf_msgs.msg import (
    ExePathAction, ExePathGoal, ExePathActionFeedback,
    GetPathAction)
from mbf_msgs.msg import RecoveryAction, RecoveryGoal
from mbf_msgs.srv import CheckPath, CheckPathRequest
from rtabmap_msgs.msg import Info

from .helpers import (
    get_tf_listener, get_robot_xy_in_map, nearest_pose_index,
    classify_mbf_result, distance_xy, direction_angle,
    safe_blade_shutdown, segmentize_raw_path, MOWER_ERROR_STATES)


# ===========================================================================
# Generic utility states
# ===========================================================================

class VerifyAtPose(smach.State):
    """Confirm via TF that the robot is within *tolerance* metres of a target.

    Prevents premature mower spinup when MBF reports SUCCESS but the robot
    hasn't actually arrived (TEB/MBF edge case).
    """

    def __init__(self, target_key, tolerance=0.5, label='VerifyAtPose'):
        smach.State.__init__(self,
                             outcomes=['arrived', 'not_arrived', 'tf_unavailable', 'preempted'],
                             input_keys=[target_key])
        self.target_key = target_key
        self.tolerance = float(tolerance)
        self.label = str(label)

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        target = getattr(userdata, self.target_key, None)
        if target is None:
            rospy.logwarn('[%s] target pose userdata.%s is None',
                          self.label, self.target_key)
            return 'not_arrived'
        for _ in range(5):
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            xy = get_robot_xy_in_map(timeout=0.5)
            if xy is not None:
                tx = target.pose.position.x
                ty = target.pose.position.y
                d = distance_xy(xy[0], xy[1], tx, ty)
                rospy.loginfo('[%s] distance to %s = %.2fm (tol=%.2fm)',
                              self.label, self.target_key, d, self.tolerance)
                if d <= self.tolerance:
                    return 'arrived'
                return 'not_arrived'
            rospy.sleep(0.2)
        rospy.logwarn('[%s] TF unavailable', self.label)
        return 'tf_unavailable'


class WaitForMowerStatus(smach.State):
    """Wait until /mower/status reaches a target string with timeout + error detection."""

    def __init__(self, targets, timeout=30.0, extra_check=None):
        smach.State.__init__(self,
                             outcomes=['reached', 'error', 'timeout', 'preempted'])
        if isinstance(targets, str):
            targets = (targets,)
        self.targets = tuple(targets)
        self.timeout = float(timeout)
        self.extra_check = extra_check

    def execute(self, userdata):
        deadline = rospy.Time.now() + rospy.Duration(self.timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            try:
                msg = rospy.wait_for_message('/mower/status', Mower, timeout=1.0)
            except rospy.ROSException:
                continue
            if msg.status in MOWER_ERROR_STATES:
                rospy.logwarn('[WaitForMowerStatus] FW error state: %s', msg.status)
                return 'error'
            if msg.status in self.targets:
                if self.extra_check is None or self.extra_check(msg):
                    return 'reached'
        rospy.logwarn('[WaitForMowerStatus] timeout (%.1fs) waiting for %s',
                      self.timeout, self.targets)
        return 'timeout'


class WaitForTopic(smach.State):
    """Generic timeout-bounded wait for a single message that satisfies *predicate*."""

    def __init__(self, topic, msg_type, predicate, timeout=30.0,
                 output_keys=None, on_match=None):
        smach.State.__init__(self,
                             outcomes=['received', 'timeout', 'preempted'],
                             input_keys=[],
                             output_keys=output_keys or [])
        self.topic = topic
        self.msg_type = msg_type
        self.predicate = predicate
        self.timeout = float(timeout)
        self.on_match = on_match

    def execute(self, userdata):
        deadline = rospy.Time.now() + rospy.Duration(self.timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            remaining = (deadline - rospy.Time.now()).to_sec()
            if remaining <= 0:
                break
            try:
                msg = rospy.wait_for_message(self.topic, self.msg_type,
                                             timeout=min(1.0, remaining))
            except rospy.ROSException:
                continue
            if self.predicate(msg):
                if self.on_match is not None:
                    try:
                        self.on_match(userdata, msg)
                    except Exception as e:
                        rospy.logwarn('[WaitForTopic %s] on_match raised: %s', self.topic, e)
                return 'received'
        rospy.logwarn('[WaitForTopic] timeout (%.1fs) on %s', self.timeout, self.topic)
        return 'timeout'


class RetryLimitedAction(smach.State):
    """Reusable retry counter; increments a userdata key, decides retry vs give-up."""

    def __init__(self, counter_key, max_retries=3):
        smach.State.__init__(self,
                             outcomes=['retry', 'give_up', 'preempted'],
                             input_keys=[counter_key],
                             output_keys=[counter_key])
        self.counter_key = counter_key
        self.max_retries = int(max_retries)

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        current = int(getattr(userdata, self.counter_key, 0) or 0) + 1
        setattr(userdata, self.counter_key, current)
        if current >= self.max_retries:
            rospy.logwarn('[RetryLimitedAction] %s reached %d retries; giving up',
                          self.counter_key, current)
            setattr(userdata, self.counter_key, 0)
            return 'give_up'
        rospy.loginfo('[RetryLimitedAction] %s retry %d/%d',
                      self.counter_key, current, self.max_retries)
        return 'retry'


# ===========================================================================
# Pre-start validation
# ===========================================================================

class PreStartCheck(smach.State):
    """Validate weather and battery conditions before allowing a mission.

    Rule 1 (weather): reject if rain now, rain in past 60 min history,
    or rain predicted in any nowcast step.
    Rule 1 (battery): reject if battery_capacity < 40% (hysteresis).
    """
    RAIN_STATUSES = ('RAIN', 'ALERT', 'WARN')

    def __init__(self, pubs):
        smach.State.__init__(self,
                             outcomes=['passed', 'rejected', 'preempted'],
                             input_keys=['program'], output_keys=[])
        self.pubs = pubs

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        # --- Weather check ---
        try:
            rain = rospy.wait_for_message(
                '/weather_alert/rain_alert', RainAlert, timeout=5.0)

            if rain.rain_now == 1:
                self.pubs.log_info.publish(String("Start rejected: raining now"))
                self.pubs.smach_status.publish(String("Rejected: rain"))
                return 'rejected'

            past = [rain.status_past60m, rain.status_past50m, rain.status_past40m,
                    rain.status_past30m, rain.status_past20m, rain.status_past10m,
                    rain.status_now]
            for s in past:
                if s == 'RAIN':
                    self.pubs.log_info.publish(String("Start rejected: recent rain"))
                    self.pubs.smach_status.publish(String("Rejected: recent rain"))
                    return 'rejected'

            nowcast = [rain.status_nowcast10m, rain.status_nowcast20m,
                       rain.status_nowcast30m]
            if any(s in self.RAIN_STATUSES for s in nowcast):
                self.pubs.log_info.publish(String("Start rejected: rain forecast"))
                self.pubs.smach_status.publish(String("Rejected: forecast"))
                return 'rejected'

        except rospy.ROSException:
            rospy.logwarn("[PRE_START_CHECK] Weather data unavailable, proceeding")

        # --- Battery check ---
        try:
            pm = rospy.wait_for_message('/pm/power_status', Power_status, timeout=5.0)
            if pm.battery_capacity < 40:
                self.pubs.log_info.publish(String(
                    "Start rejected: battery {}% < 40%".format(pm.battery_capacity)))
                self.pubs.smach_status.publish(String("Rejected: low battery"))
                return 'rejected'
        except rospy.ROSException:
            rospy.logwarn("[PRE_START_CHECK] Power status unavailable, proceeding")

        self.pubs.log_info.publish(String("Pre-start checks passed"))
        return 'passed'


# ===========================================================================
# Zone / path processing states
# ===========================================================================

class GetZoneData(smach.State):
    def __init__(self, pubs):
        smach.State.__init__(self,
                             outcomes=['available', 'failed', 'preempted', 'skip_zone'],
                             input_keys=['program', 'zone_cut_height', 'zone_rpm', 'index',
                                         'zone_name', 'zone_start_pose', 'paths',
                                         'unfinished_active', 'unfinished_zone',
                                         'unfinished_path', 'path_window_start_index',
                                         'unfinished_window'],
                             output_keys=['zone_cut_height', 'zone_rpm', 'zone_name',
                                          'zone_start_pose', 'paths', 'controller',
                                          'unfinished_active', 'unfinished_zone',
                                          'unfinished_path', 'path_window_start_index',
                                          'retry_get_path_to_start',
                                          'retry_get_path_to_begin',
                                          'retry_exe_to_begin',
                                          'retry_planner',
                                          'consecutive_nav_failures'])
        self.pubs = pubs

    def execute(self, userdata):
        rospy.loginfo('[GetZoneData] Getting zone data')
        # Reset retry counters for new zone
        userdata.retry_get_path_to_start = 0
        userdata.retry_get_path_to_begin = 0
        userdata.retry_exe_to_begin = 0
        userdata.retry_planner = 0
        userdata.consecutive_nav_failures = 0

        current_zone_name = userdata.program.zone_list[userdata.index].name

        if userdata.unfinished_active and current_zone_name != userdata.unfinished_zone:
            rospy.loginfo('[GetZoneData] Skipping zone %s to resume at %s',
                          current_zone_name, userdata.unfinished_zone)
            return 'skip_zone'

        if userdata.unfinished_active and current_zone_name == userdata.unfinished_zone:
            rospy.loginfo('[GetZoneData] Resuming zone: %s path: %s',
                          userdata.unfinished_zone, userdata.unfinished_path)

        zone = userdata.program.zone_list[userdata.index]
        userdata.zone_cut_height = zone.cut_height
        userdata.zone_rpm = zone.rpm
        userdata.zone_name = zone.name
        userdata.paths = zone.paths

        if userdata.unfinished_active and current_zone_name == userdata.unfinished_zone:
            resume_path_idx = int(userdata.unfinished_path)
            resume_window = int(userdata.unfinished_window) if userdata.unfinished_window else 0
            raw_resume_path = userdata.paths[resume_path_idx]
            try:
                segmented = segmentize_raw_path(raw_resume_path)
                if not segmented.poses:
                    raise ValueError("segmented path is empty")
                resume_window = max(0, min(resume_window, len(segmented.poses) - 1))
                userdata.zone_start_pose = segmented.poses[resume_window]
                rospy.loginfo('[GetZoneData] Resume: path %d window %d (poses=%d)',
                              resume_path_idx, resume_window, len(segmented.poses))
            except Exception as e:
                rospy.logwarn('[GetZoneData] Resume pose computation failed (%s); fallback', e)
                userdata.zone_start_pose = raw_resume_path.poses[0]
        else:
            userdata.zone_start_pose = zone.paths[0].poses[0]

        userdata.controller = 'base_local_planner/TrajectoryPlannerROS'
        self.pubs.show_map_layer.publish(String("SMACH|MAP|FULL"))
        rospy.sleep(3)

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return 'available'


class GetPathData(smach.State):
    def __init__(self, pubs):
        smach.State.__init__(self,
                             outcomes=['available', 'preempted', 'skip_path'],
                             input_keys=['program', 'zone_cut_height', 'zone_rpm', 'index',
                                         'zone_name', 'path_planner', 'zone_start_pose',
                                         'path_plan', 'index_path', 'paths',
                                         'path_start_pose', 'path', 'final_pose',
                                         'unfinished_active', 'unfinished_zone',
                                         'unfinished_path', 'unfinished_window',
                                         'path_window_start_index'],
                             output_keys=['program', 'zone_cut_height', 'zone_rpm',
                                          'zone_name', 'zone_start_pose', 'path_planner',
                                          'path_plan', 'paths', 'path_start_pose', 'path',
                                          'final_pose', 'unfinished_active', 'unfinished_zone',
                                          'unfinished_path', 'unfinished_window',
                                          'path_window_start_index',
                                          'retry_get_path_to_begin', 'retry_exe_to_begin',
                                          'retry_planner', 'consecutive_nav_failures'])
        self.pubs = pubs

    def execute(self, userdata):
        rospy.loginfo('[GetPathData] Getting path data')
        # Reset path-level retry counters
        userdata.retry_get_path_to_begin = 0
        userdata.retry_exe_to_begin = 0
        userdata.retry_planner = 0

        if userdata.unfinished_active and str(userdata.index_path) != userdata.unfinished_path:
            rospy.loginfo('[GetPathData] Skipping path %s to resume at %s',
                          userdata.index_path, userdata.unfinished_path)
            return 'skip_path'

        path = userdata.paths[userdata.index_path]
        userdata.path_start_pose = path.poses[0]
        userdata.path_planner = path

        # Save current position for crash recovery
        userdata.program.last_result = (
            'failed: on_path-{}-{}-{}'.format(
                userdata.zone_name, userdata.index_path,
                userdata.path_window_start_index))
        self.pubs.save_program.publish(userdata.program)
        self.pubs.show_map_layer.publish(
            String("SMACH|ZONE|{}".format(userdata.zone_name)))

        line = geometry.LineString(
            [[path.poses[0].pose.position.x, path.poses[0].pose.position.y],
             [path.poses[-1].pose.position.x, path.poses[-1].pose.position.y]])

        segmentize_distance = 0.03

        # Coverage outline path (closed polygon)
        if line.length == 0:
            start_offset = 0.18
            start_poses_offset = int(round(start_offset / segmentize_distance))
            path_new = Path()
            path_new.header = path.header
            polygon = geometry.LinearRing(
                [[pose.pose.position.x, pose.pose.position.y] for pose in path.poses])
            polygon = shapely.remove_repeated_points(polygon)
            for n in range(0, len(polygon.coords) - 1):
                seg_line = geometry.LineString(
                    [[polygon.coords[n][0], polygon.coords[n][1]],
                     [polygon.coords[n + 1][0], polygon.coords[n + 1][1]]])
                angle = direction_angle(list(seg_line.coords))
                seg_line = seg_line.segmentize(segmentize_distance)
                for point in seg_line.coords:
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = point[0]
                    pose.pose.position.y = point[1]
                    pose.pose.position.z = 0.0
                    pose.pose.orientation = Quaternion(
                        *(kdl.Rotation.RPY(0, 0, angle).GetQuaternion()))
                    path_new.poses.append(pose)
            path_new.poses = path_new.poses[0:-start_poses_offset]
            path_new.poses = path_new.poses[start_poses_offset:]
            path = path_new
            userdata.path_start_pose = path.poses[0]
            userdata.final_pose = path.poses[-1]

        # Coverage infill path
        else:
            line_geom = geometry.LineString(
                [[pose.pose.position.x, pose.pose.position.y] for pose in path.poses])
            path_new = Path()
            path_new.header.frame_id = "map"
            path_new.header.stamp = rospy.Time.now()
            line_geom = line_geom.segmentize(segmentize_distance)
            for pose_id in range(0, len(line_geom.coords) - 1):
                angle = direction_angle(
                    [[line_geom.coords[pose_id][0], line_geom.coords[pose_id][1]],
                     [line_geom.coords[pose_id + 1][0], line_geom.coords[pose_id + 1][1]]])
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = line_geom.coords[pose_id][0]
                pose.pose.position.y = line_geom.coords[pose_id][1]
                pose.pose.position.z = 0.0
                pose.pose.orientation = Quaternion(
                    *(kdl.Rotation.RPY(0, 0, angle).GetQuaternion()))
                path_new.poses.append(pose)
            path = path_new

        # If resuming, set start pose to unfinished window
        if userdata.unfinished_active and str(userdata.index_path) == userdata.unfinished_path:
            rospy.loginfo('[GetPathData] Resuming path %s window %s',
                          userdata.unfinished_path, userdata.unfinished_window)
            userdata.path_window_start_index = userdata.unfinished_window
            userdata.path_start_pose = path.poses[userdata.path_window_start_index]
            userdata.unfinished_active = False

        self.pubs.current_path.publish(path)
        userdata.path = path

        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        return 'available'


class CheckDistance(smach.State):
    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['on_place', 'exe_path', 'preempted'],
                             input_keys=['path_cost'],
                             output_keys=['path_cost'])

    def execute(self, userdata):
        rospy.loginfo('[CheckDistance] path_cost=%.2f', userdata.path_cost or 0)
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        if userdata.path_cost <= 0.2:
            return 'on_place'
        return 'exe_path'


# ===========================================================================
# Path windowing and trimming
# ===========================================================================

class WindowPlannerPath(smach.State):
    """Produce the next chunk of a long mowing path, anchored to the robot's
    current TF position."""

    def __init__(self, pubs):
        smach.State.__init__(self,
                             outcomes=['available', 'finished', 'preempted'],
                             input_keys=['path', 'path_window_start_index', 'zone_name',
                                         'index_path', 'program'],
                             output_keys=['path_chunk', 'path_window_start_index', 'program',
                                          'path_chunk_trim_count'])
        self.window_size = 200
        self.fallback_step = 100
        self.end_tolerance = 0.4
        self.tail_indices = 5
        self.pubs = pubs

    def execute(self, userdata):
        rospy.loginfo('[WindowPlannerPath] Windowing path')
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        userdata.path_chunk_trim_count = 0
        full_path = userdata.path
        if not full_path or not full_path.poses:
            userdata.path_window_start_index = 0
            return 'finished'

        prev_index = int(userdata.path_window_start_index or 0)
        n = len(full_path.poses)

        robot_xy = get_robot_xy_in_map()
        if robot_xy is not None:
            current_index = nearest_pose_index(
                full_path, robot_xy[0], robot_xy[1], hint_index=prev_index)
            current_index = max(current_index, prev_index)

            last_p = full_path.poses[-1].pose.position
            dist_to_end = math.hypot(robot_xy[0] - last_p.x, robot_xy[1] - last_p.y)
            if current_index >= n - self.tail_indices and dist_to_end < self.end_tolerance:
                rospy.loginfo('[WindowPlannerPath] reached end (idx=%d/%d dist=%.2fm)',
                              current_index, n, dist_to_end)
                userdata.path_window_start_index = 0
                return 'finished'
        else:
            rospy.logwarn_throttle(5.0, '[WindowPlannerPath] TF unavailable; fixed-step')
            current_index = prev_index
            if current_index >= n:
                userdata.path_window_start_index = 0
                return 'finished'

        end_index = min(current_index + self.window_size, n)
        path_chunk = Path()
        path_chunk.header = full_path.header
        path_chunk.poses = full_path.poses[current_index:end_index]

        if not path_chunk.poses:
            userdata.path_window_start_index = 0
            return 'finished'

        userdata.path_chunk = path_chunk

        if robot_xy is not None:
            userdata.path_window_start_index = current_index
        else:
            userdata.path_window_start_index = min(current_index + self.fallback_step, n)

        userdata.program.last_result = (
            'failed: on_path-{}-{}-{}'.format(
                userdata.zone_name, userdata.index_path, current_index))

        return 'available'


class TrimAndRetry(smach.State):
    """Progressively trims poses from a path chunk when CHECK_PLANNER_PATH fails."""

    def __init__(self):
        smach.State.__init__(self,
                             outcomes=['retry', 'bypass_check', 'preempted'],
                             input_keys=['path_chunk', 'path_chunk_trim_count'],
                             output_keys=['path_chunk', 'path_chunk_trim_count'])
        self.trim_step = 15
        self.max_trims = 5
        self.min_chunk_size = 20

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        trim_count = userdata.path_chunk_trim_count
        chunk = userdata.path_chunk
        if trim_count >= self.max_trims or len(chunk.poses) <= self.min_chunk_size:
            rospy.loginfo('[TrimAndRetry] max trims reached (%d), bypassing', trim_count)
            return 'bypass_check'
        trimmed = Path()
        trimmed.header = chunk.header
        trimmed.poses = chunk.poses[:-self.trim_step]
        if not trimmed.poses:
            return 'bypass_check'
        userdata.path_chunk = trimmed
        userdata.path_chunk_trim_count = trim_count + 1
        rospy.loginfo('[TrimAndRetry] trimmed %d poses, now %d (attempt %d/%d)',
                      self.trim_step, len(trimmed.poses), trim_count + 1, self.max_trims)
        return 'retry'


# ===========================================================================
# Path execution (extended with BLOCKED detection)
# ===========================================================================

class ExecutePathWithFeedback(smach.State):
    """Execute a path chunk via MBF with feedback monitoring.

    Extended outcomes:
      - 'blocked': /mower/status reported BLOCKED during execution.
        The MBF goal is cancelled and the caller should enter BLOCKED recovery.
    """

    def __init__(self, pubs):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'aborted', 'preempted',
                                       'replan_needed', 'blocked'],
                             input_keys=['path_chunk', 'paths', 'zone_name', 'index_path',
                                         'final_pose', 'path', 'path_window_start_index',
                                         'restore_height_pending', 'zone_cut_height'],
                             output_keys=['restore_height_pending'])
        self._client = actionlib.SimpleActionClient(
            '/move_base_flex/exe_path', ExePathAction)
        self.pubs = pubs
        self._feedback_sub = None
        self.last_feedback = None
        self.step_size = 100
        # Persistent mower status subscriber
        self._mower_status = None
        self._mower_sub = rospy.Subscriber(
            '/mower/status', Mower, self._mower_status_cb)
        if not self._client.wait_for_server(rospy.Duration(5.0)):
            rospy.logerr("Action server '/move_base_flex/exe_path' not available")
            raise RuntimeError("exe_path action server not available")

    def _mower_status_cb(self, msg):
        self._mower_status = msg

    def feedback_cb(self, msg):
        self.last_feedback = msg.feedback

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'

        is_last_chunk = (
            userdata.path_window_start_index + self.step_size) >= len(userdata.path.poses)

        self.last_feedback = None
        self._feedback_sub = rospy.Subscriber(
            '/move_base_flex/exe_path/feedback', ExePathActionFeedback,
            self.feedback_cb)

        goal = ExePathGoal()
        goal.path = userdata.path_chunk
        self._client.send_goal(goal)

        try:
            while self._client.get_state() in [actionlib.GoalStatus.PENDING,
                                                actionlib.GoalStatus.ACTIVE]:
                if self.preempt_requested():
                    self._client.cancel_goal()
                    self.service_preempt()
                    return 'preempted'

                # --- BLOCKED detection ---
                if self._mower_status and self._mower_status.status == 'BLOCKED':
                    rospy.logwarn("[EXE_PLANNER_PATH] Motor BLOCKED detected")
                    self._client.cancel_goal()
                    self._client.wait_for_result(rospy.Duration(1.0))
                    return 'blocked'

                # --- ERR detection ---
                if self._mower_status and self._mower_status.status == 'ERR':
                    rospy.logwarn("[EXE_PLANNER_PATH] Motor ERR detected")
                    self._client.cancel_goal()
                    self._client.wait_for_result(rospy.Duration(1.0))
                    return 'aborted'

                # --- Smooth window transition ---
                if (not is_last_chunk and self.last_feedback
                        and self.last_feedback.dist_to_goal < 0.4):
                    rospy.loginfo("[EXE_PLANNER_PATH] Near chunk end, planning next window")
                    self._client.cancel_goal()
                    self._client.wait_for_result(rospy.Duration(0.5))
                    return 'replan_needed'

                rospy.sleep(0.1)
        finally:
            if self._feedback_sub:
                self._feedback_sub.unregister()
                self._feedback_sub = None

        result = self._client.get_result()
        status = self._client.get_state()

        self.pubs.log_info.publish(String(
            "{}: path {}/{} chunk done".format(
                userdata.zone_name, userdata.index_path + 1, len(userdata.paths))))
        self.pubs.smach_status.publish(String(
            "{}: path {}/{} done".format(
                userdata.zone_name, userdata.index_path + 1, len(userdata.paths))))

        if result is None:
            rospy.logerr("[EXE_PLANNER_PATH] None result from action server")
            return 'aborted'

        # After successful chunk: restore height if pending from BLOCKED Phase 2
        if result.outcome == 0:
            if getattr(userdata, 'restore_height_pending', False):
                rospy.loginfo("[EXE_PLANNER_PATH] Restoring cut height to %d",
                              userdata.zone_cut_height)
                self.pubs.mower_set_height.publish(Int16(userdata.zone_cut_height))
                userdata.restore_height_pending = False
            return 'succeeded'
        return 'aborted'


# ===========================================================================
# Mower power states
# ===========================================================================

class WaitForRpmReached(smach.State):
    """Wait for cutting motor to reach setpoint RPM (±100)."""

    def __init__(self, pubs, timeout=30.0):
        smach.State.__init__(self,
                             outcomes=['reached', 'error', 'timeout', 'preempted'],
                             input_keys=['zone_rpm'])
        self.timeout = float(timeout)
        self.pubs = pubs

    def execute(self, userdata):
        deadline = rospy.Time.now() + rospy.Duration(self.timeout)
        target = int(userdata.zone_rpm)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            try:
                msg = rospy.wait_for_message('/mower/status', Mower, timeout=1.0)
            except rospy.ROSException:
                continue
            if msg.status in MOWER_ERROR_STATES:
                rospy.logwarn('[WaitForRpmReached] FW error: %s', msg.status)
                return 'error'
            if target - 100 < msg.moto_rpm < target + 100:
                self.pubs.log_info.publish(String("Mowing started..."))
                self.pubs.smach_status.publish(String("Mowing started"))
                return 'reached'
        rospy.logwarn('[WaitForRpmReached] timeout (%.1fs) target=%d', self.timeout, target)
        return 'timeout'


class WaitForMowerOff(smach.State):
    """Wait for /mower/status == OFF after power-off."""

    def __init__(self, pubs, timeout=30.0):
        smach.State.__init__(self,
                             outcomes=['off', 'timeout', 'preempted'],
                             input_keys=['program', 'prg_start_time'],
                             output_keys=['prg_start_time'])
        self.timeout = float(timeout)
        self.pubs = pubs

    def execute(self, userdata):
        deadline = rospy.Time.now() + rospy.Duration(self.timeout)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            try:
                msg = rospy.wait_for_message('/mower/status', Mower, timeout=1.0)
            except rospy.ROSException:
                continue
            if msg.status == 'OFF':
                if userdata.prg_start_time is not None:
                    duration = (rospy.Time.now() - userdata.prg_start_time).to_sec() / 60
                    userdata.program.last_duration_minutes = int(round(duration))
                    self.pubs.save_program.publish(userdata.program)
                    userdata.prg_start_time = None
                self.pubs.active_program.publish(String(" "))
                self.pubs.smach_status.publish(String("Ready"))
                self.pubs.log_info.publish(String("Done"))
                return 'off'
        rospy.logwarn('[WaitForMowerOff] timeout (%.1fs)', self.timeout)
        self.pubs.active_program.publish(String(" "))
        return 'timeout'


# ===========================================================================
# Dock-related states
# ===========================================================================

class CheckIfDockedState(smach.State):
    MAX_WAITS = 5

    def __init__(self, pubs):
        smach.State.__init__(self,
                             outcomes=['proceed_to_undock', 'skip_undocking', 'wait', 'preempted'])
        self.pubs = pubs
        self._wait_count = 0

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        try:
            msg = rospy.wait_for_message('/dock_smach/dock_status', Int8, timeout=5.0)
            self._wait_count = 0
            if msg.data == 3:  # Undocked
                self.pubs.log_info.publish(String("Mower is already undocked."))
                return 'skip_undocking'
            elif msg.data == 0:  # Docked
                self.pubs.log_info.publish(String("Mower is docked, starting undock."))
                return 'proceed_to_undock'
            else:
                self.pubs.log_info.publish(String(
                    "Waiting for dock status, current: {}".format(msg.data)))
                return 'wait'
        except rospy.ROSException:
            self._wait_count += 1
            rospy.logwarn("Timeout /dock_smach/dock_status (%d/%d)",
                          self._wait_count, self.MAX_WAITS)
            if self._wait_count >= self.MAX_WAITS:
                self.pubs.log_info.publish(String(
                    "No dock_status, assuming undocked."))
                self._wait_count = 0
                return 'skip_undocking'
            return 'wait'


class WaitForPlannerLoaded(smach.State):
    def __init__(self, pubs, timeout=120.0):
        smach.State.__init__(self,
                             outcomes=['received', 'timeout', 'preempted'],
                             input_keys=['program'], output_keys=['program'])
        self.timeout = float(timeout)
        self.pubs = pubs

    def execute(self, userdata):
        userdata.program.last_result = 'failed: on_planner'
        deadline = rospy.Time.now() + rospy.Duration(self.timeout)
        target = userdata.program.map_name
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.preempt_requested():
                self.service_preempt()
                return 'preempted'
            try:
                msg = rospy.wait_for_message(
                    '/web_plan/planner_loaded', String, timeout=1.0)
            except rospy.ROSException:
                continue
            if msg.data == target:
                self.pubs.show_map_layer.publish(String("SMACH|MAP|FULL"))
                self.pubs.log_info.publish(String("Map is ready"))
                self.pubs.smach_status.publish(String("Map ready"))
                self.pubs.pm_play_melody.publish(Int16(1))
                rospy.sleep(2.0)
                return 'received'
        rospy.logwarn('WAIT_FOR_PLANNER timeout (%.1fs) for map=%s', self.timeout, target)
        return 'timeout'


class WaitForDockedState(smach.State):
    DOCK_TIMEOUT = 600.0

    def __init__(self, pubs):
        smach.State.__init__(self,
                             outcomes=['succeeded', 'failed', 'timeout', 'preempted'],
                             input_keys=['error_reason'],
                             output_keys=['error_reason'])
        self.pubs = pubs

    def execute(self, userdata):
        rospy.loginfo("Waiting for docked (timeout %.0fs)...", self.DOCK_TIMEOUT)
        deadline = rospy.Time.now() + rospy.Duration(self.DOCK_TIMEOUT)
        while not rospy.is_shutdown() and rospy.Time.now() < deadline:
            if self.preempt_requested():
                self.pubs.log_info.publish(String("Docking preempted."))
                self.service_preempt()
                return 'preempted'
            try:
                msg = rospy.wait_for_message('/dock_smach/dock_status', Int8, timeout=1.0)
                if msg.data == 0:  # Docked
                    self.pubs.log_info.publish(String("Mower successfully docked."))
                    self.pubs.smach_status.publish(String("Ready"))
                    return 'succeeded'
                elif msg.data == 4:  # Failed
                    self.pubs.log_info.publish(String("Docking failed."))
                    userdata.error_reason = 'DOCKING_FAILED'
                    return 'failed'
            except rospy.ROSException:
                pass
        rospy.logwarn("WAIT_FOR_DOCKED timeout after %.0fs.", self.DOCK_TIMEOUT)
        self.pubs.log_info.publish(String("Docking timed out."))
        userdata.error_reason = 'DOCKING_TIMEOUT'
        return 'timeout'


class CheckForDockPoint(smach.State):
    """Check if a DOCK waypoint exists in the map point list."""

    def __init__(self, pubs):
        smach.State.__init__(self,
                             outcomes=['dock_point_found', 'no_dock_point', 'preempted'])
        self.pubs = pubs

    def execute(self, userdata):
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        rospy.loginfo('[CheckForDockPoint] Checking for DOCK point')
        try:
            msg = rospy.wait_for_message(
                '/navi_manager/map_point_str_list', StringList, timeout=5.0)
            if 'DOCK' in msg.string_list:
                self.pubs.log_info.publish(String("DOCK point found."))
                return 'dock_point_found'
            else:
                self.pubs.log_info.publish(String("DOCK point not found."))
                return 'no_dock_point'
        except rospy.ROSException:
            self.pubs.log_info.publish(String("Could not get map point list."))
            return 'no_dock_point'
