"""
Shared helper functions for the mower SMACH state machine.

Contains TF utilities, MBF result classification, path segmentisation,
safe blade shutdown, and constants.
"""
import math
import rospy
import tf
import shapely
from shapely import geometry
import PyKDL as kdl
import actionlib
from nav_msgs.msg import Path
from std_msgs.msg import Bool, String, Int16
from geometry_msgs.msg import PoseStamped, Quaternion
from vitulus_msgs.msg import Mower
import mbf_msgs.msg as mbf_msgs


# ---------------------------------------------------------------------------
# Mower FW status strings
# ---------------------------------------------------------------------------
MOWER_ERROR_STATES = ('ERR', 'BLOCKED', 'TEMP')


# ---------------------------------------------------------------------------
# Shared TF listener (lazy singleton)
# ---------------------------------------------------------------------------
_tf_listener_singleton = None


def get_tf_listener():
    """Return a process-wide ``tf.TransformListener``, creating it on first call."""
    global _tf_listener_singleton
    if _tf_listener_singleton is None:
        _tf_listener_singleton = tf.TransformListener()
    return _tf_listener_singleton


def get_robot_xy_in_map(timeout=0.2):
    """Return (x, y) of base_link in the map frame, or None on lookup failure."""
    listener = get_tf_listener()
    try:
        listener.waitForTransform('map', 'base_link', rospy.Time(), rospy.Duration(timeout))
        (trans, _rot) = listener.lookupTransform('map', 'base_link', rospy.Time())
        return (trans[0], trans[1])
    except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
        return None


# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------
def nearest_pose_index(path, x, y, hint_index=0, search_radius=400):
    """Find the index in ``path.poses`` closest to (x, y).

    The search is bounded to ``[hint_index, hint_index + search_radius]`` so we
    never accidentally jump backwards in the mowing path when the same path
    crosses itself (e.g. spiral coverage patterns).
    """
    if not path or not path.poses:
        return 0
    n = len(path.poses)
    lo = max(0, int(hint_index) - 5)
    hi = min(n, int(hint_index) + int(search_radius))
    if hi <= lo:
        return min(int(hint_index), n - 1)
    best_i = lo
    best_d2 = float('inf')
    for i in range(lo, hi):
        p = path.poses[i].pose.position
        d2 = (p.x - x) ** 2 + (p.y - y) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_i = i
    return best_i


def distance_xy(ax, ay, bx, by):
    return math.hypot(ax - bx, ay - by)


def direction_angle(line_coords):
    """Compute heading angle from a 2-point coordinate list [[x0,y0],[x1,y1]]."""
    x = line_coords[1][0] - line_coords[0][0]
    y = line_coords[1][1] - line_coords[0][1]
    diagonal = math.sqrt(x ** 2 + y ** 2)
    if diagonal == 0:
        return 0.0
    if y < 0:
        if x >= 0:
            return math.asin(y / diagonal)
        return math.acos(x / diagonal) * -1
    if x >= 0:
        return math.asin(y / diagonal)
    return math.acos(x / diagonal)


# ---------------------------------------------------------------------------
# MBF result classification
# ---------------------------------------------------------------------------
def classify_mbf_result(action_name, status, result, log_pub=None):
    """Map an MBF action result to ``'succeeded'`` / ``'aborted'``.

    Logs the actual MBF outcome and message on failure.
    """
    succeeded_status = (status == actionlib.GoalStatus.SUCCEEDED)
    outcome = getattr(result, 'outcome', None) if result is not None else None
    message = getattr(result, 'message', '') if result is not None else ''
    if succeeded_status and outcome == 0:
        return 'succeeded'
    rospy.logwarn("[%s] aborted: status=%s outcome=%s message=%r",
                  action_name, status, outcome, message)
    if log_pub is not None:
        try:
            log_pub.publish(String("{} aborted (outcome={})".format(action_name, outcome)))
        except Exception:
            pass
    return 'aborted'


# ---------------------------------------------------------------------------
# Path segmentisation (shared by GetPathData and resume logic)
# ---------------------------------------------------------------------------
def segmentize_raw_path(raw_path, segmentize_distance=0.03, outline_start_offset=0.18):
    """Segmentize a raw planner path the same way GetPathData does.

    Returns a Path with poses spaced ~segmentize_distance apart.
    """
    line = geometry.LineString(
        [[p.pose.position.x, p.pose.position.y] for p in raw_path.poses])
    path_new = Path()
    path_new.header.frame_id = "map"
    path_new.header.stamp = rospy.Time.now()

    if line.length == 0:
        # Outline (closed polygon) path
        start_poses_offset = int(round(outline_start_offset / segmentize_distance))
        polygon = geometry.LinearRing(
            [[p.pose.position.x, p.pose.position.y] for p in raw_path.poses])
        polygon = shapely.remove_repeated_points(polygon)
        for n in range(0, len(polygon.coords) - 1):
            seg = geometry.LineString(
                [[polygon.coords[n][0], polygon.coords[n][1]],
                 [polygon.coords[n + 1][0], polygon.coords[n + 1][1]]])
            angle = direction_angle(list(seg.coords))
            seg = seg.segmentize(segmentize_distance)
            for point in seg.coords:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = point[0]
                pose.pose.position.y = point[1]
                pose.pose.position.z = 0.0
                pose.pose.orientation = Quaternion(
                    *(kdl.Rotation.RPY(0, 0, angle).GetQuaternion()))
                path_new.poses.append(pose)
        if start_poses_offset > 0 and len(path_new.poses) > 2 * start_poses_offset:
            path_new.poses = path_new.poses[start_poses_offset:-start_poses_offset]
        return path_new

    # Coverage infill path
    line = line.segmentize(segmentize_distance)
    for pose_id in range(0, len(line.coords) - 1):
        angle = direction_angle(
            [[line.coords[pose_id][0], line.coords[pose_id][1]],
             [line.coords[pose_id + 1][0], line.coords[pose_id + 1][1]]])
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = line.coords[pose_id][0]
        pose.pose.position.y = line.coords[pose_id][1]
        pose.pose.position.z = 0.0
        pose.pose.orientation = Quaternion(
            *(kdl.Rotation.RPY(0, 0, angle).GetQuaternion()))
        path_new.poses.append(pose)
    return path_new


# ---------------------------------------------------------------------------
# Safe blade shutdown
# ---------------------------------------------------------------------------
def safe_blade_shutdown(source='unknown', timeout_motor=5.0, timeout_height=30.0, pubs=None):
    """Stop the cutting motor and raise the blade to max_height before power-off.

    The mower v4 firmware automatically triggers a HOME cycle whenever
    ``/mower/set_power(False)`` is published while the blade is not parked at
    ``max_height``.  This helper suppresses that by:
      1. Publishing ``set_motor_on(False)`` and waiting for motor stop.
      2. Publishing ``set_cut_height(max_height)`` and waiting for arrival.

    Idempotent: a second call is a cheap no-op once the blade is at max_height.

    Args:
        pubs: Optional MowerPublishers instance. When provided, uses its
              pre-existing publishers instead of creating temporary ones
              (avoids publisher leak on repeated calls).

    Returns True on success, False on timeout.
    """
    caller = rospy.get_caller_id()
    log_prefix = '[{}][safe_blade_shutdown:{}]'.format(caller, source)

    # 1. Snapshot current mower state
    max_height = 80
    current_height = -1
    status0 = 'UNKNOWN'
    rpm0 = -1
    try:
        s0 = rospy.wait_for_message('/mower/status', Mower, timeout=5.0)
        max_height = int(s0.max_height) if s0.max_height else 80
        current_height = int(s0.current_height)
        status0 = s0.status
        rpm0 = int(s0.moto_rpm)
    except rospy.ROSException as e:
        rospy.logwarn('{} cannot read /mower/status ({}); using fallback max_height=80'.format(
            log_prefix, e))

    rospy.loginfo('{} entry: status={} current_height={} max_height={} rpm={}'.format(
        log_prefix, status0, current_height, max_height, rpm0))

    # 2. Stop the cutting motor
    if pubs is not None:
        pub_motor = pubs.mower_set_motor_on
    else:
        pub_motor = rospy.Publisher('/mower/set_motor_on', Bool, latch=True, queue_size=1)
        rospy.sleep(0.1)
    pub_motor.publish(Bool(False))
    rospy.loginfo('{} published set_motor_on(False)'.format(log_prefix))

    motor_off_deadline = rospy.Time.now() + rospy.Duration(timeout_motor)
    motor_stopped = (status0 != 'RUN' and rpm0 == 0)
    while not rospy.is_shutdown() and not motor_stopped and rospy.Time.now() < motor_off_deadline:
        try:
            s = rospy.wait_for_message('/mower/status', Mower, timeout=1.0)
            if s.status != 'RUN' and int(s.moto_rpm) == 0:
                motor_stopped = True
                current_height = int(s.current_height)
                rospy.loginfo('{} motor stopped (status={}, height={})'.format(
                    log_prefix, s.status, current_height))
                break
        except rospy.ROSException:
            continue
    if not motor_stopped:
        rospy.logwarn('{} timeout waiting for motor to stop'.format(log_prefix))

    # 3. Raise blade to max_height if needed
    if current_height == max_height:
        rospy.loginfo('{} blade already at max_height={}'.format(log_prefix, max_height))
        return True

    if pubs is not None:
        pub_height = pubs.mower_set_height
    else:
        pub_height = rospy.Publisher('/mower/set_cut_height', Int16, latch=True, queue_size=1)
        rospy.sleep(0.1)
    pub_height.publish(Int16(max_height))
    rospy.loginfo('{} published set_cut_height({}) (was {})'.format(
        log_prefix, max_height, current_height))

    height_deadline = rospy.Time.now() + rospy.Duration(timeout_height)
    last_log = rospy.Time.now()
    while not rospy.is_shutdown() and rospy.Time.now() < height_deadline:
        try:
            s = rospy.wait_for_message('/mower/status', Mower, timeout=1.0)
            if int(s.current_height) == max_height:
                rospy.loginfo('{} blade reached max_height={}'.format(log_prefix, max_height))
                return True
            if (rospy.Time.now() - last_log).to_sec() > 3.0:
                rospy.loginfo('{} waiting: current_height={} target={} status={}'.format(
                    log_prefix, s.current_height, max_height, s.status))
                last_log = rospy.Time.now()
        except rospy.ROSException:
            continue

    rospy.logwarn('{} TIMEOUT: blade did not reach max_height={} within {}s'.format(
        log_prefix, max_height, timeout_height))
    return False


# ---------------------------------------------------------------------------
# MBF availability check
# ---------------------------------------------------------------------------
def wait_for_mbf():
    client = actionlib.SimpleActionClient("/move_base_flex/move_base", mbf_msgs.MoveBaseAction)
    available = client.wait_for_server(rospy.Duration(30))
    if not available:
        rospy.logwarn("Move Base Flex is not available")
    else:
        rospy.loginfo("Move Base Flex is ready")
    return available
