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
from mbf_msgs.srv import CheckPathRequest, CheckPathResponse


# ---------------------------------------------------------------------------
# Mower FW status strings
# ---------------------------------------------------------------------------
MOWER_ERROR_STATES = ('ERR', 'BLOCKED', 'TEMP')


# ---------------------------------------------------------------------------
# MBF outcome code -> human-readable name (for debug logging)
# Mirrors mbf_msgs ExePath/GetPath result outcome constants.
# ---------------------------------------------------------------------------
MBF_OUTCOME_NAMES = {
    0: 'SUCCESS',
    50: 'FAILURE', 51: 'CANCELED', 52: 'NO_VALID_CMD', 53: 'PAT_EXCEEDED',
    54: 'COLLISION', 55: 'OSCILLATION', 56: 'ROBOT_STUCK', 57: 'MISSED_GOAL',
    58: 'MISSED_PATH', 59: 'BLOCKED_PATH', 60: 'INVALID_PATH', 61: 'TF_ERROR',
    62: 'NOT_INITIALIZED', 63: 'INVALID_PLUGIN', 64: 'INTERNAL_ERROR',
    65: 'OUT_OF_MAP', 66: 'MAP_ERROR', 67: 'STOPPED',
    # GetPath-specific
    68: 'CANCELED', 69: 'EMPTY_PATH',
}


def mbf_outcome_str(code):
    """Return 'NAME(code)' for an MBF outcome code, or 'None' if code is None."""
    if code is None:
        return 'None'
    return '{}({})'.format(MBF_OUTCOME_NAMES.get(code, 'UNKNOWN'), code)


# ---------------------------------------------------------------------------
# Weather (rain) gating configuration
#
# Read from ROS params (set in navi_man.launch under the node's private
# namespace) so the rain behaviour can be tuned without editing code:
#   ~weather_rain_bypass          bool  - if true, skip ALL rain checks
#   ~weather_reject_statuses      list  - forecast statuses that block mowing
#                                         (default ['RAIN']; was RAIN/ALERT/WARN)
#   ~weather_forecast_horizon_min int   - how far ahead to look: 10/20/30 min
#                                         (the RainAlert msg only goes to 30)
#   ~weather_monitor_min_steps    int   - during a mission, how many forecast
#                                         steps must be "reject" to return to
#                                         dock (pre-start rejects on any 1 step)
# ---------------------------------------------------------------------------

_NOWCAST_FIELDS = ('status_nowcast10m', 'status_nowcast20m', 'status_nowcast30m')


class RunFlags(object):
    """What master_controller tells the running mission. Without the master
    (manual programs, tests) both stay False.

      scheduled          /scheduler/run_active: this is a calendar run, the rain
                         checks hold even with ~weather_rain_bypass
      finish_after_path  /mower_smach/finish_after_path: go home at the next
                         path boundary (see stop_at_path_boundary)
    """

    def __init__(self):
        self.scheduled = False
        self.finish_after_path = False
        self.home_latch = None
        self._subs = []

    def start(self):
        self._subs = [
            rospy.Subscriber('/scheduler/run_active', Bool, self._on_scheduled,
                             queue_size=1),
            rospy.Subscriber('/mower_smach/finish_after_path', Bool, self._on_finish,
                             queue_size=1)]

    def _on_scheduled(self, msg):
        self.scheduled = bool(msg.data)

    def _on_finish(self, msg):
        self.finish_after_path = bool(msg.data)


run_flags = RunFlags()


def mark_run_phase(program, phase):
    """Record which start phase the mission is in, unless the program carries a
    path checkpoint: that one is the resume point until mowing moves it, and a
    resume that fails before the first path must leave it in place."""
    if not (program.last_result or '').startswith('failed: on_path-'):
        program.last_result = 'failed: {}'.format(phase)


def stop_at_path_boundary(state, userdata, pubs, zone_name, path_idx):
    """Going home was requested and a path has just ended: the next path is the
    checkpoint, then the mission child is preempted like by a monitor. Returns
    True when the caller has to return 'preempted'."""
    latch = run_flags.home_latch
    if not run_flags.finish_after_path or latch is None:
        return False
    userdata.program.last_result = 'failed: on_path-{}-{}-0'.format(zone_name, path_idx)
    pubs.save_program.publish(userdata.program)
    pubs.log_info.publish(String("Window closed, going home"))
    run_flags.finish_after_path = False
    latch.request('window')
    rospy.sleep(0.2)
    if state.preempt_requested():
        state.service_preempt()
        return True
    return False


def get_weather_config():
    """Read rain-gating params with safe defaults. Called per check so a live
    `rosparam set` takes effect without restarting the node.

    The bypass is an override for manual runs: a calendar run always obeys the
    rain rules, nobody is there to watch it."""
    bypass = (bool(rospy.get_param('~weather_rain_bypass', False))
              and not run_flags.scheduled)

    statuses = rospy.get_param('~weather_reject_statuses', ['RAIN'])
    if isinstance(statuses, str):
        statuses = [s.strip() for s in statuses.split(',') if s.strip()]
    reject_statuses = tuple(str(s).upper() for s in statuses) or ('RAIN',)

    horizon = int(rospy.get_param('~weather_forecast_horizon_min', 30))
    # The RainAlert message only carries 10/20/30-minute nowcast steps.
    if horizon < 10:
        horizon = 10
    horizon = min(horizon, 30)
    n_steps = max(1, horizon // 10)

    min_steps = int(rospy.get_param('~weather_monitor_min_steps', 2))
    if min_steps < 1:
        min_steps = 1

    return {
        'bypass': bypass,
        'reject_statuses': reject_statuses,
        'horizon_min': horizon,
        'n_steps': n_steps,
        'monitor_min_steps': min_steps,
        'max_age_s': float(rospy.get_param('~weather_max_age_min', 30.0)) * 60.0,
    }


def rain_data_fresh(rain_msg, cfg):
    """False for a RainAlert older than ~weather_max_age_min: the node keeps
    publishing its last result when the radar source is unreachable."""
    stamp = getattr(rain_msg, 'time', None)
    if stamp is None or stamp.to_sec() <= 0:
        return True
    return (rospy.Time.now() - stamp).to_sec() <= cfg['max_age_s']


def rain_forecast_steps(rain_msg, n_steps):
    """Return the first `n_steps` nowcast status strings from a RainAlert msg."""
    return [getattr(rain_msg, f, '') for f in _NOWCAST_FIELDS[:n_steps]]


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
# Live costmap path collision probe
# ---------------------------------------------------------------------------
def first_lethal_index(check_path_proxy, path, safety_dist=0.10,
                       use_padded_fp=False):
    """Index of the first LETHAL pose on *path* per the live GLOBAL costmap, or
    -1 if the path is clear / the service is unavailable.

    Wraps move_base_flex's check_path_cost service (return_on=LETHAL). Gates on
    state==LETHAL only: unknown space is ignored (unknown_cost_mult=0) and
    inscribed/inflated cells (e.g. the static map walls an outline path runs
    alongside) do NOT trip it — only real obstacle cells touched by the path do.
    """
    if path is None or not getattr(path, 'poses', None):
        return -1
    try:
        req = CheckPathRequest()
        req.path = path
        req.costmap = CheckPathRequest.GLOBAL_COSTMAP
        req.return_on = CheckPathResponse.LETHAL  # stop at first lethal pose
        req.lethal_cost_mult = 1
        req.inscrib_cost_mult = 1
        req.unknown_cost_mult = 0   # ignore unknown space (common outdoors)
        req.safety_dist = safety_dist
        req.skip_poses = 0
        req.use_padded_fp = use_padded_fp
        req.path_cells_only = False
        resp = check_path_proxy(req)
    except (rospy.ServiceException, rospy.ROSException) as e:
        rospy.logwarn_throttle(
            10.0, '[first_lethal_index] check_path_cost unavailable: %s', e)
        return -1
    if resp.state == CheckPathResponse.LETHAL:
        return int(resp.last_checked)
    return -1


# ---------------------------------------------------------------------------
# MBF result classification
# ---------------------------------------------------------------------------
def classify_mbf_result(action_name, status, result, log_pub=None):
    """Map an MBF action result to ``'succeeded'`` / ``'aborted'`` / ``None``.

    Logs the actual MBF outcome and message on failure.

    A PREEMPTED goal returns None: smach_ros.SimpleActionState then decides
    itself - 'preempted' when the container asked for the preempt (STOP,
    battery, weather...), 'aborted' otherwise. SimpleActionState services
    the preempt BEFORE the result_cb outcome replaces its own, so returning
    'aborted' here swallowed the request and the mission went on to recovery
    after a user STOP (vitulus-field#53).
    """
    succeeded_status = (status == actionlib.GoalStatus.SUCCEEDED)
    outcome = getattr(result, 'outcome', None) if result is not None else None
    message = getattr(result, 'message', '') if result is not None else ''
    if succeeded_status and outcome == 0:
        return 'succeeded'
    if status == actionlib.GoalStatus.PREEMPTED:
        rospy.logwarn("[%s] preempted: outcome=%s message=%r",
                      action_name, outcome, message)
        return None
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
# Mower power evidence (18. 10. 2026)
#
# A STOP sent to an idle mower with no power waited 5 s for /mower/status,
# 5 s for the motor and 30 s for the blade height - for a board that cannot
# answer - and only then went on.  The watch below keeps the latest evidence
# from persistent subscribers so safe_blade_shutdown can tell "verified
# unpowered" at once instead of waiting for timeouts.
# ---------------------------------------------------------------------------
import threading as _threading
import time as _time

POWER_EVIDENCE_MAX_AGE_S = 3.0     # /pm/power_status younger than this counts
STATUS_SILENT_S = 3.0              # no /mower/status this long = silent


class MowerPowerWatch(object):
    """Latest /mower/status and /pm/power_status bat_out_switch (monotonic)."""

    def __init__(self, clock=_time.monotonic):
        self.clock = clock
        self.lock = _threading.Lock()
        self.started = None
        self.last_status = None          # (t, Mower msg)
        self.last_bat_out = None         # (t, bool)

    def start(self):
        self.started = self.clock()
        rospy.Subscriber('/mower/status', Mower, self.on_status, queue_size=1)
        try:
            from vitulus_ups.msg import power_status
            rospy.Subscriber('/pm/power_status', power_status, self.on_power,
                             queue_size=1)
        except ImportError:
            rospy.logwarn('[MowerPowerWatch] vitulus_ups/power_status unavailable')
        return self

    def on_status(self, msg):
        with self.lock:
            self.last_status = (self.clock(), msg)

    def on_power(self, msg):
        with self.lock:
            self.last_bat_out = (self.clock(), bool(getattr(msg, 'bat_out_switch', True)))

    def fresh_status(self, max_age=STATUS_SILENT_S):
        with self.lock:
            last = self.last_status
        if last is not None and self.clock() - last[0] <= max_age:
            return last[1]
        return None

    def unpowered(self, program_running=True):
        """(True, why) when the mower verifiably has no power, else (False, '').

        * /pm/power_status bat_out_switch false (fresh) - always enough.
        * /mower/status silent for STATUS_SILENT_S - enough only when no
          mowing program runs (during a program silence may be a comm fault
          with the blade spinning, so the full shutdown is kept).
        """
        now = self.clock()
        with self.lock:
            bat = self.last_bat_out
            status = self.last_status
        if bat is not None and now - bat[0] <= POWER_EVIDENCE_MAX_AGE_S and not bat[1]:
            return True, 'bat_out_switch=false'
        if not program_running and self.started is not None:
            silent_since = status[0] if status is not None else self.started
            if now - silent_since >= STATUS_SILENT_S:
                return True, '/mower/status silent %.0f s' % (now - silent_since)
        return False, ''


_power_watch = {'watch': None}


def start_power_watch():
    """Start the process-wide MowerPowerWatch once (node init)."""
    if _power_watch['watch'] is None:
        _power_watch['watch'] = MowerPowerWatch().start()
    return _power_watch['watch']


def power_watch():
    return _power_watch['watch']


# ---------------------------------------------------------------------------
# Safe blade shutdown
# ---------------------------------------------------------------------------
def safe_blade_shutdown(source='unknown', timeout_motor=5.0, timeout_height=30.0, pubs=None,
                        program_running=True):
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

        program_running: False when no mowing program runs (the caller knows).
              With the power watch running (start_power_watch), a mower that
              verifiably has no power (MowerPowerWatch.unpowered) gets only
              set_motor_on(False) and the call returns at once - nothing can
              spin and nothing can HOME without power.

    Returns True on success, False on timeout.
    """
    caller = rospy.get_caller_id()
    log_prefix = '[{}][safe_blade_shutdown:{}]'.format(caller, source)

    if pubs is not None:
        pub_motor = pubs.mower_set_motor_on
    else:
        pub_motor = rospy.Publisher('/mower/set_motor_on', Bool, latch=True, queue_size=1)
        rospy.sleep(0.1)

    watch = power_watch()
    s0 = None
    if watch is not None:
        off, why = watch.unpowered(program_running=program_running)
        if off:
            pub_motor.publish(Bool(False))
            rospy.loginfo('{} mower unpowered ({}): set_motor_on(False) sent, '
                          'nothing to wait for'.format(log_prefix, why))
            return True
        s0 = watch.fresh_status()

    # 1. Snapshot current mower state
    max_height = 80
    current_height = -1
    status0 = 'UNKNOWN'
    rpm0 = -1
    try:
        if s0 is None:
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
