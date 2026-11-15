"""Offline replay of a recorded drive through the code of nodes/navi_transform
(T36). No ROS master, no node init: rospy's parameter / publisher / subscriber
calls are replaced by stubs, the node is built by its real __init__ (so every
parameter gets the default the node gives it) and the recorded messages are fed
to the node's own callbacks in the order of the recording; the real loop30hz
runs at the recorded /tf times.

Data: an excerpt of bag magtest3_2026-11-14 (data/t36_magtest3_excerpt.npz,
made by t36_make_excerpt.py from the analyst's extract). Arrays, first column =
bag time:
  imu  (t, stamp, yaw, gyro_z)          /bno085/imu
  nh   (t, stamp, yaw)                  /gnss_heading/navheading
  hrel (t, iTOW, flags, heading, len, accHeading_deg, N, E)  /gnss_heading/navrelposned
  ekf  (t, stamp, x, y, z, yaw, vx, wz) /odometry/ekf_wheel_nav_odom
  wo   (t, stamp, x, y, z, yaw, vx, wz) /mobile_base_controller/odom
  dock (t, status)                      /dock_smach/dock_status
  mo, ob (t, stamp, x, y, z, yaw)       recorded /tf map->odom, odom->base_link
  loc_t, loc_json                       /nav_tf/loc_status

Not replayed (not in the extract): the GPS position path (loop3sec set_pose,
callback_map_coords georef), the tracker. The recorded map->odom can be pinned
until a given time instead (pin_until), which stands for whatever those wrote.
/gnss/fix and /gnss_heading/fix are not in the extract: fix_usable and
heading_fix_usable are taken from loc_status (1 Hz).

The EKF is not simulated: the recorded /odometry/ekf_wheel_nav_odom is fed as
it was (it answered the recorded /gnss_heading/nav_fused). That is exact as
long as the replayed diff_yaw equals the recorded one; ekf_model replaces the
EKF yaw by imu_yaw + diff_yaw delayed by a given time for runs where it does
not.
"""
import importlib.machinery
import importlib.util
import json
import math
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _ros_env  # noqa: E402
_ros_env.ensure()

import numpy as np  # noqa: E402

NAVI_TF = os.path.join(HERE, '..', 'nodes', 'navi_transform')
EXCERPT = os.path.join(HERE, 'data', 't36_magtest3_excerpt.npz')
MAIN_REF = '30eca80'          # main the T36 batch started from
CLOCK = [0.0]
PARAMS = {}
D = math.radians


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def _stub_missing_msgs():
    """Dev container without the robot's full devel space (module-level imports)."""
    try:
        import sensor_msgs_ext.msg  # noqa: F401
    except ImportError:
        pkg = types.ModuleType('sensor_msgs_ext')
        msg = types.ModuleType('sensor_msgs_ext.msg')
        msg.magnetometer = msg.accelerometer = object
        pkg.msg = msg
        sys.modules['sensor_msgs_ext'] = pkg
        sys.modules['sensor_msgs_ext.msg'] = msg
    try:
        import vitulus_navi.srv  # noqa: F401
    except ImportError:
        try:
            import vitulus_navi as pkg
        except ImportError:
            pkg = types.ModuleType('vitulus_navi')
            pkg.__path__ = []
            sys.modules['vitulus_navi'] = pkg
        srv = types.ModuleType('vitulus_navi.srv')
        srv.Point_pose = srv.Point_poseResponse = object
        pkg.srv = srv
        sys.modules['vitulus_navi.srv'] = srv


_stub_missing_msgs()

import rospy  # noqa: E402
import tf2_ros  # noqa: E402
from geometry_msgs.msg import TransformStamped  # noqa: E402
from nav_msgs.msg import Odometry  # noqa: E402
from sensor_msgs.msg import Imu  # noqa: E402


class FakePub(object):
    """Keeps the last message and a count; `keep` collects all of them."""
    def __init__(self, *a, **k):
        self.name = a[0] if a else ''
        self.last = None
        self.n = 0
        self.keep = None

    def publish(self, msg):
        self.last = msg
        self.n += 1
        if self.keep is not None:
            self.keep.append(msg)


class _Nothing(object):
    def __init__(self, *a, **k):
        pass

    def __getattr__(self, name):
        return lambda *a, **k: None


class FakeBroadcaster(object):
    def __init__(self, *a, **k):
        self.last = None
        self.n = 0

    def sendTransform(self, t):
        self.last = t if isinstance(t, list) else [t]
        self.n += 1


class FakeBuffer(object):
    def __init__(self, *a, **k):
        pass

    def lookup_transform(self, *a, **k):
        raise RuntimeError('no tf in the replay')


def _get_param(name, default=None):
    key = name.lstrip('~')
    if key in PARAMS:
        return PARAMS[key]
    return default


_INSTALLED = [False]


def install_stubs():
    """Replace what the node needs from rospy / tf2_ros by offline stubs. The
    stubs are process-wide (the module under test and the reference module
    share them)."""
    if _INSTALLED[0]:
        return
    _INSTALLED[0] = True
    rospy.Time.now = staticmethod(lambda: rospy.Time.from_sec(CLOCK[0]))
    rospy.get_time = lambda: CLOCK[0]
    rospy.get_param = _get_param
    rospy.Publisher = FakePub
    rospy.Subscriber = _Nothing
    rospy.Service = _Nothing
    rospy.ServiceProxy = _Nothing
    for f in ('loginfo', 'logwarn', 'logerr', 'logdebug', 'loginfo_throttle',
              'logwarn_throttle', 'logerr_throttle', 'logdebug_throttle'):
        setattr(rospy, f, lambda *a, **k: None)
    tf2_ros.Buffer = FakeBuffer
    tf2_ros.TransformListener = _Nothing
    tf2_ros.StaticTransformBroadcaster = _Nothing
    tf2_ros.TransformBroadcaster = FakeBroadcaster


def load(name, path=NAVI_TF):
    """Import a navi_transform source file as a module (stubs installed)."""
    install_stubs()
    spec = importlib.util.spec_from_loader(
        name, importlib.machinery.SourceFileLoader(name, path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    mod.TransformListener = _Nothing                       # tf.TransformListener
    mod.time = types.SimpleNamespace(time=lambda: CLOCK[0])  # node wall clock = replay clock
    mod.NavTfNode.roslaunch_ekf = lambda self: None
    return mod


def load_ref(name='navi_transform_ref', ref=MAIN_REF):
    """The node as it was on main before T36 (git show <ref>:nodes/navi_transform).
    None when git or the commit is not available."""
    import subprocess
    import tempfile
    try:
        src = subprocess.check_output(
            ['git', '-C', os.path.join(HERE, '..'), 'show', ref + ':nodes/navi_transform'],
            stderr=subprocess.DEVNULL)
    except Exception:
        return None
    fd, path = tempfile.mkstemp(suffix='_navi_transform_' + ref)
    os.write(fd, src)
    os.close(fd)
    try:
        return load(name, path)
    finally:
        os.unlink(path)


def load_data(path=None):
    z = np.load(path or os.environ.get('T36_NPZ') or EXCERPT, allow_pickle=True)
    d = {k: z[k] for k in z.files}
    d['loc'] = [json.loads(s) for s in d['loc_json']]
    d['T0'] = float(d['T0']) if 'T0' in d else float(d['ob'][0, 0])
    return d


def _yaw_quat(o, yaw):
    o.x, o.y = 0.0, 0.0
    o.z, o.w = math.sin(yaw / 2.0), math.cos(yaw / 2.0)


def odom_msg(stamp, x, y, z, yaw, vx, wz):
    m = Odometry()
    m.header.stamp = rospy.Time.from_sec(stamp)
    p = m.pose.pose
    p.position.x, p.position.y, p.position.z = x, y, z
    _yaw_quat(p.orientation, yaw)
    m.twist.twist.linear.x = vx
    m.twist.twist.angular.z = wz
    return m


def imu_msg(stamp, yaw, wz):
    m = Imu()
    m.header.stamp = rospy.Time.from_sec(stamp)
    _yaw_quat(m.orientation, yaw)
    m.angular_velocity.z = wz
    return m


def navheading_msg(stamp, yaw, acc_deg):
    """ublox driver: orientation_covariance[8] = accHeading^2 (rad^2)."""
    m = Imu()
    m.header.stamp = rospy.Time.from_sec(stamp)
    _yaw_quat(m.orientation, yaw)
    cov = [0.0] * 9
    cov[8] = math.radians(acc_deg) ** 2
    m.orientation_covariance = cov
    return m


def quat_yaw(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def make_node(nt, params=None):
    """The node through its real __init__, offline."""
    PARAMS.clear()
    PARAMS.update(params or {})
    n = nt.NavTfNode()
    n.publish_odom_for_gps = lambda: None      # needs navsat / fused msg plumbing
    n.init_navsat = lambda: None               # would roslaunch navsat
    n.nav_last_fused_time = rospy.Time.from_sec(CLOCK[0])
    n.fix_last_fused_time = rospy.Time.from_sec(CLOCK[0])
    n.is_indoor = False
    return n


def dock_anchor_from(data, t_rel):
    """The DOCKED waypoint as the recording shows it: map->base at a time the
    dock hold was active."""
    mo, ob = data['mo'], data['ob']
    i = int(np.searchsorted(ob[:, 0], data['T0'] + t_rel))
    m, o = mo[i], ob[i]
    c, s = math.cos(m[5]), math.sin(m[5])
    return (m[2] + c * o[2] - s * o[3], m[3] + s * o[2] + c * o[3], wrap(m[5] + o[5]))


def replay(nt, data, t0, t1, params=None, pin_until=None, dock_anchor=None,
           ekf_delay=None, hook=None, setup=None):
    """Replay data[t0..t1] (seconds from the start of the recording) through the
    node code `nt`. Returns (trace, node). trace: dict of numpy arrays, one row
    per 30 Hz tick:
      t                 time from the start of the recording
      mo_x, mo_y, mo_yaw    broadcast map->odom
      ob_x, ob_y, ob_yaw    broadcast odom->base_link
      diff_yaw, map_odom_yaw    node state after the tick
      od_v, od_w        twist of the published /odometry/odom (lin.x, ang.z)
      od_x, od_y, od_yaw    its pose
      fused_yaw         yaw of the last /gnss_heading/nav_fused
    plus trace['n_heading'] (navheading messages fed) and trace['hdg_t']
    (times _apply_heading_target ran).
    pin_until: until that time the node's map->odom is overwritten by the
    recorded one before every tick. ekf_delay: None = recorded EKF yaw, else
    EKF yaw = imu_yaw + diff_yaw as it was ekf_delay seconds ago.
    hook(node, t_rel, kind) is called after every fed message / tick."""
    T0 = data['T0']
    a, b = T0 + t0, T0 + t1
    ev = []
    for prio, key in enumerate(('dock', 'wo', 'imu', 'hrel', 'nh', 'ekf', 'ob')):
        arr = data[key]
        if len(arr) == 0:
            continue
        i0, i1 = np.searchsorted(arr[:, 0], [a, b])
        ev.extend((arr[i, 0], prio, key, i) for i in range(i0, i1))
    ev.sort(key=lambda e: (e[0], e[1]))
    CLOCK[0] = a
    n = make_node(nt, params)
    n.init = False                       # the dock hold anchored the heading long ago
    # state at t0 from the recording
    mo, ekf, imu = data['mo'], data['ekf'], data['imu']
    im = int(np.searchsorted(mo[:, 0], a))
    ie = max(0, int(np.searchsorted(ekf[:, 0], a)) - 1)
    ii = max(0, int(np.searchsorted(imu[:, 0], a)) - 1)
    n.map_odom_transform.transform.translation.x = mo[im, 2]
    n.map_odom_transform.transform.translation.y = mo[im, 3]
    n.map_odom_yaw = float(mo[im, 5])
    n._map_odom_placed = True
    n.diff_yaw = wrap(ekf[ie, 5] - imu[ii, 2])
    n.callback_ekf_wheel_nav_odom(odom_msg(*ekf[ie, 1:8]))
    n._dock_anchor_pose = dock_anchor
    n._dock_anchor_fetch_t = a
    last_ob = [None]
    n.get_base_link_transform = lambda stamp=None: last_ob[0]   # tf: what the last tick sent
    applied = []
    real_apply = n._apply_heading_target

    def counted(target):
        applied.append(CLOCK[0] - T0)
        return real_apply(target)
    n._apply_heading_target = counted
    if setup is not None:
        setup(n)
    loc_t, loc = data['loc_t'], data['loc']
    li = [max(0, int(np.searchsorted(loc_t, a)) - 1)]
    fused_hist = []                      # (t, diff_yaw) for ekf_delay
    cols = ('t', 'mo_x', 'mo_y', 'mo_yaw', 'ob_x', 'ob_y', 'ob_yaw', 'diff_yaw',
            'map_odom_yaw', 'od_v', 'od_w', 'od_x', 'od_y', 'od_yaw', 'fused_yaw')
    rows = []
    n_heading = 0
    for t, _prio, key, i in ev:
        CLOCK[0] = t
        while li[0] + 1 < len(loc_t) and loc_t[li[0] + 1] <= t:
            li[0] += 1
        j = loc[li[0]]
        n.fix_usable = bool(j.get('fix_usable'))
        n.heading_fix_usable = bool(j.get('heading_ok'))
        r = data[key][i]
        if key == 'imu':
            n.callback_imu(imu_msg(r[1], r[2], r[3]))
            if ekf_delay is not None:
                fused_hist.append((t, n.diff_yaw, r[2]))
        elif key == 'hrel':
            n.callback_relposned(types.SimpleNamespace(flags=int(r[2])))
        elif key == 'nh':
            k = int(np.argmin(np.abs(data['hrel'][:, 0] - t)))
            n_heading += 1
            n.callback_navheading(navheading_msg(r[1], r[2], data['hrel'][k, 5]))
        elif key == 'wo':
            n.callback_wheel_odom(odom_msg(*r[1:8]))
        elif key == 'dock':
            n.callback_dock_status(types.SimpleNamespace(data=int(r[1])))
        elif key == 'ekf':
            yaw = r[5]
            if ekf_delay is not None and fused_hist:
                k = len(fused_hist) - 1
                while k > 0 and fused_hist[k][0] > t - ekf_delay:
                    k -= 1
                yaw = wrap(fused_hist[-1][2] + fused_hist[k][1])
                if len(fused_hist) > 400:
                    del fused_hist[:200]
            n.callback_ekf_wheel_nav_odom(odom_msg(r[1], r[2], r[3], r[4], yaw, r[6], r[7]))
        elif key == 'ob':
            if pin_until is not None and (t - T0) < pin_until:
                m = data['mo'][i]
                n.map_odom_transform.transform.translation.x = m[2]
                n.map_odom_transform.transform.translation.y = m[3]
                n.map_odom_yaw = float(m[5])
            n.loop30hz()
            sent = n.tf2_TransformBroadcaster.last
            tm = [x for x in sent if x.child_frame_id == 'odom'][0].transform
            to = [x for x in sent if x.child_frame_id == 'base_link'][0]
            last_ob[0] = to
            od = n.odometry_odom_pub.last
            fm = n.fused_nav_pub.last
            rows.append((t - T0, tm.translation.x, tm.translation.y, quat_yaw(tm.rotation),
                         to.transform.translation.x, to.transform.translation.y,
                         quat_yaw(to.transform.rotation), n.diff_yaw, n.map_odom_yaw,
                         od.twist.twist.linear.x, od.twist.twist.angular.z,
                         od.pose.pose.position.x, od.pose.pose.position.y,
                         quat_yaw(od.pose.pose.orientation),
                         quat_yaw(fm.orientation) if fm is not None else 0.0))
        if hook is not None:
            hook(n, t - T0, key)
    arr = np.array(rows, dtype=float)
    trace = {c: arr[:, k] for k, c in enumerate(cols)}
    trace['n_heading'] = n_heading
    trace['hdg_t'] = np.array(applied)
    return trace, n


def map_base(trace):
    """Robot pose in the map from the broadcast transforms: x, y, yaw (unwrapped)."""
    c, s = np.cos(trace['mo_yaw']), np.sin(trace['mo_yaw'])
    x = trace['mo_x'] + c * trace['ob_x'] - s * trace['ob_y']
    y = trace['mo_y'] + s * trace['ob_x'] + c * trace['ob_y']
    return x, y, np.unwrap(trace['mo_yaw'] + trace['ob_yaw'])


def sel(trace, a, b):
    m = (trace['t'] >= a) & (trace['t'] <= b)
    return m
