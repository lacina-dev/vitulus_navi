#!/usr/bin/env python3
"""T36 J2 (~nav_ekf_gyro): the nav EKF (ekf_wheel_nav_odometry) can also fuse
the yaw RATE of the gyro from /gnss_heading/nav_fused, the input that already
gives it roll / pitch / yaw. A second config file (config/ekf/ekf_nav_gyro.yaml)
is laid over the base one by launch/vitulus_ekf_outdoor.launch when
navi_transform starts the EKFs with nav_ekf_gyro:=true.

  - ekf_launch_files (pure) and roslaunch_ekf: parameter off = the launch
    exactly as before (the bare path), on = the path with the argument,
  - the launch file resolved by roslaunch itself (no master, nothing started):
    off = the parameters of the base config, to the last one; on = the same
    but imu0_config of the nav EKF with vyaw; the wheel EKF untouched,
  - nav_fused carries the IMU's angular velocity and its covariance unchanged.

Run:  python3 test_t36_j2_nav_ekf_gyro.py   (needs rospy / roslaunch - robot env)
"""
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

import yaml  # noqa: E402

PKG = os.path.join(HERE, '..')
LAUNCH = os.path.join(PKG, 'launch', 'vitulus_ekf_outdoor.launch')
BASE = os.path.join(PKG, 'config', 'ekf', 'ekf_base_outdoor.yaml')
GYRO = os.path.join(PKG, 'config', 'ekf', 'ekf_nav_gyro.yaml')
VYAW = 11          # robot_localization: x y z roll pitch yaw vx vy vz vroll vpitch VYAW ax ay az


def resolved(files):
    """Parameters roslaunch would put on the server for these launch files
    ($(find vitulus_navi) pointed at this checkout); no master is contacted."""
    import roslaunch.config
    import roslaunch.substitution_args as sa
    real = sa._find

    def find(resolved_, a, args, context):
        if args and args[0] == 'vitulus_navi':
            return resolved_.replace('$(%s)' % a, os.path.abspath(PKG))
        return real(resolved_, a, args, context)
    sa._find = find
    try:
        cfg = roslaunch.config.load_config_default(files, None, verbose=False)
    finally:
        sa._find = real
    return {k: v.value for k, v in cfg.params.items() if '/ekf_' in k}


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-72s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    nt = R.load('navi_transform_t36_j2')

    print('ekf_launch_files, roslaunch_ekf:')
    check('off: the bare path, as before', nt.ekf_launch_files('/x/ekf.launch') == ['/x/ekf.launch']
          and nt.ekf_launch_files('/x/ekf.launch', False) == ['/x/ekf.launch'])
    check('on: the path with nav_ekf_gyro:=true',
          nt.ekf_launch_files('/x/ekf.launch', True) == [('/x/ekf.launch', ['nav_ekf_gyro:=true'])])
    seen = []

    class Parent(object):
        def __init__(self, uuid, files, *a, **k):
            seen.append(files)

        def start(self):
            pass
    nt.roslaunch.parent.ROSLaunchParent = Parent
    nt.roslaunch.rlutil.get_or_generate_uuid = lambda *a, **k: 'uuid'
    nt.roslaunch.configure_logging = lambda *a, **k: None
    # _t36_replay.load replaces roslaunch_ekf by a no-op: take the real one
    # from the source loaded once more
    import importlib.machinery
    import importlib.util
    spec = importlib.util.spec_from_loader(
        'navi_transform_t36_j2_raw', importlib.machinery.SourceFileLoader(
            'navi_transform_t36_j2_raw', R.NAVI_TF))
    raw = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(raw)
    for params in (None, {'nav_ekf_gyro': True}):
        n = R.make_node(nt, params)
        raw.NavTfNode.roslaunch_ekf(n)
    path = os.path.join(nt.NAVI_PKG_DIR, 'launch', 'vitulus_ekf_outdoor.launch')
    check('node, parameter off (default): ROSLaunchParent gets [path]', seen[0] == [path], str(seen[0]))
    check('node, on: ROSLaunchParent gets [(path, [nav_ekf_gyro:=true])]',
          seen[1] == [(path, ['nav_ekf_gyro:=true'])], str(seen[1]))

    print('the two config files:')
    base = yaml.safe_load(open(BASE))
    gyro = yaml.safe_load(open(GYRO))
    b_imu = base['ekf_wheel_nav_odometry']['imu0_config']
    g_imu = gyro['ekf_wheel_nav_odometry']['imu0_config']
    check('overlay holds only imu0_config of the nav EKF',
          list(gyro) == ['ekf_wheel_nav_odometry'] and list(gyro['ekf_wheel_nav_odometry']) == ['imu0_config'])
    check('... equal to the base but for vyaw (index 11): false -> true',
          len(g_imu) == 15 and [i for i in range(15) if g_imu[i] != b_imu[i]] == [VYAW]
          and g_imu[VYAW] is True and b_imu[VYAW] is False)
    check('base: the input is /gnss_heading/nav_fused, absolute (not differential / relative)',
          base['ekf_wheel_nav_odometry']['imu0'] == '/gnss_heading/nav_fused'
          and base['ekf_wheel_nav_odometry']['imu0_differential'] is False
          and base['ekf_wheel_nav_odometry']['imu0_relative'] is False)

    print('the launch file, resolved by roslaunch:')
    off = resolved([LAUNCH])
    off2 = resolved([(LAUNCH, ['nav_ekf_gyro:=false'])])
    on = resolved([(LAUNCH, ['nav_ekf_gyro:=true'])])
    flat = {}

    def walk(prefix, d):
        for k, v in d.items():
            if isinstance(v, dict):
                walk(prefix + '/' + k, v)
            else:
                flat[prefix + '/' + k] = v
    walk('', base)
    check('off: %d parameters, exactly the base config' % len(off), off == flat and off2 == off,
          str(sorted(set(off) ^ set(flat))[:4]))
    diff = sorted(k for k in set(on) | set(off) if on.get(k) != off.get(k))
    check('on: the only parameter that differs is /ekf_wheel_nav_odometry/imu0_config',
          diff == ['/ekf_wheel_nav_odometry/imu0_config'], str(diff))
    check('on: imu0_config of the nav EKF with vyaw, the wheel EKF as before',
          on['/ekf_wheel_nav_odometry/imu0_config'] == g_imu
          and on['/ekf_wheel_odometry/imu0_config'] == off['/ekf_wheel_odometry/imu0_config']
          and on['/ekf_wheel_odometry/imu0'] == '/bno085/imu')

    print('what nav_fused carries:')
    R.CLOCK[0] = 4000.0
    n = R.make_node(nt, None)
    imu = R.imu_msg(4000.0, 0.4, 0.37)
    imu.angular_velocity.x, imu.angular_velocity.y = 0.01, -0.02
    imu.angular_velocity_covariance = [0.025, 0, 0, 0, 0.025, 0, 0, 0, 0.025]
    n.callback_imu(imu)
    f = n.fused_nav_pub.last
    check('angular velocity and its covariance of /bno085/imu, frame base_link',
          f.angular_velocity.z == 0.37 and f.angular_velocity.x == 0.01 and f.angular_velocity.y == -0.02
          and list(f.angular_velocity_covariance) == list(imu.angular_velocity_covariance)
          and f.header.frame_id == 'base_link')

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
