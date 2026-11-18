#!/usr/bin/env python3
"""T36 nav_ekf_vz (~nav_ekf_vz): the nav EKF (ekf_wheel_nav_odometry) can also
fuse the vertical speed of the wheel odometry (twist.linear.z, always 0 in the
body frame). Nothing else measures vz, so its covariance and that of z grow
without bound and only the 10 Hz set_pose of the position anchor resets them.
A config file with that one difference (config/ekf/ekf_nav_vz.yaml) is laid
over the base one by launch/vitulus_ekf_outdoor.launch when navi_transform
starts the EKFs with ~nav_ekf_vz.

  - ekf_launch_env (pure) and roslaunch_ekf: parameter off = the variable is
    absent and the launch gets what it got before; on = VITULUS_NAV_EKF_VZ,
  - the launch file resolved by roslaunch itself (no master, nothing started)
    for all four combinations with J2 (~nav_ekf_gyro): both off = exactly the
    base config; vz alone = only odom0_config of the nav EKF differs; with the
    gyro = that and imu0_config; the wheel EKF never changes,
  - the wheel odometry does carry linear.z (a recorded message).

Run:  python3 test_t36_vz_nav_ekf_vz.py   (needs rospy / roslaunch - robot env)
"""
import importlib.machinery
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402

import yaml  # noqa: E402

PKG = os.path.join(HERE, '..')
LAUNCH = os.path.join(PKG, 'launch', 'vitulus_ekf_outdoor.launch')
BASE = os.path.join(PKG, 'config', 'ekf', 'ekf_base_outdoor.yaml')
OVER = os.path.join(PKG, 'config', 'ekf', 'ekf_nav_vz.yaml')
VZ = 8             # robot_localization: x y z roll pitch yaw vx vy VZ vroll vpitch vyaw ax ay az
ENV = 'VITULUS_NAV_EKF_VZ'
# J2 (~nav_ekf_gyro) is another switch: its combinations are checked only while it is there
J2 = os.path.exists(os.path.join(PKG, 'config', 'ekf', 'ekf_nav_gyro.yaml'))


def resolved(files, vz):
    """Parameters roslaunch would put on the server ($(find vitulus_navi) =
    this checkout, the environment as navi_transform leaves it); no master."""
    import roslaunch.config
    import roslaunch.substitution_args as sa
    real = sa._find

    def find(resolved_, a, args, context):
        if args and args[0] == 'vitulus_navi':
            return resolved_.replace('$(%s)' % a, os.path.abspath(PKG))
        return real(resolved_, a, args, context)
    sa._find = find
    old = os.environ.pop(ENV, None)
    if vz:
        os.environ[ENV] = 'true'
    try:
        cfg = roslaunch.config.load_config_default(files, None, verbose=False)
    finally:
        sa._find = real
        os.environ.pop(ENV, None)
        if old is not None:
            os.environ[ENV] = old
    return {k: v.value for k, v in cfg.params.items() if '/ekf_' in k}


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-72s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    nt = R.load('navi_transform_t36_vz')

    print('ekf_launch_env, roslaunch_ekf:')
    env = {'OTHER': '1'}
    check('on: the variable is set to true', nt.ekf_launch_env(True, env) is env
          and env == {'OTHER': '1', ENV: 'true'} and nt.NAV_EKF_VZ_ENV == ENV)
    check('off: the variable is removed, nothing else touched',
          nt.ekf_launch_env(False, env) == {'OTHER': '1'} and nt.ekf_launch_env(False, {}) == {})
    seen = []

    class Parent(object):
        def __init__(self, uuid, files, *a, **k):
            seen.append((files, os.environ.get(ENV)))

        def start(self):
            pass
    nt.roslaunch.parent.ROSLaunchParent = Parent
    nt.roslaunch.rlutil.get_or_generate_uuid = lambda *a, **k: 'uuid'
    nt.roslaunch.configure_logging = lambda *a, **k: None
    # _t36_replay.load replaces roslaunch_ekf by a no-op: the real one from the source again
    spec = importlib.util.spec_from_loader(
        'navi_transform_t36_vz_raw', importlib.machinery.SourceFileLoader(
            'navi_transform_t36_vz_raw', R.NAVI_TF))
    raw = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(raw)
    raw.roslaunch.configure_logging = lambda *a, **k: None
    old = os.environ.pop(ENV, None)
    infos = []
    raw.rospy.loginfo = lambda *a, **k: infos.append(a[0] % a[1:] if len(a) > 1 else a[0])
    said = []
    try:
        for params in (None, {'nav_ekf_vz': True}, None) + (({'nav_ekf_vz': True, 'nav_ekf_gyro': True},) if J2 else ()):
            n = R.make_node(nt, params)
            del infos[:]
            raw.NavTfNode.roslaunch_ekf(n)
            said.append(len([i for i in infos if '~nav_ekf_vz' in i and 'ekf_nav_vz.yaml' in i]))
    finally:
        os.environ.pop(ENV, None)
        if old is not None:
            os.environ[ENV] = old
    path = os.path.join(nt.NAVI_PKG_DIR, 'launch', 'vitulus_ekf_outdoor.launch')
    check('node, off (default): the launch gets the bare path, the variable is absent',
          seen[0] == ([path], None), str(seen[0]))
    check('node, on: the same launch call, the variable is true', seen[1] == ([path], 'true'))
    check('node, off again after on: the variable is gone', seen[2] == ([path], None))
    check('one INFO line when the parameter is on, none when it is off',
          said[:3] == [0, 1, 0] and all(x == 1 for x in said[3:]), str(said))
    if J2:
        check('node, with J2 as well: its argument and the variable',
              seen[3] == ([(path, ['nav_ekf_gyro:=true'])], 'true'), str(seen[3]))

    print('the two config files:')
    base = yaml.safe_load(open(BASE))
    over = yaml.safe_load(open(OVER))
    b = base['ekf_wheel_nav_odometry']['odom0_config']
    o = over['ekf_wheel_nav_odometry']['odom0_config']
    check('overlay holds only odom0_config of the nav EKF',
          list(over) == ['ekf_wheel_nav_odometry'] and list(over['ekf_wheel_nav_odometry']) == ['odom0_config'])
    check('... equal to the base but for vz (index 8): false -> true',
          len(o) == 15 and [i for i in range(15) if o[i] != b[i]] == [VZ] and o[VZ] is True
          and b[VZ] is False)
    check('base: odom0 is the wheel odometry, absolute twist (not differential)',
          base['ekf_wheel_nav_odometry']['odom0'] == '/mobile_base_controller/odom'
          and base['ekf_wheel_nav_odometry']['odom0_differential'] is False)

    print('the launch file, resolved by roslaunch, all four combinations with J2:')
    flat = {}

    def walk(prefix, d):
        for k, v in d.items():
            if isinstance(v, dict):
                walk(prefix + '/' + k, v)
            else:
                flat[prefix + '/' + k] = v
    walk('', base)
    off = resolved([LAUNCH], False)
    vz = resolved([LAUNCH], True)
    NAV_O, NAV_I = '/ekf_wheel_nav_odometry/odom0_config', '/ekf_wheel_nav_odometry/imu0_config'

    def diff(a):
        return sorted(k for k in set(a) | set(off) if a.get(k) != off.get(k))
    check('both off: %d parameters, exactly the base config' % len(off), off == flat,
          str(sorted(set(off) ^ set(flat))[:4]))
    check('vz alone: the only parameter that differs is odom0_config of the nav EKF',
          diff(vz) == [NAV_O] and vz[NAV_O] == o, str(diff(vz)))
    combos = [vz]
    if J2:
        gy = resolved([(LAUNCH, ['nav_ekf_gyro:=true'])], False)
        both = resolved([(LAUNCH, ['nav_ekf_gyro:=true'])], True)
        combos += [gy, both]
        check('gyro alone (J2): only imu0_config of the nav EKF, as before',
              diff(gy) == [NAV_I] and gy[NAV_O] == b)
        check('both on: the two differences, each as alone',
              diff(both) == sorted([NAV_O, NAV_I]) and both[NAV_O] == vz[NAV_O]
              and both[NAV_I] == gy[NAV_I], str(diff(both)))
    else:
        print('    (J2 ~nav_ekf_gyro is not in this tree: its combinations are not checked)')
    check('the wheel EKF (ekf_wheel_odometry) is the same in every combination',
          all({k: v for k, v in c.items() if k.startswith('/ekf_wheel_odometry/')}
              == {k: v for k, v in off.items() if k.startswith('/ekf_wheel_odometry/')}
              for c in combos))
    check('an explicit launch argument nav_ekf_vz:=true does the same as the variable',
          resolved([(LAUNCH, ['nav_ekf_vz:=true'])], False) == vz)

    print('the wheel odometry:')
    wo = R.load_data()['wo']
    check('recorded /mobile_base_controller/odom is there (%d messages); its linear.z is 0 '
          '(read from the recordings 2026-11-16: variance 0.001)' % len(wo), len(wo) > 1000)

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
