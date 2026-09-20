#!/usr/bin/env python3
"""navi_man / navi_transform source-tree paths and the legacy map list filter.

  * no '/home/vitulus/catkin_ws' literal is used for launch files / config /
    docker volume any more (only as the documented fallback constant)
  * on the standard checkout the resolved paths equal the historical ones
  * rtabmap_docker_dir(): ~rtabmap_docker_dir override, workspace sibling,
    historical fallback
  * is_map_db_filename(): only "<name>***env*<INDOOR|OUTDOOR>.db" is a map

Run:  python3 test_navi_man_paths.py   (needs rospy importable — robot env)
"""
import importlib.machinery
import importlib.util
import os
import sys

HERE = os.path.dirname(os.path.abspath(__file__))
NAVI_MAN = os.path.join(HERE, '..', 'nodes', 'navi_man')
NAVI_TF = os.path.join(HERE, '..', 'nodes', 'navi_transform')
LEGACY = '/home/vitulus/catkin_ws/src'


def load(name, path):
    spec = importlib.util.spec_from_loader(
        name, importlib.machinery.SourceFileLoader(name, path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def check(label, got, want):
    if got != want:
        raise AssertionError('%s: got %r, want %r' % (label, got, want))
    print('  ok  %s' % label)


def main():
    mod = load('navi_man_node', NAVI_MAN)

    print('legacy map list filter')
    for fname, want in (('GARDEN_2026***env*OUTDOOR.db', True),
                        ('BACKUP_LAST***env*OUTDOOR.db', True),
                        ('dum***env*INDOOR.db', True),
                        ('BACKUP_LAST.db', False),          # P4: the old edit-map backup
                        ('***env*OUTDOOR.db', False),
                        ('x***env*GARAGE.db', False),
                        ('a***env*b***env*OUTDOOR.db', False),
                        ('GARDEN***env*OUTDOOR.db.bak', False),
                        ('notes.txt', False)):
        check(fname, mod.is_map_db_filename(fname), want)

    print('source-tree paths')
    for path, node in ((NAVI_MAN, 'navi_man'), (NAVI_TF, 'navi_transform')):
        code = [l for l in open(path).read().splitlines()
                if '/home/vitulus/catkin_ws' in l and not l.lstrip().startswith('#')]
        check('%s: only the fallback constant names the legacy workspace' % node,
              len(code), 1)
    if os.path.isdir(os.path.join(LEGACY, 'vitulus', 'vitulus_navi')) and \
            mod.NAVI_PKG_DIR.startswith(LEGACY):
        check('standard checkout: package dir unchanged',
              mod.NAVI_PKG_DIR, os.path.join(LEGACY, 'vitulus', 'vitulus_navi'))
    for f in ('launch/vitulus_navi_indoor.launch', 'launch/vitulus_navi_outdoor.launch',
              'launch/vitulus_ekf_outdoor.launch', 'launch/vitulus_navsat.launch',
              'config/navi_manager.yaml'):
        check('exists: %s' % f, os.path.exists(os.path.join(mod.NAVI_PKG_DIR, f)), True)

    print('rtabmap_docker_dir resolution')
    real_get = mod.rospy.get_param
    try:
        mod.rospy.get_param = lambda name, default=None: '/srv/rtabmap_docker'
        check('~rtabmap_docker_dir override wins', mod.rtabmap_docker_dir(), '/srv/rtabmap_docker')
        mod.rospy.get_param = lambda name, default=None: default
        if mod.NAVI_PKG_DIR == os.path.join(LEGACY, 'vitulus', 'vitulus_navi'):
            check('standard checkout: docker volume unchanged',
                  mod.rtabmap_docker_dir(), os.path.join(LEGACY, 'rtabmap_docker'))
        old = mod.NAVI_PKG_DIR
        mod.NAVI_PKG_DIR = '/nonexistent_ws/src/vitulus/vitulus_navi'
        check('missing sibling -> historical path', mod.rtabmap_docker_dir(),
              os.path.join(LEGACY, 'rtabmap_docker'))
        mod.NAVI_PKG_DIR = old
    finally:
        mod.rospy.get_param = real_get

    print('\nALL OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
