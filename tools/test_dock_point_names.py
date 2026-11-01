#!/usr/bin/env python3
"""DOCK, UNDOCK, DOCKED uppercase everywhere (owner 2026-11-01), on navi_man's side.

An old site, an old pickle, a node not restarted yet: all must end with exactly
one uppercase point and never lose it. Nothing touches ROS or ~/.vitulus.

Run:  python3 test_dock_point_names.py   (needs rospy importable — robot env)
"""
import importlib.machinery
import importlib.util
import json
import math
import os
import pickle
import shutil
import sys
import tempfile
import threading
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _ros_env  # noqa: E402
_ros_env.ensure()
# Sibling package of this tree: a worktree next to the one the robot runs.
for candidate in (os.path.join(HERE, '..', '..', '..', '..', '..', 'vitulus_mapping', '.claude',
                               'worktrees', 'map-editor', 'src'),
                  os.path.join(HERE, '..', '..', 'vitulus_mapping', 'src')):
    if os.path.isdir(candidate):
        sys.path.insert(0, candidate)
        break

# `navi_man` resolves through devel to the checkout the robot runs, and importing
# it puts that src on sys.path; the test is about this tree.
import vitulus_navi.srv  # noqa: E402,F401
_pkg = types.ModuleType('navi_man')
_pkg.__path__ = [os.path.join(HERE, '..', 'src', 'navi_man')]
sys.modules['navi_man'] = _pkg

from geometry_msgs.msg import Point, Pose, Quaternion  # noqa: E402
from std_msgs.msg import String  # noqa: E402

FAILS = []


def check(name, got, want):
    ok = got == want
    print('  %-58s %s %s' % (name, 'OK' if ok else 'FAIL', '' if ok else '%r != %r' % (got, want)))
    if not ok:
        FAILS.append(name)


def load(name, path):
    spec = importlib.util.spec_from_loader(
        name, importlib.machinery.SourceFileLoader(name, path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class Pub(object):
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(msg)


def pose(x=0.0, y=0.0, yaw=0.0):
    return Pose(Point(x, y, 0.0), Quaternion(0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)))


def node(mod, data):
    n = types.SimpleNamespace(
        running_map_data=data, status_msg_pub=Pub(), nex_log_info_pub=Pub(),
        _republish_map_lists=lambda: None, saved=[])
    n.save_running_map_data = lambda: n.saved.append([p.name for p in data.points])
    return n


def names(data):
    return [p.name for p in data.points]


def test_mapdata(mod):
    print('MapData')
    data = mod.MapData('m')
    data.add_map_point(mod.MapPoint('DOCKED', pose(1.0), 'map'))
    data.add_map_point(mod.MapPoint('DOCK', pose(2.0), 'map'))
    check('old client asks docked', data.get_map_point('docked').pose.position.x, 1.0)
    check('any case of dock', data.get_map_point('dock').pose.position.x, 2.0)
    check('ordinary names stay exact', data.get_map_point('test'), False)

    old = mod.MapData('m')
    old.points = [mod.MapPoint('docked', pose(1.0), 'map'), mod.MapPoint('TEST', pose(), 'map')]
    blob = pickle.dumps(old, pickle.HIGHEST_PROTOCOL)
    back = pickle.loads(blob)
    check('old pickle: renamed', back.normalize_dock_names(), True)
    check('old pickle: names', names(back), ['DOCKED', 'TEST'])
    check('old pickle: marker too', back.points[0].marker_message.ns, 'DOCKED')
    check('old pickle: pose kept', back.points[0].pose.position.x, 1.0)
    check('idempotent', back.normalize_dock_names(), False)

    both = mod.MapData('m')
    both.points = [mod.MapPoint('docked', pose(1.0), 'map'), mod.MapPoint('DOCKED', pose(2.0), 'map')]
    both.normalize_dock_names()
    check('two become one', [(p.name, p.pose.position.x) for p in both.points], [('DOCKED', 2.0)])


def test_publishers_not_restarted(mod):
    print('a publisher still saying docked')
    data = mod.MapData('m')
    data.add_map_point(mod.MapPoint('DOCKED', pose(1.0), 'map'))
    n = node(mod, data)
    mod.Node.callback_save_waypoint_at(n, String(json.dumps(
        {'name': 'docked', 'x': 5.0, 'y': 6.0, 'yaw': 0.0})))
    check('save_waypoint_at moves DOCKED', [(p.name, p.pose.position.x) for p in data.points],
          [('DOCKED', 5.0)])
    n.getCurrentPose = lambda: (pose(7.0), 'map')
    mod.Node.callback_save_waypoint(n, String('docked'))
    check('save_waypoint moves DOCKED', [(p.name, p.pose.position.x) for p in data.points],
          [('DOCKED', 7.0)])


def test_protected(mod):
    print('remove_point')
    data = mod.MapData('m')
    for name in ('DOCK', 'UNDOCK', 'DOCKED', 'TEST'):
        data.add_map_point(mod.MapPoint(name, pose(), 'map'))
    n = node(mod, data)
    for name in ('DOCK', 'UNDOCK', 'DOCKED', 'docked', 'undock'):
        mod.Node.callback_remove_point(n, String(name))
    check('dock points stay', names(data), ['DOCK', 'UNDOCK', 'DOCKED', 'TEST'])
    check('nothing saved', n.saved, [])
    mod.Node.callback_remove_point(n, String('TEST'))
    check('others go', names(data), ['DOCK', 'UNDOCK', 'DOCKED'])


def site_node(mod, site, data):
    n = node(mod, data)
    n._served_site = site
    n._points_site = None
    n._map_data_lock = threading.Lock()
    n._build_map_point_from_spec = lambda s: mod.MapPoint(s['name'], pose(s['x'], s['y'], s['yaw']), 'map')
    n._build_map_path_from_spec = lambda s: s
    n._bundle_site_for_active_map = lambda: site
    n._republish_map_lists = lambda: None
    n._legacy_maps = False
    return n


def test_import_migrates_the_file(mod):
    print('importing an old site')
    root = tempfile.mkdtemp(prefix='dock_names_test_')
    old_root, old_pkl = mod._bundle.SITES_ROOT, mod.MAPDATA_PKL
    try:
        mod._bundle.SITES_ROOT = root
        mod.MAPDATA_PKL = os.path.join(root, 'mapdata.pkl')
        d = os.path.join(root, 'Old')
        os.makedirs(d)
        with open(os.path.join(d, 'datum.yaml'), 'w') as f:
            f.write('utm_e: 493000.0\nutm_n: 5540000.0\nyaw_rad: 0.0\nalt: 340.0\nutm_zone: 33\n')
        fc = {'type': 'FeatureCollection', 'features': [
            {'type': 'Feature', 'geometry': {'type': 'Point', 'coordinates': [493001.0, 5540002.0]},
             'properties': {'name': name, 'yaw_rad': 0.0}} for name in ('docked', 'DOCK', 'UNDOCK')]}
        with open(os.path.join(d, 'waypoints.geojson'), 'w') as f:
            json.dump(fc, f)
        data = mod.MapData('SITE')
        n = site_node(mod, 'Old', data)
        for _ in range(2):
            mod.Node._import_served_site_bundle(n)
            check('one DOCKED in memory', sorted(names(data)), ['DOCK', 'DOCKED', 'UNDOCK'])
            with open(os.path.join(d, 'waypoints.geojson')) as f:
                on_disk = [x['properties']['name'] for x in json.load(f)['features']]
            check('file rewritten', sorted(on_disk), ['DOCK', 'DOCKED', 'UNDOCK'])
    finally:
        mod._bundle.SITES_ROOT = old_root
        mod.MAPDATA_PKL = old_pkl
        shutil.rmtree(root, ignore_errors=True)


def test_virgin_site_chain(mod):
    print('a virgin site: the three saves after the stop in the dock')
    root = tempfile.mkdtemp(prefix='dock_names_test_')
    old_root, old_pkl = mod._bundle.SITES_ROOT, mod.MAPDATA_PKL
    try:
        mod._bundle.SITES_ROOT = root
        mod.MAPDATA_PKL = os.path.join(root, 'mapdata.pkl')
        d = os.path.join(root, 'New')
        os.makedirs(d)
        with open(os.path.join(d, 'datum.yaml'), 'w') as f:
            f.write('utm_e: 493000.0\nutm_n: 5540000.0\nyaw_rad: 0.0\nalt: 340.0\nutm_zone: 33\n')
        data = mod.MapData('SITE')
        n = site_node(mod, 'New', data)
        mod.Node._import_served_site_bundle(n)
        check('no points, no manifest needed', names(data), [])
        n.save_running_map_data = lambda: mod.Node._export_bundle(n)
        n._legacy_maps = False
        for spec in ({'name': 'DOCKED', 'x': 1.0, 'y': 2.0, 'yaw': 0.1},
                     {'name': 'UNDOCK', 'x': 2.0, 'y': 2.1, 'yaw': 0.1},
                     {'name': 'DOCK', 'x': 2.0, 'y': 2.1, 'yaw': 0.1}):
            mod.Node.callback_save_waypoint_at(n, String(json.dumps(spec)))
        check('all three on disk', sorted(w['name'] for w in mod._bundle.load_waypoints('New')),
              ['DOCK', 'DOCKED', 'UNDOCK'])
    finally:
        mod._bundle.SITES_ROOT = old_root
        mod.MAPDATA_PKL = old_pkl
        shutil.rmtree(root, ignore_errors=True)


def test_readers():
    print('readers of the file')
    sitebundle = load('sitebundle_test', os.path.join(HERE, '..', 'src', 'vitulus_navi',
                                                      'geofence', 'sitebundle.py'))
    root = tempfile.mkdtemp(prefix='dock_names_test_')
    old = sitebundle.BUNDLE_ROOT
    try:
        sitebundle.BUNDLE_ROOT = root
        os.makedirs(os.path.join(root, 'S'))
        with open(os.path.join(root, 'S', 'waypoints.geojson'), 'w') as f:
            json.dump({'features': [
                {'geometry': {'type': 'Point', 'coordinates': [1.0, 2.0]}, 'properties': {'name': n}}
                for n in ('docked', 'TEST')]}, f)
        check('sitebundle answers uppercase', sorted(sitebundle.load_waypoints('S')),
              ['DOCKED', 'TEST'])
    finally:
        sitebundle.BUNDLE_ROOT = old
        shutil.rmtree(root, ignore_errors=True)


def test_transform_fallback():
    print('navi_transform against a navi_man not restarted')
    nt = load('navi_transform_test', os.path.join(HERE, '..', 'nodes', 'navi_transform'))
    asked = []

    def srv(name):
        asked.append(name)
        return types.SimpleNamespace(result=name == 'docked')
    check('asks DOCKED, then docked', nt._dock_point_pose(srv, 'DOCKED').result, True)
    check('order', asked, ['DOCKED', 'docked'])
    asked[:] = []
    check('other anchors do not fall back', nt._dock_point_pose(srv, 'X').result, False)
    check('order', asked, ['X'])


if __name__ == '__main__':
    os.environ['ROS_MASTER_URI'] = 'http://127.0.0.1:1'
    navi = load('navi_man_node', os.path.join(HERE, '..', 'nodes', 'navi_man'))
    test_mapdata(navi)
    test_publishers_not_restarted(navi)
    test_protected(navi)
    test_import_migrates_the_file(navi)
    test_virgin_site_chain(navi)
    test_readers()
    test_transform_fallback()
    print('FAILED: %s' % FAILS if FAILS else 'ALL OK')
    sys.exit(1 if FAILS else 0)
