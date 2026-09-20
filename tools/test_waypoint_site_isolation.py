#!/usr/bin/env python3
"""Waypoints belong to the map they came from (2026-08-30).

A map with no waypoints must show NO waypoints — never the previous map's.
Before this, `_import_served_site_bundle` kept whatever was in memory when the
newly served site had no waypoints.geojson, so switching to a fresh map showed
the old map's points (placed by the NEW datum, i.e. in the wrong real spot) and
the first save exported them into that fresh map for good.

The one case the old exception existed for still has to work: the dock 'docked'
seed placed BEFORE anything was served is owned by no site yet, so a site
without its own waypoints file adopts it.

Run:  python3 test_waypoint_site_isolation.py   (needs rospy importable — robot env)
"""
import importlib.machinery
import importlib.util
import json
import os
import shutil
import sys
import tempfile
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
NAVI_MAN = os.path.join(HERE, '..', 'nodes', 'navi_man')


def load(name, path):
    spec = importlib.util.spec_from_loader(
        name, importlib.machinery.SourceFileLoader(name, path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


class FakePoint(object):
    def __init__(self, name):
        self.name = name


class FakeMapData(object):
    def __init__(self, points=(), paths=()):
        self.points = list(points)
        self.paths = list(paths)
        self.utm_x = self.utm_y = self.utm_z = 0.0
        self.utm_orientation_x = self.utm_orientation_y = 0.0
        self.utm_orientation_z = 0.0
        self.utm_orientation_w = 1.0


class FakeNode(object):
    """Just enough of Node for the two methods under test."""

    def __init__(self, mod, served=None, points_site=None, points=(), paths=()):
        self._mod = mod
        self._served_site = served
        self._points_site = points_site
        self._legacy_maps = False
        self._map_data_lock = threading.RLock()
        self.running_map_data = FakeMapData(points, paths)
        self.republished = 0
        self.imported = 0
        self._bundle_site_for = None
        self._georef_vals_for = None
        self._georef_vals_cache = None
        self._nomap_coords_warned = False

    def _republish_map_lists(self):
        self.republished += 1

    def _import_served_site_bundle(self):
        self.imported += 1

    def _build_map_point_from_spec(self, spec):
        return FakePoint(spec.get('name', '?'))

    def _build_map_path_from_spec(self, spec):
        return FakePoint(spec.get('name', '?'))

    def _bundle_datum_guard(self, site, datum):
        pass


class Msg(object):
    def __init__(self, site):
        self.data = json.dumps({'serving': {'site': site} if site else None})


def names(node):
    return [p.name for p in node.running_map_data.points]


def status(mod, node, site):
    mod.Node.callback_mapping_status(node, Msg(site))


def check(label, got, want):
    if got != want:
        raise AssertionError('%s: got %r, want %r' % (label, got, want))
    print('  ok  %s' % label)


def test_switch_drops_the_previous_maps_points(mod):
    print('switching maps drops the previous map\'s points')
    n = FakeNode(mod, served='Nmap', points_site='Nmap',
                 points=[FakePoint('DOCK'), FakePoint('UNDOCK')],
                 paths=[FakePoint('cesta')])
    status(mod, n, 'zahradaDneska')
    check('points cleared', names(n), [])
    check('paths cleared', n.running_map_data.paths, [])
    check('ownership released', n._points_site, None)
    check('UI told at once', n.republished, 1)
    check('import still runs', n.imported, 1)


def test_same_site_keeps_its_points(mod):
    print('a status repeat for the same site changes nothing')
    n = FakeNode(mod, served='Nmap', points_site='Nmap', points=[FakePoint('DOCK')])
    status(mod, n, 'Nmap')
    check('points kept', names(n), ['DOCK'])
    check('no republish', n.republished, 0)
    check('no import', n.imported, 0)


def test_unowned_seed_survives_first_serving(mod):
    print('a dock seed placed before anything was served is not dropped')
    n = FakeNode(mod, served=None, points_site=None, points=[FakePoint('docked')])
    status(mod, n, 'zahradaDneska')
    check('seed kept for the import to own', names(n), ['docked'])
    check('import runs', n.imported, 1)


def test_unserving_clears_everything(mod):
    print('un-serving a site clears points and datum')
    n = FakeNode(mod, served='Nmap', points_site='Nmap', points=[FakePoint('DOCK')])
    status(mod, n, None)
    check('points cleared', names(n), [])
    check('ownership released', n._points_site, None)
    check('datum zeroed', n.running_map_data.utm_x, 0)


def _site(root, name, datum=True, waypoints=None):
    d = os.path.join(root, name)
    os.makedirs(d)
    if datum:
        with open(os.path.join(d, 'datum.yaml'), 'w') as f:
            f.write('utm_e: 500000.0\nutm_n: 5500000.0\nutm_zone: 33\n'
                    'yaw_rad: 0.0\nalt: 300.0\n')
    if waypoints is not None:
        fc = {'type': 'FeatureCollection', 'features': [
            {'type': 'Feature', 'properties': {'name': w, 'yaw_rad': 0.0},
             'geometry': {'type': 'Point', 'coordinates': [500001.0, 5500001.0]}}
            for w in waypoints]}
        with open(os.path.join(d, 'waypoints.geojson'), 'w') as f:
            json.dump(fc, f)
    return d


def test_import_from_a_site_without_a_waypoint_file(mod):
    """The live half: a served site with no waypoints.geojson yields no points."""
    print('importing a site that has no waypoints.geojson')
    root = tempfile.mkdtemp(prefix='wpsite_test_')
    old_root, old_pkl = mod._bundle.SITES_ROOT, mod.MAPDATA_PKL
    # The warm cache must go to the temp dir, NOT to the robot's live
    # ~/.ros/mapdata.pkl: opening that 'wb' truncates it before a single byte is
    # written, so stubbing pickle.dump is not enough — it emptied the live file
    # once (2026-08-30). Redirect the path itself.
    try:
        mod._bundle.SITES_ROOT = root
        mod.MAPDATA_PKL = os.path.join(root, 'mapdata.pkl')
        _site(root, 'Plna', waypoints=['docked', 'DOCK'])
        _site(root, 'Prazdna')                       # datum only, no waypoints

        n = FakeNode(mod, served='Prazdna', points_site='Plna',
                     points=[FakePoint('DOCK'), FakePoint('UNDOCK')])
        mod.Node._import_served_site_bundle(n)
        check('stale points dropped', names(n), [])
        check('owned by the served site', n._points_site, 'Prazdna')

        n = FakeNode(mod, served='Prazdna', points_site=None,
                     points=[FakePoint('docked')])
        mod.Node._import_served_site_bundle(n)
        check('unowned seed adopted', names(n), ['docked'])
        check('owned by the served site', n._points_site, 'Prazdna')

        n = FakeNode(mod, served='Plna', points_site=None,
                     points=[FakePoint('docked')])
        mod.Node._import_served_site_bundle(n)
        check('the site file wins', sorted(names(n)), ['DOCK', 'docked'])
    finally:
        mod._bundle.SITES_ROOT = old_root
        mod.MAPDATA_PKL = old_pkl
        shutil.rmtree(root, ignore_errors=True)


def test_boot_never_inherits_the_warm_cache(mod):
    """Boot with a warm cache (~/.ros/mapdata.pkl) that still holds site A's
    points while the first served site is a file-less site B.

    Site-native boot never READS the cache (running_map_data starts empty; the
    only reader, load_running_map_data, is reachable from the legacy load
    callbacks alone, and those first overwrite the cache with the chosen map's
    own pickle). So B must come up empty and the cache must be rewritten as B's.
    Guards against anyone wiring a cache read into the site-native boot without
    an ownership tag."""
    print('boot: a warm cache from another site never reaches a file-less site')
    import pickle
    root = tempfile.mkdtemp(prefix='wpsite_test_')
    old_root, old_pkl = mod._bundle.SITES_ROOT, mod.MAPDATA_PKL
    try:
        mod._bundle.SITES_ROOT = root
        mod.MAPDATA_PKL = os.path.join(root, 'mapdata.pkl')
        _site(root, 'Prazdna')                       # datum only, no waypoints
        with open(mod.MAPDATA_PKL, 'wb') as f:       # cache left by site 'Plna'
            pickle.dump(FakeMapData(points=[FakePoint('DOCK'), FakePoint('UNDOCK')]), f)

        # Boot state: nothing in memory, owned by nobody, B served first.
        n = FakeNode(mod, served=None, points_site=None)
        n._import_served_site_bundle = lambda: mod.Node._import_served_site_bundle(n)
        status(mod, n, 'Prazdna')
        check('file-less site comes up empty', names(n), [])
        check('owned by the served site', n._points_site, 'Prazdna')
        with open(mod.MAPDATA_PKL, 'rb') as f:
            cached = pickle.load(f)
        check('cache rewritten as the served site\'s', [p.name for p in cached.points], [])

        # The cache has exactly one reader, and site-native boot is not it.
        src = open(NAVI_MAN).read()
        check('single cache reader', src.count("open(MAPDATA_PKL, 'rb')"), 1)
        boot = src[src.index('def _site_native_boot'):src.index('def callback_mapping_status')]
        check('site-native boot does not load the cache',
              'load_running_map_data' in boot, False)
    finally:
        mod._bundle.SITES_ROOT = old_root
        mod.MAPDATA_PKL = old_pkl
        shutil.rmtree(root, ignore_errors=True)


def main():
    mod = load('navi_man_node', NAVI_MAN)
    if not getattr(mod, '_BUNDLE_OK', False):
        print('SKIP: vitulus_mapping bundle libs unavailable')
        return 0
    for t in (test_switch_drops_the_previous_maps_points,
              test_same_site_keeps_its_points,
              test_unowned_seed_survives_first_serving,
              test_unserving_clears_everything,
              test_import_from_a_site_without_a_waypoint_file,
              test_boot_never_inherits_the_warm_cache):
        t(mod)
    print('\nALL OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
