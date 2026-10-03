#!/usr/bin/env python3
"""Site-native navi has ONE map: the served site (vitulus-field#42).

A program started from the dock made dock_smach publish /navi_manager/new_map
"indoor". navi_man restarted its launch in mapping mode and replaced
running_map_data with an empty one: the site's waypoints, paths and datum were
gone, the mission later failed with DOCKING_NO_DOCK_POINT.

  * callback_new_map is rejected in site-native mode (same gate as load_map);
  * the served site's waypoints are imported as soon as its datum.yaml exists
    (the import used to run on boot / serving change only, never again).

Run:  python3 test_site_native_new_map.py   (needs rospy importable — robot env)
"""
import json
import os
import shutil
import sys
import tempfile
import threading

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
from test_waypoint_site_isolation import (  # noqa: E402
    NAVI_MAN, FakeMapData, FakePoint, check, load, names, _site)


class LaunchTouched(Exception):
    pass


class Pub(object):
    def __init__(self):
        self.sent = []

    def publish(self, msg):
        self.sent.append(getattr(msg, 'data', msg))


class Str(object):
    def __init__(self, data):
        self.data = data


class FakeNode(object):
    """Just enough of Node for callback_new_map / served_site_import_tick."""

    def __init__(self, mod, legacy=False, served=None, points_site=None, points=()):
        self._legacy_maps = legacy
        self._served_site = served
        self._points_site = points_site
        self._import_retry_key = None
        self._map_data_lock = threading.RLock()
        self.running_map_data = FakeMapData(points)
        self.map_data = FakeMapData()            # the empty map a new map installs
        self.active_map = "SITE"
        self.indoor = False
        self.map_show = 'octomap'
        self.status_msg_pub = Pub()
        self.nex_log_info_pub = Pub()
        self.republished = 0
        self.imports = 0
        self._real_import = lambda: mod.Node._import_served_site_bundle(self)

    # Anything past the gate reaches the launch first ("indoor" != "INIT").
    def shutdown_navi_launch(self):
        raise LaunchTouched()

    def start_navi_launch(self, args):
        raise LaunchTouched()

    def _republish_map_lists(self):
        self.republished += 1

    def _import_served_site_bundle(self):
        self.imports += 1
        self._real_import()

    def _build_map_point_from_spec(self, spec):
        return FakePoint(spec.get('name', '?'))

    def _build_map_path_from_spec(self, spec):
        return FakePoint(spec.get('name', '?'))

    def _bundle_datum_guard(self, site, datum):
        pass


def test_new_map_rejected_in_site_native(mod):
    print('site-native: new_map is rejected, the site stays in memory')
    for name in ('indoor', 'outdoor', 'test_x', 'INIT'):
        n = FakeNode(mod, served='BACK', points_site='BACK',
                     points=[FakePoint('DOCK'), FakePoint('UNDOCK'), FakePoint('docked')])
        held = n.running_map_data
        mod.Node.callback_new_map(n, Str(name))     # LaunchTouched = gate missing
        check('%s: waypoints kept' % name, names(n), ['DOCK', 'UNDOCK', 'docked'])
        check('%s: same map data object' % name, n.running_map_data is held, True)
        check('%s: active_map stays SITE' % name, n.active_map, 'SITE')
        check('%s: stays outdoor' % name, n.indoor, False)
        check('%s: map source untouched' % name, n.map_show, 'octomap')
        check('%s: UI told' % name, n.status_msg_pub.sent,
              ['New map rejected (site-native).'])


def test_new_map_still_runs_with_legacy_maps(mod):
    print('legacy_maps=true: new_map passes the gate (flow unchanged)')
    n = FakeNode(mod, legacy=True)
    n.active_map = 'GARDEN***env*OUTDOOR'
    try:
        mod.Node.callback_new_map(n, Str('indoor'))
        passed = False
    except LaunchTouched:
        passed = True
    check('reached the launch restart', passed, True)
    check('active_map New', n.active_map, 'New')
    check('indoor', n.indoor, True)


def test_gate_matches_load_map(mod):
    print('the three legacy map entry points share the gate')
    src = open(NAVI_MAN).read()
    body = src[src.index('def callback_new_map'):src.index('def interactiveMarkerFeedback')]
    check('new_map gated before any state change',
          body.index('if not self._legacy_maps:') < body.index('self.active_map = "New"'),
          True)


def test_import_retries_when_datum_appears(mod):
    print('waypoints are imported as soon as the served site has a datum')
    root = tempfile.mkdtemp(prefix='newmap_test_')
    old_root, old_pkl = mod._bundle.SITES_ROOT, mod.MAPDATA_PKL
    try:
        mod._bundle.SITES_ROOT = root
        mod.MAPDATA_PKL = os.path.join(root, 'mapdata.pkl')   # never the live cache
        d = _site(root, 'BACK', datum=False, waypoints=['DOCK', 'UNDOCK'])

        # Served at boot without a datum: nothing imported, nothing owned.
        n = FakeNode(mod, served='BACK', points_site=None)
        n._real_import()
        check('no datum -> no points', names(n), [])
        check('not owned yet', n._points_site, None)

        tick = lambda: mod.Node.served_site_import_tick(n)
        tick()
        tick()
        check('no retry while the datum is missing', n.imports, 0)

        with open(os.path.join(d, 'datum.yaml'), 'w') as f:
            f.write('utm_e: 500000.0\nutm_n: 5500000.0\nutm_zone: 33\n'
                    'yaw_rad: 0.0\nalt: 300.0\n')
        tick()
        check('imported on the next tick', sorted(names(n)), ['DOCK', 'UNDOCK'])
        check('owned by the served site', n._points_site, 'BACK')
        check('datum in memory', n.running_map_data.utm_x, 500000.0)
        tick()
        tick()
        check('one import, then quiet', n.imports, 1)
    finally:
        mod._bundle.SITES_ROOT = old_root
        mod.MAPDATA_PKL = old_pkl
        shutil.rmtree(root, ignore_errors=True)


def test_tick_is_a_noop_when_nothing_is_missing(mod):
    print('the retry never touches a loaded site, an un-served robot or legacy maps')
    root = tempfile.mkdtemp(prefix='newmap_test_')
    old_root, old_pkl = mod._bundle.SITES_ROOT, mod.MAPDATA_PKL
    try:
        mod._bundle.SITES_ROOT = root
        mod.MAPDATA_PKL = os.path.join(root, 'mapdata.pkl')
        _site(root, 'BACK', waypoints=['DOCK'])

        n = FakeNode(mod, served='BACK', points_site='BACK', points=[FakePoint('X')])
        mod.Node.served_site_import_tick(n)
        check('loaded site: no import', (n.imports, names(n)), (0, ['X']))

        n = FakeNode(mod, served=None, points_site=None, points=[FakePoint('docked')])
        mod.Node.served_site_import_tick(n)
        check('nothing served: seed kept', (n.imports, names(n)), (0, ['docked']))

        n = FakeNode(mod, legacy=True, served='BACK', points_site=None)
        mod.Node.served_site_import_tick(n)
        check('legacy maps: no import', n.imports, 0)

        # A site that cannot be imported is tried once per datum state.
        d = _site(root, 'ROZBITA', datum=False)
        with open(os.path.join(d, 'datum.yaml'), 'w') as f:
            f.write('utm_e: [not, a, number\n')
        n = FakeNode(mod, served='ROZBITA', points_site=None)
        for _ in range(3):
            mod.Node.served_site_import_tick(n)
        check('broken datum: one attempt only', n.imports, 1)
        check('broken datum: still not owned', n._points_site, None)
    finally:
        mod._bundle.SITES_ROOT = old_root
        mod.MAPDATA_PKL = old_pkl
        shutil.rmtree(root, ignore_errors=True)


def test_slow_loop_calls_the_retry(mod):
    print('the slow loop runs the retry')
    src = open(NAVI_MAN).read()
    main = src[src.index("if __name__ == '__main__':"):]
    check('served_site_import_tick in the slow loop',
          'node.served_site_import_tick()' in main, True)


def main():
    mod = load('navi_man_node', NAVI_MAN)
    if not getattr(mod, '_BUNDLE_OK', False):
        print('SKIP: vitulus_mapping bundle libs unavailable')
        return 0
    for t in (test_new_map_rejected_in_site_native,
              test_new_map_still_runs_with_legacy_maps,
              test_gate_matches_load_map,
              test_import_retries_when_datum_appears,
              test_tick_is_a_noop_when_nothing_is_missing,
              test_slow_loop_calls_the_retry):
        t(mod)
    print('\nALL OK')
    return 0


if __name__ == '__main__':
    sys.exit(main())
