#!/usr/bin/env python3
"""Unit tests for navi_transform.georef_status (/nav_tf/georef_status,
vitulus-field#32): the UTM frame of the map (zone, band, origin, map rotation
in the grid vs the grid convergence) and the map-vs-GPS deviation (the
gps-map-probe dD) with its reasons when it cannot be measured.

Same style/runner as test_georef_rotation.py.

Run:  python3 test_georef_status.py   (needs rospy importable — robot env)
"""
import math
import os
import sys
import importlib.util
import importlib.machinery

HERE = os.path.dirname(os.path.abspath(__file__))
NAVI_TF = os.path.join(HERE, '..', 'nodes', 'navi_transform')


def load(name, path):
    spec = importlib.util.spec_from_loader(
        name, importlib.machinery.SourceFileLoader(name, path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def map_to_utm(x, y, utm_e, utm_n, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return utm_e + c * x - s * y, utm_n + s * x + c * y


def main():
    nt = load('navi_transform_node', NAVI_TF)
    fails = []

    def check(name, cond):
        print('  %-64s %s' % (name, 'OK' if cond else 'FAIL'))
        if not cond:
            fails.append(name)

    # owner site 10_2026 (~/.vitulus/mapping_v3/10_2026/datum.yaml)
    own = {'utm_e': 493073.71, 'utm_n': 5540714.687, 'yaw_rad': -0.001293,
           'utm_zone': 33, 'origin_lat': 50.01870316, 'origin_lon': 14.90331712}
    g_own = (own['utm_e'], own['utm_n'], own['yaw_rad'], 'datum')
    loc_gps = {'owner': 'gps', 'docked': False, 'gps_good': True, 'proven': True}
    now = 1000.0

    print('UTM helpers:')
    e, n, z = nt.ll_to_utm(own['origin_lat'], own['origin_lon'])
    check('origin lat/lon -> datum UTM within 2 cm (zone 33)',
          z == 33 and abs(e - own['utm_e']) < 0.02 and abs(n - own['utm_n']) < 0.02)
    la, lo = nt.utm_to_ll(own['utm_e'], own['utm_n'], 33, True)
    check('utm_to_ll inverts ll_to_utm (< 1e-7 deg)',
          abs(la - own['origin_lat']) < 1e-7 and abs(lo - own['origin_lon']) < 1e-7)
    for (lat, lon) in ((49.55, 12.1), (-33.9, 151.2), (64.1, -21.9)):
        e, n, z = nt.ll_to_utm(lat, lon)
        la, lo = nt.utm_to_ll(e, n, z, lat >= 0)
        check('round trip %.2f, %.2f' % (lat, lon), abs(la - lat) < 1e-7 and abs(lo - lon) < 1e-7)
    check('band 50.02 N = U', nt.utm_band_of(50.0187) == 'U')
    check('band -33.9 = H', nt.utm_band_of(-33.9) == 'H')

    print('frame (owner site):')
    s = nt.georef_status(own, g_own, None, None, {'owner': 'dock', 'docked': True}, now)
    check('zone 33 N, band U, from datum',
          s['utm_zone'] == 33 and s['hemisphere'] == 'N' and s['utm_band'] == 'U'
          and s['zone_src'] == 'datum')
    check('map_yaw_deg = -0.0741', abs(s['map_yaw_deg'] - math.degrees(-0.001293)) < 1e-3)
    check('convergence = datum yaw within 0.001 deg (manual ~0)',
          abs(s['convergence_deg'] - s['map_yaw_deg']) < 0.001 and abs(s['manual_yaw_deg']) < 0.001)
    check('docked + no fix -> reason docked (the dock first)', s['dev_m'] is None and s['reason'] == 'docked')
    s = nt.georef_status(own, g_own, None, None, {'owner': 'tracker', 'docked': False}, now)
    check('no fix -> dev None, reason no_fix', s['dev_m'] is None and s['reason'] == 'no_fix')

    print('frame (Mac site, no origin lat/lon -> inverse UTM):')
    mac = {'utm_e': 309148.230, 'utm_n': 5491540.863, 'yaw_rad': -0.035051}
    s = nt.georef_status(mac, (mac['utm_e'], mac['utm_n'], mac['yaw_rad'], 'datum'),
                         (49.55, 12.33, 2, now, True), None, loc_gps, now)
    check('zone from the fix (navsat) = 33', s['utm_zone'] == 33 and s['zone_src'] == 'navsat')
    check('convergence -2.01 deg, manual ~0 (< 0.02 deg)',
          abs(s['convergence_deg'] + 2.01) < 0.02 and abs(s['manual_yaw_deg']) < 0.02)
    check('no antenna TF -> reason no_tf', s['dev_m'] is None and s['reason'] == 'no_tf')

    print('deviation map vs GPS (dD):')
    ant = (3.0, 4.0)
    for off, yaw in ((0.0, own['yaw_rad']), (0.10, own['yaw_rad']), (0.30, -0.6)):
        g = (own['utm_e'], own['utm_n'], yaw, 'datum')
        e, n = map_to_utm(ant[0] + off, ant[1], own['utm_e'], own['utm_n'], yaw)
        la, lo = nt.utm_to_ll(e, n, 33, True)
        s = nt.georef_status(own, g, (la, lo, 2, now - 0.2, True), ant, loc_gps, now)
        check('GPS %.2f m east (map) of the antenna -> dev %.3f, dx %.3f'
              % (off, -1 if s['dev_m'] is None else s['dev_m'], s['dev_dx'] or 0.0),
              s['dev_m'] is not None and abs(s['dev_m'] - off) < 0.002
              and abs(s['dev_dx'] - off) < 0.002 and abs(s['dev_dy']) < 0.002
              and s['reason'] is None)
    e, n = map_to_utm(3.2, 4.0, own['utm_e'], own['utm_n'], own['yaw_rad'])
    la, lo = nt.utm_to_ll(e, n, 33, True)
    fix = (la, lo, 2, now - 0.2, True)
    s = nt.georef_status(own, g_own, fix, ant, {'owner': 'dock', 'docked': True,
                                                 'gps_good': False, 'proven': False}, now)
    check('docked: dev None, raw 0.20 kept, reason docked',
          s['dev_m'] is None and abs(s['dev_raw_m'] - 0.2) < 0.002 and s['reason'] == 'docked')
    s = nt.georef_status(own, g_own, fix, ant, {'owner': 'tracker', 'docked': False,
                                                 'gps_good': False, 'proven': True}, now)
    check('GPS not holding -> reason gps_not_holding', s['reason'] == 'gps_not_holding')
    s = nt.georef_status(own, g_own, (la, lo, 0, now - 0.2, False), ant, loc_gps, now)
    check('no RTK fixed -> reason no_rtk, no raw number',
          s['dev_m'] is None and s['reason'] == 'no_rtk' and s['dev_raw_m'] is None)
    s = nt.georef_status(own, g_own, (la, lo, 2, now - 10.0, True), ant, loc_gps, now)
    check('stale fix (10 s) -> reason no_fix', s['dev_m'] is None and s['reason'] == 'no_fix')
    s = nt.georef_status(None, None, fix, ant, loc_gps, now)
    check('no georef -> reason no_datum', s['reason'] == 'no_datum' and s['utm_zone'] is None)
    import json
    check('payload is JSON-serialisable', bool(json.dumps(s)))

    if fails:
        print('\nFAILED: %d' % len(fails))
        for n in fails:
            print('  - ' + n)
        sys.exit(1)
    print('\nall OK')


if __name__ == '__main__':
    main()
