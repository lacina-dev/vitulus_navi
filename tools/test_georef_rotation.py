#!/usr/bin/env python3
"""Unit tests for navi_transform.georef_map_odom_xy (vitulus-field#32):

the GPS georef map->odom translation must be the UTM-grid offset
utm_odom - utm_map expressed in MAP axes (rotated by -datum yaw). Before the
fix it stayed in grid axes, so a stack started away from the map origin at a
site with 2.01 deg grid convergence put the whole map 0.82 m off.

Same style/runner as test_dockfix_gates.py.

Run:  python3 test_georef_rotation.py   (needs rospy importable — robot env)
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
    """vitulus_mapping.geo.map_to_utm — the convention every other consumer
    of the site datum (waypoints, geofence, bundle) uses."""
    c, s = math.cos(yaw), math.sin(yaw)
    return utm_e + c * x - s * y, utm_n + s * x + c * y


def main():
    nt = load('navi_transform_node', NAVI_TF)
    f = nt.georef_map_odom_xy
    fails = []

    def check(name, cond):
        print('  %-60s %s' % (name, 'OK' if cond else 'FAIL'))
        if not cond:
            fails.append(name)

    # Mac G6 (#32): datum + the live utm->odom after a restart 21.5 m from dock
    utm_map = (309148.230, 5491540.863)
    utm_odom = (309160.536, 5491521.358)
    yaw = -0.035051                       # -2.01 deg = grid convergence there

    print('georef_map_odom_xy:')
    tx, ty = f(utm_odom, utm_map, 0.0)
    check('zero datum yaw = plain UTM difference',
          abs(tx - 12.306) < 1e-6 and abs(ty - (-19.505)) < 1e-6)

    for y in (yaw, -0.001293, 0.5, -3.0):
        tx, ty = f(utm_map, utm_map, y)
        check('start at map origin (dock) -> (0, 0), yaw %.4f' % y,
              abs(tx) < 1e-12 and abs(ty) < 1e-12)

    # round trip: the odom origin placed at t in the map must map back onto
    # utm_odom through the datum — for any yaw / offset
    for (uo, um, y) in [(utm_odom, utm_map, yaw),
                        ((493076.497, 5540708.086), (493073.710, 5540714.687),
                         -0.001293),
                        ((1000.0, 2000.0), (990.0, 2030.0), math.pi / 2),
                        ((5.0, -7.0), (-40.0, 12.0), -2.5)]:
        tx, ty = f(uo, um, y)
        e, n = map_to_utm(tx, ty, um[0], um[1], y)
        check('odom origin maps back onto utm_odom (yaw %.4f)' % y,
              abs(e - uo[0]) < 1e-6 and abs(n - uo[1]) < 1e-6)
        check('rotation keeps the length (yaw %.4f)' % y,
              abs(math.hypot(tx, ty)
                  - math.hypot(uo[0] - um[0], uo[1] - um[1])) < 1e-9)

    # the measured G6 shift: old (grid-axes) pose minus the fixed one
    tx, ty = f(utm_odom, utm_map, yaw)
    dx, dy = 12.306 - tx, -19.505 - ty
    print('    G6 fixed t=(%.3f, %.3f), old-new=(%.3f, %.3f) |%.3f| m'
          % (tx, ty, dx, dy, math.hypot(dx, dy)))
    check('G6: fixed translation (12.982, -19.062)',
          abs(tx - 12.982) < 0.002 and abs(ty - (-19.062)) < 0.002)
    check('G6: removed error matches measured (-0.689, -0.446) within 3 cm',
          abs(dx - (-0.689)) < 0.03 and abs(dy - (-0.446)) < 0.03)

    # owner site 10_2026 (yaw -0.07 deg): the change is below 1 cm at 7 m
    uo, um = (493076.497, 5540708.086), (493073.710, 5540714.687)
    tx, ty = f(uo, um, -0.001293)
    ch = math.hypot(tx - (uo[0] - um[0]), ty - (uo[1] - um[1]))
    check('owner site: change %.4f m < 0.01 m' % ch, ch < 0.01)

    if fails:
        print('\nFAILED: %d' % len(fails))
        for n in fails:
            print('  - ' + n)
        sys.exit(1)
    print('\nall OK')


if __name__ == '__main__':
    main()
