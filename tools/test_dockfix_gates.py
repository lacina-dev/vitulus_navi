#!/usr/bin/env python3
"""Unit tests for the DOCKFIX-2026-08-29 dock-localization helpers:

  navi_transform.dock_hold_map_odom   — the continuous dock hold (A2) algebra
  navi_transform.tracker_budget_limits — the drift-budget RATCHET
  gloc_server.prior_margin_verdict     — the prior-referenced staleness gate

These are the genuinely new pieces of maths behind "robot docked => pose IS the
docked waypoint" and "a changed garden degrades to REJECT, never to confidently
wrong". Same style/runner as test_anchor_helpers.py.

Run:  python3 test_dockfix_gates.py   (needs rospy importable — robot env)
"""
import math
import os
import sys
import importlib.util
import importlib.machinery

HERE = os.path.dirname(os.path.abspath(__file__))
NAVI_TF = os.path.join(HERE, '..', 'nodes', 'navi_transform')
GLOC = os.path.join(HERE, '..', 'nodes', 'gloc_server')


def load(name, path):
    spec = importlib.util.spec_from_loader(
        name, importlib.machinery.SourceFileLoader(name, path))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod


def compose(a, b):
    """a (.) b for 2D poses (x, y, yaw) — the map->base the TF tree publishes
    when map->odom == a and odom->base == b."""
    ax, ay, ath = a
    bx, by, bth = b
    c, s = math.cos(ath), math.sin(ath)
    return (ax + c * bx - s * by,
            ay + s * bx + c * by,
            math.atan2(math.sin(ath + bth), math.cos(ath + bth)))


def main():
    nt = load('navi_transform_node', NAVI_TF)
    gl = load('gloc_server_node', GLOC)
    fails = []

    def check(name, cond):
        print('  %-52s %s' % (name, 'OK' if cond else 'FAIL'))
        if not cond:
            fails.append(name)

    # ---- A2: dock hold round-trip -------------------------------------------
    print('dock_hold_map_odom (A2 continuous dock hold):')
    # the live "docked" waypoint of this robot
    anchor = (2.4830, 0.1260, -0.27437)
    for ob in [(0.0, 0.0, 0.0),
               (0.345, -0.093, 0.0),          # the measured frozen odom offset
               (-1.2, 3.4, 1.9),
               (0.5, 0.5, math.pi - 0.01),
               (0.0, 0.0, -math.pi + 0.01)]:
        tx, ty, m = nt.dock_hold_map_odom(anchor, ob)
        got = compose((tx, ty, m), ob)
        err = math.hypot(got[0] - anchor[0], got[1] - anchor[1])
        eyaw = abs(math.atan2(math.sin(got[2] - anchor[2]),
                              math.cos(got[2] - anchor[2])))
        check('base lands on the anchor for odom->base %s' % (ob,),
              err < 1e-9 and eyaw < 1e-9)
    # a pure odometry drift must be absorbed by the hold, not by the pose
    tx0, ty0, _ = nt.dock_hold_map_odom(anchor, (0.0, 0.0, 0.0))
    tx1, ty1, _ = nt.dock_hold_map_odom(anchor, (0.32, 0.0, 0.0))
    check('0.32 m odom drift moves map->odom by 0.32 m (pose stays)',
          abs(math.hypot(tx1 - tx0, ty1 - ty0) - 0.32) < 1e-9)

    # ---- drift-budget ratchet ------------------------------------------------
    print('tracker_budget_limits (drift-budget ratchet):')
    P = dict(c0_m=0.15, per_m=0.06, per_rot_m=0.003,
             yaw0_deg=4.0, yaw_per_rot=0.05)
    lim_m, lim_deg = nt.tracker_budget_limits(
        P['c0_m'], P['per_m'], P['per_rot_m'], P['yaw0_deg'],
        P['yaw_per_rot'], 0.0, 0.0, 0.0, 0.0, True)
    check('fresh anchor, no travel -> base budget 0.15 m',
          abs(lim_m - 0.15) < 1e-12 and abs(lim_deg - 4.0) < 1e-12)
    lim_m, _ = nt.tracker_budget_limits(
        P['c0_m'], P['per_m'], P['per_rot_m'], P['yaw0_deg'],
        P['yaw_per_rot'], 10.0, 0.0, 0.0, 0.0, True)
    check('10 m driven -> 0.75 m budget', abs(lim_m - 0.75) < 1e-12)
    # THE BUG: consuming the budget must shrink it (old code reset it to base)
    lim_m, _ = nt.tracker_budget_limits(
        P['c0_m'], P['per_m'], P['per_rot_m'], P['yaw0_deg'],
        P['yaw_per_rot'], 0.0, 0.0, 0.15, 0.0, True)
    check('0.15 m consumed at standstill -> 0 m left (no free refill)',
          abs(lim_m) < 1e-12)
    lim_m, _ = nt.tracker_budget_limits(
        P['c0_m'], P['per_m'], P['per_rot_m'], P['yaw0_deg'],
        P['yaw_per_rot'], 0.0, 0.0, 5.0, 0.0, True)
    check('over-consumed budget clamps at 0 (never negative)', lim_m == 0.0)
    lim_m, _ = nt.tracker_budget_limits(
        P['c0_m'], P['per_m'], P['per_rot_m'], P['yaw0_deg'],
        P['yaw_per_rot'], 0.0, 0.0, 0.15, 0.0, False)
    check('ratchet=False restores the legacy (refilling) behaviour',
          abs(lim_m - 0.15) < 1e-12)
    # a walk of N in-budget corrections at standstill must terminate
    used = 0.0
    steps = 0
    while steps < 1000:
        lim_m, _ = nt.tracker_budget_limits(
            P['c0_m'], P['per_m'], P['per_rot_m'], P['yaw0_deg'],
            P['yaw_per_rot'], 0.0, 0.0, used, 0.0, True)
        corr = 0.05
        if corr > lim_m:
            break
        used += corr
        steps += 1
    check('standstill walk terminates after <= 3 x 5 cm corrections',
          steps <= 3)

    # ---- gloc prior-referenced staleness gate --------------------------------
    print('prior_margin_verdict (map-staleness gate):')
    FRAC, EPS = 0.8, 0.02
    b, o = gl.prior_margin_verdict(0.05, 0.20, 0.99, 0.98, FRAC, EPS)
    check('genuine drift (rms 0.05 vs prior 0.20) ACCEPTED', b and o)
    b, o = gl.prior_margin_verdict(0.17, 0.20, 0.99, 0.98, FRAC, EPS)
    check('marginal 15 % improvement REJECTED (needs < 0.8x)', not b)
    b, o = gl.prior_margin_verdict(0.15, 0.20, 0.99, 0.98, FRAC, EPS)
    check('19 % better is a strict pass at the boundary', b and o)
    # changed garden: an obstacle disappeared -> the minimum is no better
    b, o = gl.prior_margin_verdict(0.19, 0.20, 0.97, 0.98, FRAC, EPS)
    check('changed garden (rms barely moves) -> REJECT', not b)
    # aliased slide onto unmapped space: rms drops but overlap collapses
    b, o = gl.prior_margin_verdict(0.05, 0.20, 0.40, 0.98, FRAC, EPS)
    check('slide onto unmapped space -> overlap veto', b and not o)
    b, o = gl.prior_margin_verdict(0.05, 0.20, 0.965, 0.98, FRAC, EPS)
    check('overlap within eps tolerance still passes', b and o)
    b, o = gl.prior_margin_verdict(0.05, None, 0.99, 0.98, FRAC, EPS)
    check('missing prior rms fails CLOSED (no evidence, no apply)', not b)
    b, o = gl.prior_margin_verdict(0.05, 0.20, None, None, FRAC, EPS)
    check('missing overlap does not veto on its own', b and o)

    print('-' * 62)
    if fails:
        print('FAILED: %d test(s): %s' % (len(fails), ', '.join(fails)))
        sys.exit(1)
    print('ALL PASSED')


if __name__ == '__main__':
    main()
