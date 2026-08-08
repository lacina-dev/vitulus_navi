#!/usr/bin/env python3
"""Unit tests for the navi_transform anchor-pairing helpers (AUDIT P1-2,
2026-08-08): _hist_at / _interp_scalar / _interp_angle, incl. the yaw +-pi
wrap and out-of-range timestamps that back the stamped GPS<->wheel pairing
(BAG_ANALYZA_2026-08-07.md).

Run:  python3 test_anchor_helpers.py   (needs rospy importable — robot env)
"""
import math
import os
import sys
import importlib.util
from collections import deque

NODE = os.path.join(os.path.dirname(__file__), '..', 'nodes', 'navi_transform')


def load_node_class():
    spec = importlib.util.spec_from_loader(
        'navi_transform_node',
        importlib.machinery.SourceFileLoader('navi_transform_node', NODE))
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    return mod.NavTfNode


def main():
    N = load_node_class()
    fails = []

    def check(name, cond):
        print('  %-46s %s' % (name, 'OK' if cond else 'FAIL'))
        if not cond:
            fails.append(name)

    print('_hist_at:')
    h = deque([(0.0, 0.0, 10.0, 3.1), (1.0, 1.0, 20.0, -3.1),
               (2.0, 2.0, 30.0, -3.0)])
    r = N._hist_at(h, 0.5)
    check('midpoint found', r is not None and r[2] == 0.5)
    check('exact first stamp', N._hist_at(h, 0.0) is not None)
    check('exact last stamp', N._hist_at(h, 2.0) is not None)
    check('before range -> None (late-history GPS)', N._hist_at(h, -0.1) is None)
    check('after range -> None (future stamp)', N._hist_at(h, 2.1) is None)
    check('empty history -> None (restart)', N._hist_at(deque(), 0.5) is None)
    check('single entry -> None', N._hist_at(deque([(0.0, 1.0)]), 0.0) is None)
    dup = deque([(1.0, 0.0), (1.0, 5.0)])          # zero-dt pair
    rd = N._hist_at(dup, 1.0)
    check('zero-dt pair -> f=0 (no div-by-zero)', rd is not None and rd[2] == 0.0)
    seg = N._hist_at(h, 1.5)
    check('picks correct segment', seg is not None and seg[0][0] == 1.0
          and seg[1][0] == 2.0 and abs(seg[2] - 0.5) < 1e-12)

    print('_interp_scalar:')
    check('linear midpoint', abs(N._interp_scalar(10.0, 20.0, 0.5) - 15.0) < 1e-12)
    check('f=0 endpoint', N._interp_scalar(10.0, 20.0, 0.0) == 10.0)
    check('f=1 endpoint', N._interp_scalar(10.0, 20.0, 1.0) == 20.0)

    print('_interp_angle:')
    y = N._interp_angle(3.1, -3.1, 0.5)            # shortest arc crosses +-pi
    check('pi-crossing midpoint ~ +-pi', abs(abs(y) - math.pi) < 1e-9)
    y = N._interp_angle(-0.1, 0.1, 0.5)
    check('zero-crossing midpoint ~ 0', abs(y) < 1e-12)
    y = N._interp_angle(0.0, 1.0, 0.25)
    check('plain quarter', abs(y - 0.25) < 1e-12)
    y = N._interp_angle(math.pi - 0.01, -math.pi + 0.01, 1.0)
    check('f=1 lands on target (mod 2pi)',
          abs(math.atan2(math.sin(y - (-math.pi + 0.01)),
                         math.cos(y - (-math.pi + 0.01)))) < 1e-9)

    print('-' * 56)
    if fails:
        print('FAILED: %d test(s): %s' % (len(fails), ', '.join(fails)))
        sys.exit(1)
    print('ALL PASSED')


if __name__ == '__main__':
    main()
