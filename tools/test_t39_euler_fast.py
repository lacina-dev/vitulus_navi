#!/usr/bin/env python3
"""T39: navi_transform's own euler_from_quaternion (the arithmetic of
tf.transformations written out for axes 'sxyz', no numpy matrices) returns BIT
FOR BIT what tf.transformations.euler_from_quaternion returns.

  - 100 000 random quaternions: not normalized, unit, robot-like (small roll and
    pitch, any yaw), magnitudes 1e-6..1e6, at and next to the gimbal lock;
  - edge cases: zero and tiny quaternions (the identity branch), +-90 deg pitch,
    signed zeros, 1e-200 / 1e150 components, NaN and inf components;
  - other axes go to tf.transformations unchanged;
  - numpy scalars and tuples as input.
The function is taken from the node's source (no ROS master, the node is not
run).  Needs tf.transformations and numpy (robot environment).

Run:  python3 tools/test_t39_euler_fast.py
"""
import ast
import math
import os
import random
import struct
import sys
import time
import unittest

import numpy as np
from tf.transformations import euler_from_quaternion as tf_euler

HERE = os.path.dirname(os.path.abspath(__file__))
NODE = os.path.join(os.path.dirname(HERE), 'nodes', 'navi_transform')


def load():
    """euler_from_quaternion and _EULER_EPS exactly as they stand in the node."""
    src = open(NODE).read()
    tree = ast.parse(src)
    keep = [n for n in tree.body
            if (isinstance(n, ast.FunctionDef) and n.name == 'euler_from_quaternion')
            or (isinstance(n, ast.Assign) and getattr(n.targets[0], 'id', '') == '_EULER_EPS')]
    assert len(keep) == 2, [type(n).__name__ for n in keep]
    ns = {'math': math, 'np': np, '_tf_euler_from_quaternion': tf_euler}
    exec(compile(ast.Module(body=keep, type_ignores=[]), NODE, 'exec'), ns)
    return ns['euler_from_quaternion'], src


FAST, SRC = load()


def same(a, b):
    """Bit equality; a NaN equals a NaN."""
    if len(a) != len(b):
        return False
    for x, y in zip(a, b):
        if x != x or y != y:
            if not (x != x and y != y):
                return False
        elif struct.pack('<d', x) != struct.pack('<d', y):
            return False
    return True


def randoms(n, seed=39):
    random.seed(seed)
    out = []
    for i in range(n):
        k = i % 5
        if k == 0:
            q = [random.gauss(0, 1) for _ in range(4)]
        elif k == 1:
            q = [random.gauss(0, 1) for _ in range(4)]
            norm = math.sqrt(sum(a * a for a in q))
            q = [a / norm for a in q]
        elif k == 2:
            r, p, yw = random.gauss(0, 0.05), random.gauss(0, 0.05), random.uniform(-math.pi, math.pi)
            cr, sr, cp, sp = math.cos(r / 2), math.sin(r / 2), math.cos(p / 2), math.sin(p / 2)
            cw, sw = math.cos(yw / 2), math.sin(yw / 2)
            q = [sr * cp * cw - cr * sp * sw, cr * sp * cw + sr * cp * sw,
                 cr * cp * sw - sr * sp * cw, cr * cp * cw + sr * sp * sw]
        elif k == 3:
            q = [random.gauss(0, 1) * 10 ** random.uniform(-6, 6) for _ in range(4)]
        else:
            yw = random.uniform(-math.pi, math.pi)
            p = random.choice((1, -1)) * (math.pi / 2 - 10 ** random.uniform(-16, -3) * random.choice((0, 1)))
            cp, sp, cw, sw = math.cos(p / 2), math.sin(p / 2), math.cos(yw / 2), math.sin(yw / 2)
            q = [-sp * sw, sp * cw, cp * sw, cp * cw]
        out.append(q)
    return out


H = math.sqrt(0.5)
NAN, INF = float('nan'), float('inf')
EDGE = [[0, 0, 0, 1], [0, 0, 0, 0], [1e-9, 0, 0, 0], [3e-8, 0, 0, 0], [0, 0, 1e-8, 1e-8],
        [0, 0, 0, -1], [0, 0, 1, 0], [1, 0, 0, 0], [0, 1, 0, 0], [0.5, 0.5, 0.5, 0.5],
        [0, H, 0, H], [0, -H, 0, H], [H, 0, H, 0], [1e-200, 0, 0, 1e-200], [1e150, 0, 0, 1e150],
        [0.0, 0.0, 0.0, 2.0], [-0.0, 0.0, -0.0, 1.0], [-0.0, -0.0, -0.0, -0.0],
        [NAN, 0, 0, 1], [0, 0, NAN, 1], [NAN, NAN, NAN, NAN], [INF, 0, 0, 1], [0, 0, 0, INF],
        [-INF, 1, 2, 3], [1e308, 1e308, 0, 0]]


class EulerFast(unittest.TestCase):

    def test_bit_identical_on_100000_random_quaternions(self):
        bad = []
        for q in randoms(100000):
            a, b = FAST(q), tuple(tf_euler(q))
            if not same(a, b):
                bad.append((q, a, b))
        self.assertEqual(len(bad), 0, bad[:3])

    def test_bit_identical_on_edge_cases(self):
        for q in EDGE:
            q = [float(v) for v in q]
            with np.errstate(all='ignore'):
                want = tuple(tf_euler(q))
            got = FAST(q)
            self.assertTrue(same(got, want), (q, got, want))
            self.assertTrue(all(type(v) is float for v in got), (q, got))

    def test_identity_branch_and_gimbal_lock_are_reached(self):
        self.assertEqual(struct.pack('<3d', *FAST([0.0, 0.0, 0.0, 0.0])), struct.pack('<3d', 0.0, -0.0, 0.0))
        self.assertEqual(FAST([0.0, H, 0.0, H])[2], 0.0)          # pitch +90: yaw is defined as 0
        self.assertAlmostEqual(FAST([0.0, H, 0.0, H])[1], math.pi / 2, places=7)

    def test_input_types(self):
        q = [0.01, -0.02, 0.6, 0.8]
        want = tuple(tf_euler(q))
        for form in (tuple(q), np.array(q), [np.float64(v) for v in q], q + [9.0]):
            self.assertTrue(same(FAST(form), want), type(form))

    def test_other_axes_go_to_tf(self):
        q = [0.1, 0.2, 0.3, 0.9]
        for axes in ('rzyx', 'szyx', 'sxyx', 'ryxz'):
            self.assertTrue(same(tuple(FAST(q, axes)), tuple(tf_euler(q, axes))), axes)
        self.assertTrue(same(FAST(q, 'sxyz'), tuple(tf_euler(q, 'sxyz'))))

    def test_the_node_uses_only_its_own_function(self):
        self.assertNotIn('from tf.transformations import euler_from_quaternion, ', SRC)
        self.assertEqual(SRC.count('_tf_euler_from_quaternion('), 1, 'tf is called only for other axes')
        self.assertGreater(SRC.count('euler_from_quaternion('), 25)

    def test_speed_is_reported(self):
        qs = randoms(20000, seed=1)
        t = time.perf_counter()
        for q in qs:
            tf_euler(q)
        a = (time.perf_counter() - t) / len(qs)
        t = time.perf_counter()
        for q in qs:
            FAST(q)
        b = (time.perf_counter() - t) / len(qs)
        sys.stderr.write('tf.transformations %.1f us, direct %.2f us per call (x%.0f)\n'
                         % (a * 1e6, b * 1e6, a / b))
        self.assertLess(b, a)


if __name__ == '__main__':
    unittest.main(verbosity=1)
