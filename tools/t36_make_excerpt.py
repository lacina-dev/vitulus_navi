#!/usr/bin/env python3
"""Cut the T36 test excerpt out of the analyst's extract of bag
magtest3_2026-11-14 (extract.py -> magtest3.npz: one array per topic, first
column = bag time). Usage:  t36_make_excerpt.py <magtest3.npz> [out.npz]

Windows (seconds from the first /tf of the bag):
   84..113  tracker -> GNSS heading hand-over at UNDOCK (robot stands)
  314..334  driving on GPS, reversing
  530..560  driving on GPS, turning
  793..823  docking, first write of the dock anchor
"""
import os
import sys

import numpy as np

WINDOWS = ((84.0, 113.0), (314.0, 334.0), (530.0, 560.0), (793.0, 823.0))
KEYS = ('mo', 'ob', 'ekf', 'wo', 'hrel', 'nh', 'dock', 'imu')


def main():
    src = sys.argv[1]
    out = sys.argv[2] if len(sys.argv) > 2 else os.path.join(
        os.path.dirname(os.path.abspath(__file__)), 'data', 't36_magtest3_excerpt.npz')
    z = np.load(src, allow_pickle=True)
    t0 = float(z['ob'][0, 0])

    def cut(t):
        m = np.zeros(len(t), dtype=bool)
        for a, b in WINDOWS:
            m |= (t >= t0 + a - 1.0) & (t <= t0 + b + 1.0)
        return m
    d = {k: z[k][cut(z[k][:, 0])] for k in KEYS}
    m = cut(z['loc_t'])
    d['loc_t'] = z['loc_t'][m]
    d['loc_json'] = z['loc_json'][m]
    d['T0'] = np.array(t0)
    np.savez_compressed(out, **d)
    print(out, os.path.getsize(out), {k: len(v) for k, v in d.items() if k != 'T0'})


if __name__ == '__main__':
    main()
