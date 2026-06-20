#!/usr/bin/env python3
"""Analyse a nav_telemetry JSONL log — safe to run anytime, even while logging.

Examples:
  nav_telemetry_analyze.py                      # latest log, whole file
  nav_telemetry_analyze.py --last 120           # last 120 s
  nav_telemetry_analyze.py --signals speed gps_status
  nav_telemetry_analyze.py /path/to/file.jsonl --last 60

Produces a navigation-health report plus a generic per-signal stats table.
Extensible: new signals in the YAML automatically appear in the generic table;
add bespoke checks in the EVALUATORS section below.
"""
import json
import sys
import os
import glob
import math
import re
import argparse

BRIDGE_RE = re.compile(
    r'gps_good=(?P<gps>\d)\s+active=(?P<active>\w+)\s+div_from_live=(?P<div>[\d.]+)m.*?'
    r'vo\[use=(?P<vo>\d).*?licp\[use=(?P<licp>\d).*?wheel\[use=(?P<wheel>\d)',
    re.S)


def find_latest(d='/home/vitulus/nav_logs', prefix='nav_telemetry'):
    cands = glob.glob(os.path.join(os.path.expanduser(d), prefix + '.jsonl'))
    return cands[0] if cands else None


def load(path, last_s):
    recs = []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            try:
                r = json.loads(line)
            except ValueError:
                continue
            if 'event' in r:      # header/marker records
                continue
            recs.append(r)
    if last_s and recs:
        tmax = recs[-1].get('t', 0)
        recs = [r for r in recs if r.get('t', 0) >= tmax - last_s]
    return recs


def nums(recs, key):
    return [r[key] for r in recs if isinstance(r.get(key), (int, float)) and not isinstance(r.get(key), bool)]


def stat(vals):
    if not vals:
        return None
    n = len(vals)
    mean = sum(vals) / n
    return dict(n=n, min=min(vals), max=max(vals), mean=mean, last=vals[-1])


def pct(c, n):
    return (100.0 * c / n) if n else 0.0


def transitions(recs, key, to_value):
    """count rising edges where key changes to to_value."""
    c, prev = 0, None
    for r in recs:
        v = r.get(key)
        if v == to_value and prev != to_value:
            c += 1
        prev = v
    return c


def parse_bridge(recs):
    out = []
    for r in recs:
        s = r.get('bridge_status')
        if not isinstance(s, str):
            continue
        m = BRIDGE_RE.search(s)
        if not m:
            continue
        out.append(dict(
            t=r.get('t'),
            gps=int(m.group('gps')), active=m.group('active'),
            div=float(m.group('div')),
            vo=int(m.group('vo')), licp=int(m.group('licp')), wheel=int(m.group('wheel')),
        ))
    return out


def report(recs, only=None):
    if not recs:
        print("no data records in window")
        return
    t0, t1 = recs[0].get('t', 0), recs[-1].get('t', 0)
    dur = t1 - t0
    print("=" * 64)
    print("nav_telemetry report  |  %d records  |  %.0f s  |  ~%.1f Hz"
          % (len(recs), dur, (len(recs) / dur) if dur else 0))
    print("=" * 64)

    # ---- EVALUATORS (bespoke navigation health) -----------------------
    # motion
    sp = nums(recs, 'speed')
    if sp:
        moving = sum(1 for v in sp if abs(v) > 0.05)
        print("MOTION    moving %4.0f%%   max %.2f m/s   mean(when moving) %.2f m/s"
              % (pct(moving, len(sp)),
                 max(abs(v) for v in sp),
                 (sum(abs(v) for v in sp if abs(v) > 0.05) / moving) if moving else 0.0))

    # GPS
    gs = nums(recs, 'gps_status')
    if gs:
        rtk = sum(1 for v in gs if int(v) == 2)
        cz = nums(recs, 'gps_cov_z')
        cx = nums(recs, 'gps_cov_x')
        print("GPS       RTK-fix(status=2) %4.0f%%   cov_x~%.2gm  cov_z~%.2gm"
              % (pct(rtk, len(gs)),
                 (math.sqrt(stat(cx)['mean']) if cx else float('nan')),
                 (math.sqrt(stat(cz)['mean']) if cz else float('nan'))))

    # bridge / source availability (the Phase-1 heart)
    b = parse_bridge(recs)
    if b:
        denied = [x for x in b if x['gps'] == 0]
        good = [x for x in b if x['gps'] == 1]
        print("BRIDGE    gps_good %4.0f%%   denied %4.0f%%   (n=%d)"
              % (pct(len(good), len(b)), pct(len(denied), len(b)), len(b)))
        if denied:
            vo_ok = sum(1 for x in denied if x['vo'])
            li_ok = sum(1 for x in denied if x['licp'])
            either = sum(1 for x in denied if x['vo'] or x['licp'])
            blind = sum(1 for x in denied if not (x['vo'] or x['licp']))
            divs = [x['div'] for x in denied]
            print("  WHEN GPS DENIED:  VO usable %3.0f%%   LiDAR usable %3.0f%%   "
                  "either %3.0f%%   BLIND(wheel-only) %3.0f%%"
                  % (pct(vo_ok, len(denied)), pct(li_ok, len(denied)),
                     pct(either, len(denied)), pct(blind, len(denied))))
            print("  bridge div_from_live (denied):  max %.2fm  mean %.2fm  last %.2fm"
                  % (max(divs), sum(divs) / len(divs), divs[-1]))
        # active-source distribution
        from collections import Counter
        dist = Counter(x['active'] for x in b)
        print("  active source:  " + "  ".join("%s=%.0f%%" % (k, pct(v, len(b)))
                                                for k, v in dist.most_common()))

    # VO / LiDAR tracking health
    for src, lost_key, q_key, q_label in (
            ('VO', 'vo_lost', 'vo_inliers', 'inliers'),
            ('LiDAR', 'licp_lost', 'licp_ratio', 'ratio')):
        lost_vals = [r.get(lost_key) for r in recs if isinstance(r.get(lost_key), bool)]
        if lost_vals:
            lost_pct = pct(sum(1 for v in lost_vals if v), len(lost_vals))
            rec_cnt = transitions(recs, lost_key, False)
            q = stat(nums(recs, q_key))
            qtxt = ("%s mean %.3g / max %.3g" % (q_label, q['mean'], q['max'])) if q else "no quality data"
            print("%-9s lost %4.0f%%   recoveries %d   (%s)"
                  % (src, lost_pct, rec_cnt, qtxt))

    # ---- GENERIC per-signal numeric table -----------------------------
    print("-" * 64)
    print("%-16s %6s %10s %10s %10s %10s" % ("signal", "valid%", "min", "mean", "max", "last"))
    keys = [k for k in recs[-1].keys() if k != 't']
    if only:
        keys = [k for k in keys if k in only]
    for k in keys:
        v = nums(recs, k)
        if v:
            s = stat(v)
            print("%-16s %5.0f%% %10.3g %10.3g %10.3g %10.3g"
                  % (k, pct(len(v), len(recs)), s['min'], s['mean'], s['max'], s['last']))
        else:
            # non-numeric (e.g. strings/bools) — show validity + last value
            last = next((r.get(k) for r in reversed(recs) if r.get(k) is not None), None)
            valid = sum(1 for r in recs if r.get(k) is not None)
            sv = str(last)
            print("%-16s %5.0f%%   last=%s" % (k, pct(valid, len(recs)), sv[:40]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('logfile', nargs='?', default=None)
    ap.add_argument('--last', type=float, default=0, help="only the last N seconds")
    ap.add_argument('--signals', nargs='*', default=None, help="restrict generic table to these")
    a = ap.parse_args()
    path = a.logfile or find_latest()
    if not path or not os.path.isfile(path):
        print("no log file found (looked in /home/vitulus/nav_logs)")
        sys.exit(1)
    print("file: %s" % path)
    report(load(path, a.last), only=a.signals)


if __name__ == '__main__':
    main()
