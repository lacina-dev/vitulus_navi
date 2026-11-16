#!/usr/bin/env python3
"""T35 K1 (~rtk_mag_report_wide, with ~rtk_mag_report): M2 compares every RTK
heading sample whose NAV-RELPOSNED is fixed, valid and fresh - also the ones
the node does not take into the heading (variance out of the band, heading /
position fix not usable). REPORT ONLY.

Why: on 48 arrivals at the garage with a wrong RTK heading (25 recorded
drives) M2 as it is would raise the alarm at 16 (live: 2 of 23). It is not the
magnetometer - its last `ok` offset is 0-15 s old there and M2 already carries
it by the gyro for 30 s. The wrong heading comes with bad quality: of the
samples more than 6 deg off, 44 % fail the variance band and 31 % the heading
fix; M2 looked only at samples that pass everything, and those rarely add up
to the 0.3 s of disagreement the alarm needs. With this parameter: 36 of 48.

Run:  python3 test_t35_k1_rtk_mag_report_wide.py   (needs rospy - robot env)
"""
import os
import sys
import types

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
import _t36_replay as R  # noqa: E402
import test_t35_m2_rtk_mag_report as M2  # noqa: E402

WIDE = dict(M2.ON, rtk_mag_report_wide=True)


def main():
    fails = []

    def check(name, cond, detail=''):
        print('  %-72s %s %s' % (name, 'OK' if cond else 'FAIL', detail))
        if not cond:
            fails.append(name)

    nt = R.load('navi_transform_t35_k1b')

    def run(params, **bad):
        """3 s of agreement, then 4 s of RTK 9 deg off with the given quality."""
        f = M2.Feed(nt, params)
        f.go(3.0)
        st0 = f.heading_state()
        for k, v in bad.items():
            if k in ('fix_usable', 'heading_fix_usable'):
                setattr(f.n, k, v)
        kw = {k: v for k, v in bad.items() if k in ('acc_deg', 'flags')}
        loc = f.go(4.0, rtk_err=9.0, **kw)
        return f, loc, st0

    print('RTK 9 deg off the magnetometer, by the quality of the samples:')
    for label, bad in (('variance above the band (accHeading 2.0 deg)', {'acc_deg': 2.0}),
                       ('variance under the band (accHeading 0.9 deg)', {'acc_deg': 0.9}),
                       ('heading fix not usable', {'heading_fix_usable': False}),
                       ('position fix not usable', {'fix_usable': False})):
        f0, l0, _ = run(M2.ON, **bad)
        f1, l1, st0 = run(WIDE, **bad)
        check('%s: M2 alone silent, wide raises the alarm' % label,
              l0['rtk_mag_disagree'] is False and l0['rtk_mag_disagree_count'] == 0
              and l1['rtk_mag_disagree'] is True and l1['rtk_mag_disagree_count'] == 1
              and l1['rtk_mag_diff_deg'] == 9.0,
              '%s / %s' % (l0['rtk_mag_diff_deg'], l1['rtk_mag_diff_deg']))
        check('... the heading takes none of those samples, wide or not (report only)',
              f1.heading_state() == f0.heading_state() and f1.heading_state()[0] == st0[0])
    f0, l0, _ = run(M2.ON)
    f1, l1, _ = run(WIDE)
    check('samples the node takes: the same alarm with and without wide',
          l0['rtk_mag_disagree'] is True and {k: l0[k] for k in M2.KEYS} == {k: l1[k] for k in M2.KEYS}
          and f0.heading_state() == f1.heading_state())
    for label, flags in (('NAV-RELPOSNED float', M2.FLOAT), ('heading not valid', (2 << 3) | 0x7)):
        _, l1, _ = run(WIDE, flags=flags)
        check('%s: not compared even with wide' % label,
              l1['rtk_mag_disagree'] is False and l1['rtk_mag_disagree_count'] == 0)
    f = M2.Feed(nt, WIDE)
    f.go(3.0)
    f.n._relpos_rx_t = R.CLOCK[0] - 5.0
    real = f.n.callback_relposned
    f.n.callback_relposned = lambda m: None            # RELPOSNED stopped coming
    loc = f.go(4.0, rtk_err=9.0)
    f.n.callback_relposned = real
    check('NAV-RELPOSNED stale: not compared', loc['rtk_mag_disagree_count'] == 0)
    f = M2.Feed(nt, WIDE)
    f.n._relpos_flags = None
    f.n.callback_relposned = lambda m: None
    f.go(3.0)
    loc = f.go(4.0, rtk_err=9.0)
    check('no NAV-RELPOSNED at all: not compared', loc['rtk_mag_disagree_count'] == 0)
    f = M2.Feed(nt, {'rtk_mag_report_wide': True})
    loc = f.go(5.0, rtk_err=9.0, acc_deg=2.0)
    check('wide without ~rtk_mag_report: nothing runs, no field',
          f.n._rmr_enable is False and not [k for k in loc if k.startswith('rtk_mag')])
    f = M2.Feed(nt, M2.ON)
    check('default: wide off', f.n._rmr_wide is False)

    print('\n%s' % ('ALL OK' if not fails else 'FAILED: %d' % len(fails)))
    return 1 if fails else 0


if __name__ == '__main__':
    sys.exit(main())
