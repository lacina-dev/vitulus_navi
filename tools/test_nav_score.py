#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Jednotkove testy tools/nav_score.py na malych syntetickych datech.
Bez ROS: testuji se ciste funkce metrik a score() nad vytahem (dict poli).
Spusteni: python3 tools/test_nav_score.py   (nebo pytest tools/test_nav_score.py)
"""
import io
import json
import math
import os
import struct
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import nav_score as ns  # noqa: E402

T0 = 1000.0


def drive(n=600, v=0.3, hz=30.0, yaw=0.0):
    """Rovna jizda rychlosti v: vraci (t, x, y) po 1/hz."""
    t = T0 + np.arange(n) / hz
    s = v * (t - T0)
    return t, s * math.cos(yaw), s * math.sin(yaw)


def odo(t, x, y, yaw=0.0, v=0.3, w=0.0, z=0.0):
    """Pole odometrie N x 9 (bag_t, stamp, x, y, z, yaw, vx, vy, wz)."""
    n = len(t)
    f = lambda a: np.full(n, a, dtype=float) if np.isscalar(a) else np.asarray(a, dtype=float)
    return np.column_stack([t, t, f(x), f(y), f(z), f(yaw), f(v), np.zeros(n), f(w)])


def tfrows(t, x, y, yaw=0.0, z=0.0):
    """Pole TF N x 6 (bag_t, stamp, x, y, z, yaw)."""
    n = len(t)
    f = lambda a: np.full(n, a, dtype=float) if np.isscalar(a) else np.asarray(a, dtype=float)
    return np.column_stack([t, t, f(x), f(y), f(z), f(yaw)])


def imu(t, yaw=0.0, w=0.0):
    n = len(t)
    f = lambda a: np.full(n, a, dtype=float) if np.isscalar(a) else np.asarray(a, dtype=float)
    return np.column_stack([t, t, f(yaw), f(w)])


def base_extract(n=600, v=0.3):
    """Cista jizda: kola, EKF i TF se presne shoduji, map->odom je nulovy."""
    t, x, y = drive(n, v)
    D = {'wheel': odo(t, x, y, v=v), 'ekf': odo(t, x, y, v=v), 'odom': odo(t, x, y, v=v),
         'imu': imu(t), 'mo': tfrows(t, 0.0, 0.0), 'ob': tfrows(t, x, y), 'counts': {}}
    return D, t, x, y


class Helpers(unittest.TestCase):
    def test_wrap(self):
        self.assertAlmostEqual(float(ns.wrap(3 * math.pi + 0.1)), -math.pi + 0.1)

    def test_wheel_path_integrates_speed(self):
        t, x, y = drive(300, 0.2)
        _, s = ns.wheel_path(odo(t, x, y, v=0.2))
        self.assertAlmostEqual(s[-1], 0.2 * (t[-1] - t[0]), places=6)

    def test_wheel_path_sign(self):
        t, x, y = drive(300, -0.2)
        _, s = ns.wheel_path(odo(t, x, y, v=-0.2))
        self.assertLess(s[-1], 0)

    def test_first_sustained_skips_short_run(self):
        t = np.arange(0, 100, 1.0)
        c = np.zeros(100, dtype=bool)
        c[5:8] = True          # 2 s, kratky
        c[40:80] = True
        self.assertEqual(ns.first_sustained(t, c, 0.0, 10.0), 40.0)
        self.assertIsNone(ns.first_sustained(t, np.zeros(100, dtype=bool), 0.0, 10.0))


class Jumps(unittest.TestCase):
    def test_clean_drive_has_no_jumps(self):
        D, *_ = base_extract()
        m = ns.score(D)['metrics']
        for p in ('mo', 'ob', 'mb'):
            self.assertEqual(m[p + '_n_gt1cm'], 0, p)
            self.assertLess(m[p + '_pos_max_cm'], 0.01, p)
            self.assertEqual(m[p + '_n_yaw_gt03'], 0, p)
        self.assertEqual(m['ekf_steps_gt1cm_per_min'], 0.0)

    def test_map_odom_jump_is_attributed_to_mo(self):
        D, t, x, y = base_extract()
        mx = np.zeros(len(t))
        mx[300:] = 0.20                       # skok map->odom o 20 cm v jednom tiku
        D['mo'] = tfrows(t, mx, 0.0)
        m = ns.score(D)['metrics']
        self.assertAlmostEqual(m['mo_pos_max_cm'], 20.0, places=3)
        self.assertEqual(m['mo_n_gt3cm'], 1)
        self.assertEqual(m['ob_n_gt1cm'], 0)
        self.assertAlmostEqual(m['mb_pos_max_cm'], 20.0, places=3)
        self.assertEqual(m['mb_n_gt3cm'], 1)

    def test_ekf_step_is_attributed_to_ob(self):
        D, t, x, y = base_extract()
        x2 = x.copy()
        x2[200:] += 0.04                      # set_pose: krok EKF o 4 cm
        D['ekf'] = odo(t, x2, y)
        D['odom'] = odo(t, x2, y)
        D['ob'] = tfrows(t, x2, y)
        m = ns.score(D)['metrics']
        self.assertEqual(m['ob_n_gt3cm'], 1)
        self.assertAlmostEqual(m['ob_pos_max_cm'], 4.0, places=3)
        self.assertEqual(m['mo_n_gt1cm'], 0)
        drive_min = (t[-1] - t[0]) / 60.0
        self.assertAlmostEqual(m['ekf_steps_gt3cm_per_min'], 1.0 / drive_min, delta=0.05)
        self.assertAlmostEqual(m['ekf_steps_gt1cm_per_min'], 1.0 / drive_min, delta=0.05)

    def test_smooth_slew_is_not_a_jump(self):
        D, t, x, y = base_extract()
        # map->odom se plynule posune o 30 cm rychlosti 0,1 m/s (3,3 mm za tik)
        mx = np.clip((t - T0 - 5.0) * 0.1, 0.0, 0.30)
        D['mo'] = tfrows(t, mx, 0.0)
        m = ns.score(D)['metrics']
        self.assertEqual(m['mo_n_gt1cm'], 0)
        self.assertLess(m['mo_pos_max_cm'], 0.4)

    def test_map_odom_rotation_moves_robot(self):
        # otoceni map->odom o 2 st kolem pocatku mapy s robotem 6 m daleko = skok ~21 cm
        D, t, x, y = base_extract()
        myaw = np.zeros(len(t))
        myaw[590:] = math.radians(2.0)
        D['mo'] = tfrows(t, 0.0, 0.0, myaw)
        m = ns.score(D)['metrics']
        self.assertAlmostEqual(m['mo_yaw_max_deg'], 2.0, places=3)
        self.assertAlmostEqual(m['mo_pos_max_cm'], 100 * 2 * x[590] * math.sin(math.radians(1.0)),
                               delta=0.5)

    def test_yaw_step_counted_only_when_calm(self):
        D, t, x, y = base_extract()
        oyaw = np.zeros(len(t))
        oyaw[250:] = math.radians(1.0)        # krok kurzu 1 st pri rovne jizde
        D['ob'] = tfrows(t, x, y, oyaw)
        D['ekf'] = odo(t, x, y, oyaw)
        m = ns.score(D)['metrics']
        self.assertEqual(m['ob_n_yaw_gt03'], 1)
        self.assertAlmostEqual(m['ob_yaw_max_deg'], 1.0, places=3)
        # tentyz krok pri otaceni (gyro i kola hlasi 0,5 rad/s) se nepocita
        D['imu'] = imu(t, w=0.5)
        D['wheel'] = odo(t, x, y, w=0.5)
        m = ns.score(D)['metrics']
        self.assertEqual(m['ob_n_yaw_gt03'], 0)

    def test_pose_time_matching_removes_aliasing(self):
        # /tf kazdy treti tik zopakuje predchozi pozu EKF (nesoubeh smycek)
        D, t, x, y = base_extract()
        idx = np.arange(len(t))
        src = idx - (idx % 3 == 2)
        tf_t = t + 0.004
        ob = np.column_stack([tf_t, tf_t, x[src], y[src], np.zeros(len(t)), np.zeros(len(t))])
        D['ob'] = ob
        D['mo'] = np.column_stack([tf_t, tf_t] + [np.zeros(len(t))] * 4)
        R = ns.score(D)
        self.assertGreater(R['meta']['tf_pose_time_matched_pct'], 99.0)
        self.assertEqual(R['metrics']['ob_n_gt1cm'], 0)
        self.assertLess(R['metrics']['ob_pos_max_cm'], 0.01)
        self.assertAlmostEqual(R['metrics']['tf_stale_pct'], 100.0 / 3, delta=1.0)


class ZeroTwist(unittest.TestCase):
    def test_zero_twist_share(self):
        t, x, y = drive(400, 0.3)
        wheel = odo(t, x, y, v=0.3)
        v = np.full(len(t), 0.3)
        v[::10] = 0.0                         # kazdy desaty vzorek ma nulovou rychlost
        self.assertAlmostEqual(ns.zero_twist_pct(odo(t, x, y, v=v), wheel), 10.0, places=6)

    def test_standing_zero_twist_not_counted(self):
        t = T0 + np.arange(300) / 30.0
        wheel = odo(t, 0.0, 0.0, v=0.0)
        self.assertIsNone(ns.zero_twist_pct(odo(t, 0.0, 0.0, v=0.0), wheel))

    def test_fallback_to_ekf_topic_is_noted(self):
        D, *_ = base_extract()
        del D['odom']
        R = ns.score(D)
        self.assertEqual(R['metrics']['odom_zero_twist_pct'], 0.0)
        self.assertIn('odom_zero_twist_pct', R['notes'])


class Owner(unittest.TestCase):
    def _loc(self, D, t, owners, anchor=None, heading=None):
        lt = T0 + np.arange(len(owners), dtype=float)
        D['loc_t'] = lt
        D['loc_owner'] = np.array(owners)
        D['loc_anchor_age'] = np.full(len(lt), 0.1) if anchor is None else np.asarray(anchor, float)
        D['loc_heading_age'] = np.full(len(lt), 0.1) if heading is None else np.asarray(heading, float)
        return lt

    def test_transitions_and_rate(self):
        D, t, x, y = base_extract()
        self._loc(D, t, ['dr'] * 5 + ['gps'] * 5 + ['dr'] * 5 + ['gps'] * 5)
        m = ns.score(D)['metrics']
        self.assertEqual(m['owner_changes'], 3)
        self.assertAlmostEqual(m['owner_dr_gps_per_hour'], 3 / (19.0 / 3600.0), places=3)

    def test_takeover_gap_and_settle(self):
        D, t, x, y = base_extract()
        lt = self._loc(D, t, ['dr'] * 8 + ['gps'] * 12)
        t_take = lt[8]
        # EKF je do prevzeti 10 cm vedle GPS, pak se 2 s plynule dorovnava
        off = 0.10 * np.clip(1.0 - (t - t_take) / 2.0, 0.0, 1.0)
        D['ekf'] = odo(t, x, y + off)
        D['ob'] = tfrows(t, x, y + off)
        tg = t[::6]
        D['gps'] = np.column_stack([tg, tg, x[::6], y[::6], np.zeros(len(tg))])
        m = ns.score(D)['metrics']
        self.assertAlmostEqual(m['takeover_gap_med_cm'], 10.0, delta=0.2)
        self.assertAlmostEqual(m['takeover_settle_max_s'], 1.0, delta=0.25)   # pod 5 cm po 1 s
        self.assertLess(m['gps_gap_med_cm'], 1.0)

    def test_anchor_freezes(self):
        D, t, x, y = base_extract()
        age = [0.1] * 20
        age[6:9] = [6.0, 7.0, 8.0]            # jedna epizoda > 5 s
        age[14] = 9.0                         # druha
        self._loc(D, t, ['gps'] * 20, anchor=age, heading=[float('nan')] * 20)
        R = ns.score(D)
        m = R['metrics']
        self.assertEqual(m['anchor_freezes'], 2)
        self.assertEqual(m['anchor_age_max_s'], 9.0)
        self.assertIsNone(m['heading_anchor_age_max_s'])
        self.assertIn('heading_anchor_age_max_s', R['notes'])


class Dock(unittest.TestCase):
    def _dock(self, t, seq):
        """seq = [(od_s, hodnota)], 10 Hz."""
        dt = T0 + np.arange(0, t[-1] - T0, 0.1)
        v = np.zeros(len(dt))
        for (a, val) in seq:
            v[dt >= T0 + a] = val
        return np.column_stack([dt, v])

    def test_edges(self):
        d = np.array([[0, 0], [1, 0], [2, 1], [3, 3], [4, 2], [5, 0], [6, 0]], dtype=float)
        und, dk = ns.dock_edges(d)
        self.assertEqual(und, [2.0])
        self.assertEqual(dk, [5.0])

    def test_dock_jump(self):
        D, t, x, y = base_extract()
        D['dock'] = self._dock(t, [(0, 3), (10, 0)])
        mx = np.zeros(len(t))
        my = np.zeros(len(t))
        i = int(np.searchsorted(t, T0 + 12.0))    # 2 s po hrane (debounce drzeni doku)
        mx[i:], my[i:] = 0.15, 0.20
        D['mo'] = tfrows(t, mx, my)
        m = ns.score(D)['metrics']
        self.assertAlmostEqual(m['dock_jump_cm'], 25.0, places=3)
        self.assertAlmostEqual(m['dock_shift_cm'], 25.0, places=3)

    def test_undock_times(self):
        n = 1800                                   # 60 s
        t = T0 + np.arange(n) / 30.0
        v = np.where(((t - T0 > 6) & (t - T0 < 10)) | (t - T0 > 42), 0.3, 0.0)
        x = np.concatenate([[0], np.cumsum(v[:-1] * np.diff(t))])
        D = {'wheel': odo(t, x, 0.0, v=v), 'ekf': odo(t, x, 0.0, v=v), 'imu': imu(t),
             'mo': tfrows(t, 0.0, 0.0), 'ob': tfrows(t, x, 0.0), 'counts': {}}
        D['dock'] = self._dock(t, [(0, 0), (5, 1), (40, 3)])
        tp = T0 + np.arange(0, 60, 0.2)
        carr = np.where(((tp - T0 > 2) & (tp - T0 < 7)) | (tp - T0 > 25), 2, 1)   # kratky fix v doku
        D['pvt'] = np.column_stack([tp, np.full(len(tp), 3), carr, np.full(len(tp), 0.014)])
        D['loc_t'] = T0 + np.arange(60, dtype=float)
        D['loc_owner'] = np.array(['dock'] * 5 + ['dr'] * 25 + ['gps'] * 30)
        D['loc_anchor_age'] = np.full(60, 0.1)
        D['loc_heading_age'] = np.full(60, 0.1)
        m = ns.score(D)['metrics']
        self.assertAlmostEqual(m['to_rtk_fixed_s'], 20.0, delta=0.3)
        self.assertAlmostEqual(m['to_owner_gps_s'], 25.0, delta=0.01)
        self.assertAlmostEqual(m['to_undocked_s'], 35.0, delta=0.11)
        self.assertAlmostEqual(m['to_depart_s'], 37.0, delta=0.15)
        self.assertAlmostEqual(m['undock_standing_s'], 31.0, delta=0.3)

    def test_yaw_handover_smooth_and_jittery(self):
        n = 900
        t = T0 + np.arange(n) / 30.0
        D = {'wheel': odo(t, 0.0, 0.0, v=0.0), 'ekf': odo(t, 0.0, 0.0, v=0.0), 'imu': imu(t),
             'counts': {}}
        D['dock'] = self._dock(t, [(0, 0), (5, 1)])
        # map->odom se od 10. s otaci z 10 st na 0 rychlosti 5 st/s (2 s)
        myaw = np.radians(np.clip(10.0 - 5.0 * (t - T0 - 10.0), 0.0, 10.0))
        D['mo'] = tfrows(t, 0.0, 0.0, myaw)
        # plynule: odom->base se otaci presne opacne -> kurz v mape stoji
        D['ob'] = tfrows(t, 0.0, 0.0, math.radians(10.0) - myaw)
        m = ns.score(D)['metrics']
        self.assertAlmostEqual(m['yaw_handover_s'], 2.0, delta=0.1)
        self.assertAlmostEqual(m['yaw_handover_turn_deg'], 10.0, delta=0.2)
        self.assertLess(m['yaw_handover_excess_deg'], 0.01)
        # cukave: odom->base dobiha po krocich 0,5 st jednou za 0,2 s
        lag = np.radians(np.floor((10.0 - np.degrees(myaw)) / 0.5 - 1e-9).clip(0) * 0.5)
        D['ob'] = tfrows(t, 0.0, 0.0, lag)
        m = ns.score(D)['metrics']
        self.assertGreater(m['yaw_handover_excess_deg'], 5.0)
        self.assertGreater(m['yaw_handover_max_step_deg'], 0.3)

    def test_standing_jitter(self):
        n = 600
        t = T0 + np.arange(n) / 30.0
        yaw = np.radians(0.5) * np.sign(np.sin(2 * math.pi * (t - T0)))   # +-0,5 st, 1 Hz
        D = {'wheel': odo(t, 0.0, 0.0, v=0.0), 'ekf': odo(t, 0.0, 0.0, yaw, v=0.0),
             'imu': imu(t), 'mo': tfrows(t, 0.0, 0.0), 'ob': tfrows(t, 0.0, 0.0, yaw), 'counts': {}}
        D['loc_t'] = T0 + np.arange(20, dtype=float)
        D['loc_owner'] = np.array(['gps'] * 20)
        D['loc_anchor_age'] = np.full(20, 0.1)
        D['loc_heading_age'] = np.full(20, 0.1)
        m = ns.score(D)['metrics']
        self.assertAlmostEqual(m['gps_standing_yaw_p2p_deg'], 1.0, delta=0.01)
        self.assertAlmostEqual(m['gps_standing_yaw_std_deg'], 0.5, delta=0.05)
        self.assertLess(m['gps_standing_pos_p95_cm'], 0.001)


class TakeoverWander(unittest.TestCase):
    """Oddil C3: pohyb obrazu robota v mape po prevzeti GPS, ktery kola nevysvetli."""
    TG = 25.0          # s od T0: owner -> gps

    def _run(self, v=0.0, w=0.0, n=1800):
        t = T0 + np.arange(n) / 30.0
        th = w * (t - T0)
        if w:
            x, y = v / w * np.sin(th), v / w * (1.0 - np.cos(th))     # presny oblouk
        else:
            x, y = v * (t - T0), np.zeros(n)
        D = {'wheel': odo(t, x, y, th, v=v, w=w), 'ekf': odo(t, x, y, th, v=v, w=w),
             'imu': imu(t, th, w), 'mo': tfrows(t, 0.0, 0.0), 'ob': tfrows(t, x, y, th),
             'counts': {}}
        dt = T0 + np.arange(0, n / 30.0, 0.1)
        D['dock'] = np.column_stack([dt, np.where(dt >= T0 + 5.0, 1.0, 0.0)])
        nl = int(n / 30.0)
        D['loc_t'] = T0 + np.arange(nl, dtype=float)
        D['loc_owner'] = np.array(['dock'] * 5 + ['dr'] * (int(self.TG) - 5) + ['gps'] * (nl - int(self.TG)))
        D['loc_anchor_age'] = np.full(nl, 0.1)
        D['loc_heading_age'] = np.full(nl, 0.1)
        return D, t - T0 - self.TG                 # cas od prevzeti

    @staticmethod
    def _ramp(ts, a, b, size):
        return size * np.clip((ts - a) / (b - a), 0.0, 1.0)

    def test_smooth_takeover_equals_needed(self):
        D, ts = self._run()
        D['mo'] = tfrows(ts + T0 + self.TG, self._ramp(ts, 0.5, 2.5, 0.06), 0.0)   # 6 cm za 2 s
        R = ns.score(D)
        m = R['metrics']
        self.assertAlmostEqual(m['takeover_needed_cm'], 6.0, delta=0.01)
        self.assertAlmostEqual(m['takeover_path_map_cm'], 6.0, delta=0.01)
        self.assertAlmostEqual(m['takeover_yaw_wobble_deg'], 0.0, delta=1e-6)
        self.assertAlmostEqual(m['takeover_wander_med'], 6.0, delta=0.01)
        self.assertAlmostEqual(m['takeover_wander_max'], m['takeover_wander_med'])
        self.assertEqual(len(R['detail']['takeover_wander']), 1)
        self.assertAlmostEqual(R['detail']['takeover_wander'][0]['t'], self.TG, delta=0.01)

    def test_zigzag_is_larger(self):
        D, ts = self._run()
        # kotva v odom: +9 cm skokem a zpet; georef v map->odom: 6 cm; cisty posun 6 cm
        oy = 0.09 * ((ts > 0.52) & (ts <= 2.02))
        D['ob'] = tfrows(ts + T0 + self.TG, 0.0, oy)
        D['ekf'] = odo(ts + T0 + self.TG, 0.0, oy, v=0.0)
        D['mo'] = tfrows(ts + T0 + self.TG, self._ramp(ts, 3.0, 5.0, 0.06), 0.0)
        m = ns.score(D)['metrics']
        self.assertAlmostEqual(m['takeover_needed_cm'], 6.0, delta=0.01)
        self.assertAlmostEqual(m['takeover_path_map_cm'], 24.0, delta=0.01)
        self.assertAlmostEqual(m['takeover_wander_med'], 24.0, delta=0.01)

    def test_yaw_wobble_adds_half_cm_per_deg(self):
        D, ts = self._run()
        # kurz v mape 4x preskoci o 1 st (po 1 s), poloha se nehne
        k = np.clip(np.floor(ts - 0.27) + 1, 0, 4)
        yaw = np.radians(1.0) * (k % 2)
        D['ob'] = tfrows(ts + T0 + self.TG, 0.0, 0.0, yaw)
        m = ns.score(D)['metrics']
        self.assertAlmostEqual(m['takeover_yaw_wobble_deg'], 4.0, delta=0.01)
        self.assertAlmostEqual(m['takeover_path_map_cm'], 0.0, delta=1e-6)
        self.assertAlmostEqual(m['takeover_wander_med'], 2.0, delta=0.01)

    def test_real_motion_is_subtracted(self):
        # robot behem prevzeti jede 0,3 m/s a otaci se: ujede 3,3 m, obraz mimo kola 0
        D, ts = self._run(v=0.3, w=0.08)
        m = ns.score(D)['metrics']
        self.assertLess(m['takeover_path_map_cm'], 0.01)
        self.assertLess(m['takeover_needed_cm'], 0.6)      # zbytek = krok integrace oblouku
        self.assertLess(m['takeover_yaw_wobble_deg'], 0.01)
        # stejna jizda + oprava 6 cm v map->odom: M = oprava, jizda se odecetla
        D['mo'] = tfrows(ts + T0 + self.TG, 0.0, self._ramp(ts, 0.5, 2.5, 0.06))
        m = ns.score(D)['metrics']
        self.assertAlmostEqual(m['takeover_path_map_cm'], 6.0, delta=0.1)
        self.assertAlmostEqual(m['takeover_wander_med'], 6.0, delta=0.1)

    def test_turning_robot_yaw_not_scored(self):
        # pri otaceni (|w| > 0,1 rad/s) se kolisani kurzu nehodnoti
        D, ts = self._run(v=0.0, w=0.3)
        ob = D['ob'].copy()
        ob[:, 5] += np.radians(1.0) * (np.floor(ts) % 2)
        D['ob'] = ob
        m = ns.score(D)['metrics']
        self.assertEqual(m['takeover_yaw_wobble_deg'], 0.0)

    def test_two_undocks_median_and_max(self):
        D, ts = self._run(n=3600)                          # 120 s
        dt = D['dock'][:, 0] - T0
        D['dock'][:, 1] = np.where((dt < 5) | ((dt >= 60) & (dt < 70)), 0.0, 1.0)
        D['loc_owner'] = np.array(['dock'] * 5 + ['dr'] * 20 + ['gps'] * 35 + ['dock'] * 10
                                  + ['dr'] * 15 + ['gps'] * 35)          # prevzeti v 25 a 85 s
        mx = self._ramp(ts, 0.5, 2.5, 0.06) + self._ramp(ts, 60.5, 62.5, 0.10)
        D['mo'] = tfrows(ts + T0 + self.TG, mx, 0.0)
        R = ns.score(D)
        m = R['metrics']
        self.assertEqual([round(x['wander'], 2) for x in R['detail']['takeover_wander']], [6.0, 10.0])
        self.assertAlmostEqual(m['takeover_wander_med'], 8.0, delta=0.01)
        self.assertAlmostEqual(m['takeover_wander_max'], 10.0, delta=0.01)

    def test_notes_when_not_measurable(self):
        keys = ['takeover_wander_med', 'takeover_wander_max', 'takeover_path_map_cm',
                'takeover_needed_cm', 'takeover_yaw_wobble_deg']
        D, ts = self._run(n=900)                           # nahravka konci 5 s po prevzeti
        R = ns.score(D)
        for k in keys:
            self.assertIsNone(R['metrics'][k])
            self.assertIn('okno prevzeti', R['notes'][k])
        D, ts = self._run()
        D['loc_owner'] = np.array(['dock'] * 5 + ['dr'] * 55)       # bez prevzeti
        R = ns.score(D)
        self.assertIsNone(R['metrics']['takeover_wander_med'])
        self.assertIn('nenastalo prevzeti', R['notes']['takeover_wander_med'])
        D, ts = self._run()
        D['dock'][:, 1] = 3.0                              # bez vyjezdu
        R = ns.score(D)
        self.assertIn('neni vyjezd', R['notes']['takeover_wander_max'])
        del D['dock']
        R = ns.score(D)
        self.assertIn(ns.TOPICS['dock'], R['notes']['takeover_wander_max'])

    def test_table_has_section(self):
        D, ts = self._run()
        buf = io.StringIO()
        ns.print_table(ns._jsonable(dict(ns.score(D), bag='x')), buf)
        self.assertIn('C3. Pohyblive prevzeti', buf.getvalue())
        self.assertIn('M = draha + 0,5 x kolisani kurzu, median', buf.getvalue())


class Missing(unittest.TestCase):
    def test_missing_topics_are_none_not_zero(self):
        D, *_ = base_extract()
        R = ns.score(D)
        m, notes = R['metrics'], R['notes']
        for k in ('owner_changes', 'takeover_gap_med_cm', 'gps_gap_p95_cm', 'dock_jump_cm',
                  'to_rtk_fixed_s', 'anchor_age_max_s', 'no_progress_n', 'rewindow_n',
                  'mbf_fail_n', 'yaw_handover_s', 'gps_standing_yaw_std_deg'):
            self.assertIsNone(m[k], k)
            self.assertIn(ns.NA, notes[k], k)

    def test_every_spec_key_is_reported(self):
        D, *_ = base_extract()
        m = ns.score(D)['metrics']
        for key, label, _, _ in ns.SPEC:
            if label is not None:
                self.assertIn(key, m, key)

    def test_no_tf_no_wheel(self):
        R = ns.score({'counts': {}})
        self.assertTrue(all(v is None for v in R['metrics'].values()))

    def test_table_prints_not_in_recording(self):
        D, *_ = base_extract()
        R = ns._jsonable(ns.score(D))
        out = io.StringIO()
        ns.print_table(R, out)
        txt = out.getvalue()
        self.assertIn(ns.NA, txt)
        line = [l for l in txt.splitlines() if 'no goal progress' in l][0]
        self.assertNotRegex(line, r'\s0\s*$')

    def test_rosout_counts(self):
        D, *_ = base_extract()
        D['rosout_text'] = np.array(['[EXE] no goal progress 8.1s -> re-window from current pos',
                                     'x', '[EXE] no goal progress 6.0s -> recovery/detour'])
        m = ns.score(D)['metrics']
        self.assertEqual(m['no_progress_n'], 2)
        self.assertEqual(m['rewindow_n'], 1)

    def test_z_drift_and_tf_gap(self):
        D, t, x, y = base_extract()
        D['ekf'] = odo(t, x, y, z=np.linspace(0.0, 0.5, len(t)))
        t2 = t.copy()
        t2[400:] += 1.5                       # mezera 1,5 s v /tf
        D['mo'] = tfrows(t2, 0.0, 0.0)
        D['ob'] = tfrows(t2, x, y)
        m = ns.score(D)['metrics']
        self.assertAlmostEqual(m['z_drift_m'], 0.5, places=6)
        self.assertAlmostEqual(m['tf_gap_max_s'], 1.5 + 1 / 30.0, places=3)


class Compare(unittest.TestCase):
    def test_compare_marks_direction(self):
        D, *_ = base_extract()
        A = ns._jsonable(ns.score(D))
        B = json.loads(json.dumps(A))
        A['metrics']['mb_n_gt1cm'], B['metrics']['mb_n_gt1cm'] = 100, 20
        A['metrics']['dock_jump_cm'], B['metrics']['dock_jump_cm'] = 5.0, 19.0
        A['metrics']['z_drift_m'], B['metrics']['z_drift_m'] = -0.5, 0.1     # |z| mensi = lepsi
        A['metrics']['tf_gap_max_s'], B['metrics']['tf_gap_max_s'] = 0.050, 0.052   # v toleranci
        out = io.StringIO()
        up, dn = ns.compare(A, B, out)
        self.assertEqual((up, dn), (2, 1))
        txt = out.getvalue()
        self.assertIn('ZLEPSENI', [l for l in txt.splitlines() if 'map->base: kroku > 1 cm' in l][0])
        self.assertIn('ZHORSENI', [l for l in txt.splitlines() if 'skok polohy za tik' in l][0])
        self.assertIn('nelze porovnat', txt)


class ClockShift(unittest.TestCase):
    """Oddil C2: mezera EKF - GPS s razitky opravenymi o posun hodin, v ramu odom."""
    OFF = -0.68          # systemove hodiny za UTC

    def _run(self, off=OFF, lag_m=0.0, side_m=0.0, mo_drift=0.0, with_pvt=True, back=True):
        # jizda tam (kurz 0) a zpet (kurz pi), 0,3 m/s; GPS 10 Hz s razitkem UTC
        n, v = 1800, 0.3
        t = T0 + np.arange(n) / 30.0
        h = n // 2 if back else n
        yaw = np.where(np.arange(n) < h, 0.0, math.pi)
        s_ = v / 30.0
        x = np.concatenate([[0.0], np.cumsum(np.cos(yaw[:-1]) * s_)])
        y = np.zeros(n)
        D = {'wheel': odo(t, x, y, yaw, v=v), 'ekf': odo(t, x, y, yaw, v=v),
             'odom': odo(t, x, y, yaw, v=v), 'imu': imu(t, yaw),
             'mo': tfrows(t, np.linspace(0.0, mo_drift, n), 0.0), 'ob': tfrows(t, x, y, yaw),
             'counts': {}}
        tg = t[::3]
        # GPS = EKF posunute o lag_m dozadu podel drahy a side_m vpravo
        gx = x[::3] - lag_m * np.cos(yaw[::3]) + side_m * np.sin(yaw[::3])
        gy = y[::3] - lag_m * np.sin(yaw[::3]) - side_m * np.cos(yaw[::3])
        utc = tg - off
        D['gps'] = np.column_stack([tg + 0.02, utc, gx, gy, np.zeros(len(tg))])
        if with_pvt:
            D['pvt'] = np.column_stack([tg + 0.01, np.full(len(tg), 3.0), np.full(len(tg), 2.0),
                                        np.full(len(tg), 0.01), utc])   # dodani 0,01 s
        lt = T0 + np.arange(0, n / 30.0, 0.1)
        D['loc_t'], D['loc_owner'] = lt, np.array(['gps'] * len(lt))
        D['loc_anchor_age'] = np.full(len(lt), 0.2)
        D['loc_heading_age'] = np.full(len(lt), 0.2)
        return ns.score(D)

    def test_old_metric_is_biased_new_is_not(self):
        m = self._run()['metrics']
        # stara metrika: posun x rychlost = 0,68 * 0,3 = 20 cm
        self.assertGreater(m['gps_gap_med_cm'], 15.0)
        self.assertAlmostEqual(m['gps_stamp_offset_s'], self.OFF + 0.01, places=2)
        self.assertLess(m['gps_gap_odom_med_cm'], 0.5)
        self.assertLess(abs(m['gps_along_med_cm']), 0.5)
        self.assertIsNone(m['gps_gap_fit_dt_s'])        # stala rychlost: primku nelze urcit

    def test_fit_recovers_residual_time_shift(self):
        # promenna rychlost, GPS razitka o 0,1 s jinde, nez rika oprava
        t = np.arange(0.0, 60.0, 1 / 30.0)
        v = 0.3 + 0.2 * np.sin(0.5 * t)
        x = np.concatenate([[0.0], np.cumsum(v[:-1] / 30.0)])
        ekf = odo(t, x, 0.0, 0.0, v=v)
        tg = t[30:-30:3]
        gps = np.column_stack([tg, tg, np.interp(tg - 0.1, t, x), 0 * tg, 0 * tg])
        G = ns.gps_gap_odom(ekf, gps, 0.0, [(0.0, 60.0)])
        self.assertAlmostEqual(ns.gap_odom_stats(G)['gps_gap_fit_dt_s'], 0.1, delta=0.01)
        G = ns.gps_gap_odom(ekf, gps, -0.1, [(0.0, 60.0)])
        self.assertLess(ns.gap_odom_stats(G)['gps_gap_odom_p95_cm'], 0.2)

    def test_old_metric_note_and_clean_clock(self):
        R = self._run()
        self.assertIn('zkreslene posunem hodin', R['notes']['gps_gap_med_cm'])
        R = self._run(off=0.0)
        self.assertNotIn('gps_gap_med_cm', R['notes'])
        self.assertLess(R['metrics']['gps_gap_odom_med_cm'], 0.5)

    def test_along_sign_same_both_directions(self):
        # EKF 4 cm pred GPS v obou smerech (chyba spojena s robotem)
        m = self._run(lag_m=0.04)['metrics']
        self.assertAlmostEqual(m['gps_along_out_med_cm'], 4.0, delta=0.5)
        self.assertAlmostEqual(m['gps_along_back_med_cm'], 4.0, delta=0.5)
        self.assertAlmostEqual(m['gps_along_med_cm'], 4.0, delta=0.5)
        self.assertLess(abs(m['gps_cross_med_cm']), 0.3)

    def test_cross_component_and_sign(self):
        m = self._run(side_m=0.03)['metrics']      # GPS vpravo => EKF vlevo => +
        self.assertAlmostEqual(m['gps_cross_med_cm'], 3.0, delta=0.3)
        self.assertLess(abs(m['gps_along_med_cm']), 0.5)
        self.assertAlmostEqual(m['gps_gap_odom_med_cm'], 3.0, delta=0.3)

    def test_map_odom_drift_does_not_enter(self):
        m = self._run(mo_drift=1.0)['metrics']
        self.assertGreater(m['gps_gap_p95_cm'], 50.0)       # stara: artefakt map->odom
        self.assertLess(m['gps_gap_odom_p95_cm'], 0.5)

    def test_fixed_world_offset_flips_sign(self):
        # posun pevny ve svete (GPS o 5 cm v +x): tam EKF vzadu, zpet vpredu
        G = ns.gps_gap_odom(odo(np.arange(10.0), np.arange(10.0), 0.0, 0.0),
                            np.column_stack([np.arange(1.0, 9.0), np.arange(1.0, 9.0),
                                             np.arange(1.0, 9.0) + 0.05, np.zeros(8), np.zeros(8)]),
                            0.0, [(0.0, 20.0)])
        self.assertTrue(np.allclose(G['along'], -0.05))
        self.assertTrue(np.allclose(G['cross'], 0.0))

    def test_only_rtk_fixed_samples_count(self):
        t = np.arange(0.0, 60.0, 0.1)
        ekf = odo(t, 0.3 * t, 0.0, 0.0)
        gps = np.column_stack([t, t, 0.3 * t, np.where(t >= 40, 0.4, 0.0), 0 * t])   # float: 40 cm bokem
        pvt = np.column_stack([t, 3 + 0 * t, np.where(t >= 40, 1.0, 2.0), 0 * t, t])
        M = ns.gap_odom_stats(ns.gps_gap_odom(ekf, gps, 0.0, [(0.0, 60.0)], pvt))
        self.assertLess(M['gps_gap_odom_p95_cm'], 0.1)
        self.assertAlmostEqual(M['gps_gap_rtk_fixed_pct'], 100 * 400 / 600.0, delta=1.0)
        M = ns.gap_odom_stats(ns.gps_gap_odom(ekf, gps, 0.0, [(0.0, 60.0)]))
        self.assertGreater(M['gps_gap_odom_p95_cm'], 30.0)

    def test_fallback_without_navpvt_is_noted(self):
        R = self._run(with_pvt=False)
        self.assertEqual(R['meta']['gps_stamp_offset_src'], 'gps')
        self.assertIn('navsatu', R['notes']['gps_stamp_offset_s'])
        self.assertLess(R['metrics']['gps_gap_odom_med_cm'], 1.0)   # 0,02 s * 0,3 m/s

    def test_rolling_offset_follows_step_and_drift(self):
        t = np.arange(0.0, 600.0, 0.1)
        utc = t + 0.5 + 0.001 * t                       # drift 1 ms/s
        utc[t >= 300] -= 0.3                            # skok hodin
        pvt = np.column_stack([t, t * 0, t * 0, t * 0, utc])
        off, src = ns.stamp_offset(np.array([100.0, 250.0, 400.0, 550.0]), pvt)
        self.assertEqual(src, 'navpvt')
        want = -(0.5 + 0.001 * np.array([100.0, 250.0, 400.0, 550.0])) + np.array([0, 0, 0.3, 0.3])
        self.assertTrue(np.allclose(off, want, atol=0.02), (off, want))

    def test_invalid_utc_is_skipped(self):
        self.assertTrue(math.isnan(ns.pvt_utc(2026, 11, 16, 7, 0, 0, 0, valid=0)))
        self.assertAlmostEqual(ns.pvt_utc(2026, 11, 16, 7, 0, 1, -500000000, valid=7),
                               ns.pvt_utc(2026, 11, 16, 7, 0, 0, 500000000, valid=3))
        pvt = np.column_stack([np.arange(5.0), np.zeros(5), np.zeros(5), np.zeros(5),
                               np.full(5, np.nan)])
        self.assertEqual(ns.stamp_offset(np.array([1.0]), pvt), (None, None))

    def test_buffered_share_from_loc_status(self):
        D, t, x, y = base_extract()
        lt = t[::3]
        D['loc_t'], D['loc_owner'] = lt, np.array(['dr'] * 50 + ['gps'] * (len(lt) - 50))
        D['loc_anchor_age'] = np.full(len(lt), 0.2)
        D['loc_heading_age'] = np.full(len(lt), 0.2)
        bs = np.full(len(lt), 0.98)
        bs[:50] = 0.1                       # mimo owner=gps se nepocita
        bs[60:64] = np.nan
        D['loc_buffered_share'] = bs
        m = ns.score(D)['metrics']
        self.assertAlmostEqual(m['anchor_buffered_share_med_pct'], 98.0)
        R = ns.score(base_extract()[0])
        self.assertIsNone(R['metrics']['anchor_buffered_share_med_pct'])

    def test_compare_with_older_json_works(self):
        A = ns._jsonable(self._run())
        old = json.loads(json.dumps(A))
        old['version'] = 2
        for k in list(old['metrics']):
            if '_odom_' in k or k.startswith(('gps_along', 'gps_cross', 'gps_stamp', 'gps_gap_fit',
                                              'gps_gap_standing', 'anchor_buffered')):
                del old['metrics'][k]
        buf = io.StringIO()
        ns.compare(old, A, out=buf)
        self.assertIn('nelze porovnat', buf.getvalue())
        self.assertIn('(stare*)', buf.getvalue())


class RawParsers(unittest.TestCase):
    @staticmethod
    def _hdr(stamp, frame):
        sec = int(stamp)
        return struct.pack('<III', 7, sec, int(round((stamp - sec) * 1e9))) + \
            struct.pack('<I', len(frame)) + frame

    def test_parse_odom(self):
        q = (0.0, 0.0, math.sin(0.25), math.cos(0.25))      # yaw 0,5 rad
        buf = self._hdr(12.5, b'odom') + struct.pack('<I', 9) + b'base_link' + \
            struct.pack('<7d', 1.0, 2.0, 3.0, *q) + b'\0' * 288 + \
            struct.pack('<6d', 0.4, 0.0, 0.0, 0.0, 0.0, -0.2) + b'\0' * 288
        st, x, y, z, yaw, vx, vy, wz = ns.parse_odom(buf)
        self.assertAlmostEqual(st, 12.5)
        self.assertEqual((x, y, z, vx, vy, wz), (1.0, 2.0, 3.0, 0.4, 0.0, -0.2))
        self.assertAlmostEqual(yaw, 0.5)

    def test_parse_imu(self):
        q = (0.0, 0.0, math.sin(-0.5), math.cos(-0.5))
        buf = self._hdr(3.25, b'imu_link') + struct.pack('<4d', *q) + b'\0' * 72 + \
            struct.pack('<3d', 0.0, 0.0, 0.7) + b'\0' * 72 + b'\0' * 96
        st, yaw, wz = ns.parse_imu(buf)
        self.assertAlmostEqual(st, 3.25)
        self.assertAlmostEqual(yaw, -1.0)
        self.assertEqual(wz, 0.7)

    def test_parse_tf(self):
        one = lambda p, c, x: self._hdr(5.0, p) + struct.pack('<I', len(c)) + c + \
            struct.pack('<7d', x, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0)
        buf = struct.pack('<I', 2) + one(b'map', b'odom', 1.5) + one(b'/odom', b'base_link', 2.5)
        out = ns.parse_tf(buf)
        self.assertEqual([(o[0], o[1], o[3]) for o in out],
                         [(b'map', b'odom', 1.5), (b'odom', b'base_link', 2.5)])


if __name__ == '__main__':
    unittest.main(verbosity=1)
