#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""nav_score.py - skore jizdy z nahravky (rosbag), bez ROS mastera.

Pouziti
-------
    nav_score.py JIZDA.bag                  # tabulka na stdout
    nav_score.py JIZDA.bag --json           # JSON na stdout (misto tabulky)
    nav_score.py JIZDA.bag --json-out F     # tabulka na stdout + JSON do souboru F
    nav_score.py JIZDA.bag --cache DIR      # vytah z bagu ulozi/nacte z DIR/*.npz
    nav_score.py --compare PRED.json PO.json   # rozdily pred/po

Bag se cte jednou a jen potrebna temata (TOPICS). Metrika, pro kterou v bagu
chybi tema, ma hodnotu None a vypise se "neni v nahravce" (nikdy nula).

Pojmy
-----
tik         jedna zprava /tf, ve ktere jsou map->odom i odom->base_link se
            stejnym header.stamp (navi_transform je posila v jedne zprave, 30 Hz).
cas pozy    odom->base_link v /tf nese cas odeslani, ale poza je posledni zprava
            EKF. Kazdemu tiku se proto dohleda zprava /odometry/ekf_wheel_nav_odom
            (nebo /odometry/odom) se stejnou pozou a pouzije se JEJI header.stamp.
            Bez toho by se do skoku michalo v*33 ms z nesoubehu dvou 30Hz smycek.
kola        /mobile_base_controller/odom. Draha s(t) = integral rychlosti vx
            (stejny vstup, jaky dostava EKF); dl = s(t2) - s(t1) je ujeta draha
            mezi dvema casy pozy.
gyro        kurz z /bno085/imu (kdyz chybi, kurz z kol).
jizda       |v kol| > 0,03 m/s nebo |w kol| > 0,05 rad/s.
klid kurzu  |w kol| i |w gyra| < 0,1 rad/s. Kroky kurzu se pocitaji jen v klidu
            kurzu: IMU chodi ve shlucich, pri otaceni by rozdil proti gyru
            ukazoval casovy nesoubeh, ne skok lokalizace.
stani       |v kol| < 0,005 m/s a |w kol| < 0,005 rad/s.

Metriky (klic v JSON -> definice)
---------------------------------
A. Skoky pozy za jeden tik (to, co vidi planovac a mapa)
   Krok = zmena pozy mezi dvema sousednimi tiky MINUS pohyb podle kol
   (dl ve smeru kurzu) a gyra (zmena kurzu). Cisty odometricky pohyb = 0.
   mo_*  cast zpusobena zmenou map->odom: posun robota v mape pri odom->base
         drzenem na hodnote noveho tiku (obsahuje i otoceni map->odom kolem
         pocatku mapy), kurz = zmena yaw map->odom.
   ob_*  cast v odom->base: zmena odom->base minus kola/gyro (zasahy set_pose,
         zmeny diff_yaw).
   mb_*  celkem map->base_link (to, co vidi konzument).
   *_pos_p95_cm / *_yaw_p95_deg   95. percentil pres tiky ZA JIZDY
   *_pos_max_cm / *_yaw_max_deg   maximum pres vsechny tiky
   *_n_gt1cm, *_n_gt3cm           pocet tiku s krokem > 1 cm / > 3 cm (vse)
   *_n_yaw_gt03                   pocet tiku s krokem kurzu > 0,3 st. (vse)
   Kroky kurzu (vsechny tri udaje) jen z tiku v klidu kurzu.
B. Kroky EKF (/odometry/ekf_wheel_nav_odom, kazda zprava jednou, casy z hlavicky)
   ekf_steps_gt1cm_per_min, ekf_steps_gt3cm_per_min
         pocet kroku polohy EKF mimo kola > 1 cm / > 3 cm na minutu jizdy.
   ekf_corr_cm_per_m    soucet velikosti techto kroku za jizdy / ujeta draha.
   odom_zero_twist_pct  podil vzorku /odometry/odom, ktere pri jizde kol
         > 0,1 m/s hlasi vx mensi nez 20 % rychlosti kol - stopa po set_pose,
         ktery v EKF nuluje rychlosti.
C. Vlastnik pozy (/nav_tf/loc_status, pole owner)
   owner_changes            pocet zmen vlastnika
   owner_dr_gps_per_hour    pocet prepnuti mezi dr a gps (obema smery) za hodinu
   takeover_gap_med_cm, takeover_gap_max_cm
         mezera |fuze - GPS| v mape v posledni sekunde pred prevzetim (owner
         -> gps). Fuze = map->odom(t) o EKF(t); GPS = map->odom(ref) o
         /odometry/gps(t), kde ref je map->odom na konci te same epizody gps
         (georef je pak dorovnany a konstantni).
   takeover_settle_med_s, takeover_settle_max_s
         doba od prevzeti, nez mezera klesne pod 5 cm a vydrzi tam 1 s.
   gps_gap_med_cm, gps_gap_p95_cm
         tataz mezera pres vsechny vzorky GPS pri owner=gps.
   POZOR: tyto stare metriky (takeover_gap_*, takeover_settle_*, gps_gap_*)
   jsou ZKRESLENE PRI POSUNU HODIN: hledaji pozu EKF (systemovy cas) na
   razitku GPS (UTC prijimace) -> chyba = posun hodin x rychlost, a GPS
   prevadeji pres map->odom z konce epizody, ktery se behem epizody hybe.
   Zustavaji jen kvuli --compare se starsimi JSON. Platne jsou tyto:
C2. Mezera EKF - GPS s opravou razitek, v ramu odom
   Razitko vzorku /odometry/gps (UTC prijimace) se prevede na systemovy cas
   prictenim posunu hodin = klouzavy median (okno 60 s) rozdilu (cas bagu -
   UTC) zprav /gnss/navpvt; kdyz navpvt chybi, median (cas bagu - razitko)
   /odometry/gps (poznamka u metriky). Mezera = EKF(cas mereni) - GPS, obe
   v ramu odom (map->odom do ni nevstupuje; stejnou velikost ma mezera v
   mape s map->odom v case vzorku). Rozklad podle kurzu EKF: PODEL drahy
   (+ = EKF pred GPS) a PRICNE (+ = EKF vlevo od GPS).
   gps_stamp_offset_s       pouzity posun hodin (median); jen udaj
   Do gps_gap_* / gps_along_* / gps_cross_* jdou jen vzorky pri owner=gps,
   kdy prijimac hlasi RTK fix (NAV-PVT carrSoln == 2): bez fixu (float u
   garaze) je desitky cm mimo GPS, ne EKF. gps_gap_rtk_fixed_pct = podil
   takovych vzorku. Prevzeti (takeover_*_odom_*) bere vsechny vzorky.
   gps_gap_odom_med_cm, gps_gap_odom_p95_cm   |mezera| pri owner=gps
   gps_gap_standing_med_cm  |mezera| ve stoje (|v| < 0,02 m/s)
   gps_along_med_cm         slozka podel, median se znamenkem, jizda vpred
                            (v > 0,1 m/s)
   gps_along_out_med_cm, gps_along_back_med_cm
         totez zvlast pro smer TAM (kurz do 45 st. od smeru prvnich 20 s
         jizdy vpred pri owner=gps) a ZPET (opacny smer). Stejne znamenko
         v obou smerech = chyba spojena s robotem (cas, zpozdeni); opacne
         = posun pevny v mape (datum, georeference).
   gps_along_abs_p95_cm     |podel| p95 za jizdy (|v| > 0,1 m/s)
   gps_cross_med_cm, gps_cross_abs_p95_cm   slozka pricne za jizdy
   gps_gap_fit_dt_s         zbytkovy casovy posun: smernice primky
         podel = dt * v + c pres vzorky za jizdy. Kontrola opravy razitek
         (0 = razitka sedi; obsahuje i zpozdeni dodani navpvt a kotvy).
   takeover_gap_odom_med_cm / _max_cm, takeover_settle_odom_med_s / _max_s
         jako takeover_gap_* / takeover_settle_*, ale z teto mezery.
C3. "Pohyblive" prevzeti polohy po vyjezdu z doku (takeover_wander, v4)
   Co vidi majitel na mape: obraz robota po prevzeti GPS cikcak ujede
   desitky cm, i kdyz staci opravit par cm, a robot sam se pritom nehybe.
   Prevzeti = prvni owner -> gps po opusteni doku (jedno na vyjezd). Okno
   <-1 s, +10 s> kolem nej. R(t) = soucet pres tiky [zmena polohy map->base
   minus draha kol dl ve smeru kurzu robota v mape] - pohyb obrazu, ktery
   kola nevysvetli (jizda i otaceni robota se odecte).
   takeover_path_map_cm     draha R(t): soucet |R(t + 0,1 s) - R(t)| pres
                            kroky vetsi nez 2 mm (median pres vyjezdy)
   takeover_needed_cm       skutecne potrebna oprava |R(konec) - R(zacatek)|
                            (median); idealni prevzeti ma drahu = opravu
   takeover_yaw_wobble_deg  kolisani kurzu: soucet |zmen H| po 0,5 s, H =
                            yaw(map->base) - gyro, jen kroky v klidu kurzu
                            (median); pri otaceni se kurz nehodnoti (0)
   takeover_wander_med, takeover_wander_max
         jedno cislo na vyjezd M = draha [cm] + 0,5 * kolisani kurzu [st]
         (0,5 cm/st = roh robota na rameni 0,3 m); median a maximum pres
         vyjezdy. Jednotlive vyjezdy: detail.takeover_wander.
   v5: PRVNI VYJEZD PO STARTU UZLU neni vypovidajici (navsat teprve bere
   datum, EKF je na GPS a kotva nema co dotahovat; serie 9: 7-17 cm bez
   ohledu na vypinace). Pozna se tak, ze mezi opustenim doku a prevzetim
   neprisla zadna zprava /odometry/gps (kdyz tema v nahravce chybi: |yaw
   map->odom| v doku > 5 st.) - v detailu cold = true. Stare klice vyse
   berou vsechny vyjezdy jako dosud.
   takeover_wander_warm_med, takeover_wander_warm_max
         M jen z vyjezdu, ktere nejsou prvni po startu uzlu
   takeover_warm_n          pocet takovych vyjezdu (jen udaj)
   takeover_step_max_cm     nejvetsi pohyb obrazu za 0,1 s pri prevzeti
                            (max pres vsechny vyjezdy) - skok, ktery je videt
   takeover_step_warm_max_cm   totez bez prvniho vyjezdu po startu uzlu
D. Predani kurzu po vyjezdu (dok -> GNSS)
   Okno predani = souvisly usek po opusteni doku, kdy se yaw map->odom meni
   (mezery mezi zmenami <= 5 s) a konci presnou nulou.
   yaw_handover_s           delka okna (max pres vyjezdy)
   yaw_handover_turn_deg    o kolik se map->odom behem okna otocil (max)
   yaw_handover_excess_deg  nadbytecne otaceni kurzu robota v mape: soucet
         |zmen H| minus |H(konec) - H(zacatek)|, H = yaw(map->base) - gyro,
         okno prodlouzene o 3 s (EKF dobiha), jen tiky v klidu kurzu.
         0 = plynule predani. (max)
   yaw_handover_max_step_deg  nejvetsi zmena H za jeden tik v okne (max)
   gps_standing_yaw_std_deg, gps_standing_yaw_p2p_deg
         kolisani kurzu robota v mape, kdyz robot stoji (useky stani >= 2 s)
         a owner=gps: odchylka H od prumeru useku - smerodatna odchylka pres
         vsechny useky a nejvetsi rozkmit (max - min) v jednom useku.
   gps_standing_pos_p95_cm
         totez pro polohu map->base: p95 vzdalenosti od medianu useku.
E. Zadokovani
   dock_jump_cm, dock_jump_deg   nejvetsi krok map->odom casti (mo) za jeden
         tik v okne <-1 s, +6 s> kolem prechodu dock_status -> 0 (max pres
         zadokovani); dock_shift_cm = celkovy posun robota v mape zpusobeny
         map->odom v tomtez okne.
F. Casy po opusteni doku (t0 = dock_status opusti 0); u vice vyjezdu max
   to_rtk_fixed_s    do prvniho RTK fixu prijimace polohy (NAV-PVT carrSoln
                     == 2), ktery vydrzi aspon 10 s (kratke fixy pod
                     strechou doku se nepocitaji)
   to_owner_gps_s    do owner=gps
   to_undocked_s     do dock_status == 3 (vyjezd dokoncen, mise muze jet)
   to_depart_s       do prvniho pohybu po dock_status == 3
   undock_standing_s stani (kola v klidu) mezi t0 a dock_status == 3
G. Kotvy (loc_status pri owner=gps)
   anchor_age_p95_s / _max_s, heading_anchor_age_p95_s / _max_s
   anchor_freezes, heading_anchor_freezes   pocet epizod se starim > 5 s
   anchor_fallback_pct   podil vzorku loc_status pri owner=gps, kdy polohu
         nepsala prubezna kotva, ale nouzovy set_pose na posledni zpravu GPS
         (10x za sekundu). Z pole anchor_mode (navi_transform s parametrem
         anchor_mode_report); kde pole chybi, plati vzorek s anchor_age_s
         null (kotva se od startu uzlu ani jednou neposunula).
   anchor_starving_pct   podil vzorku loc_status pri owner=gps s priznakem
         anchor_starving (do bufferu kotvy se dostava mene nez polovina vzorku
         GPS; kotva pak bezi na starych vzorcich). Jen s polem v nahravce.
   gps_rx_minus_stamp_s  median (cas prijeti - header.stamp) /odometry/gps.
         Razitko je cas prijimace GPS, zbytek systemovy cas. Jen udaj.
   gps_buffered_est_pct  odhad podilu vzorku GPS, ktere se dostanou do bufferu
         kotvy bez opravy razitek (navi_transform bez gps_stamp_offset_track):
         vzorek se ulozi, kdyz v nekterem tiku 10 Hz, dokud je posledni zpravou
         /odometry/gps, uz prisla zprava /odometry/ekf_wheel_odom s razitkem
         >= razitko vzorku. Median pres 10 fazi tiku. Jen udaj; do priznaku
         vstupuje jen u nahravek bez pole anchor_mode.
   anchor_buffered_share_med_pct, anchor_buffered_share_p05_pct
         skutecny podil vzorku GPS ulozenych do bufferu kotvy: pole
         anchor_buffered_share z loc_status pri owner=gps (median a 5.
         percentil). Jen s polem v nahravce (anchor_mode_report). Nahrazuje
         odhad gps_buffered_est_pct, ktery se zapnutym gps_stamp_offset_track
         nic nerika (pocita s neopravenymi razitky).
   Priznak pos_metrics_invalid (vedle metrics; True/False/None = nelze urcit):
   jizda bezela z vetsi casti v nouzovem rezimu kotvy nebo kotva hladovela,
   takze kroky EKF, mezera fuze - GPS a kolisani polohy meri ten rezim, ne
   upravu, ktera se porovnava. True, kdyz anchor_fallback_pct > 20 nebo
   anchor_starving_pct > 20, a u nahravek bez pole anchor_mode take kdyz
   gps_buffered_est_pct < 50 (stejny prah jako anchor_starving). Samotny
   posun razitek nerozhoduje. Duvod je v pos_metrics_invalid_why; --compare
   na priznak upozorni.
H. Ostatni
   z_drift_m         z EKF na konci minus na zacatku;  z_range_m = max - min
   tf_gap_max_s      nejvetsi mezera mezi tiky /tf (cas bagu); tf_gap_p99_s
   tf_stale_pct      podil tiku za jizdy, kdy /tf zopakoval predchozi pozu
   no_progress_n     pocet hlasek "no goal progress" v /rosout(_agg)
   rewindow_n        pocet hlasek "rewindow"/"re-window" v /rosout(_agg)
   mbf_fail_n        pocet vysledku exe_path s outcome != 0
"""
import argparse
import calendar
import json
import math
import os
import struct
import sys

import numpy as np

VERSION = 5
EXTRACT_VERSION = 3    # format vytahu (.npz v --cache); v4 a v5 pridaly jen metriky, vytah je stejny

TOPICS = {
    'tf': '/tf',
    'tf_static': '/tf_static',
    'loc': '/nav_tf/loc_status',
    'ekf': '/odometry/ekf_wheel_nav_odom',
    'odom': '/odometry/odom',
    'ekfw': '/odometry/ekf_wheel_odom',
    'wheel': '/mobile_base_controller/odom',
    'gps': '/odometry/gps',
    'pvt': '/gnss/navpvt',
    'hrel': '/gnss_heading/navrelposned',
    'imu': '/bno085/imu',
    'dock': '/dock_smach/dock_status',
    'res': '/move_base_flex/exe_path/result',
    'rosout': '/rosout',
    'rosout_agg': '/rosout_agg',
}

MOVE_V = 0.03          # m/s
MOVE_W = 0.05          # rad/s
SETTLE_M = 0.05        # m, mezera povazovana za dorovnanou
SETTLE_HOLD_S = 1.0
FREEZE_S = 5.0
WANDER_PRE_S = 1.0     # okno takeover_wander pred prevzetim
WANDER_POST_S = 10.0   # ... a po nem
WANDER_STEP_S = 0.1    # krok vzorkovani drahy obrazu
WANDER_MIN_M = 0.002   # mensi kroky se do drahy nepocitaji (sum)
WANDER_YAW_STEP_S = 0.5
WANDER_YAW_CM_PER_DEG = 0.5    # vaha kolisani kurzu v M (roh robota na rameni 0,3 m)
WANDER_COLD_YAW_DEG = 5.0      # bez /odometry/gps: |yaw map->odom| v doku nad timto = po startu uzlu
FALLBACK_INVALID_PCT = 20.0     # anchor_fallback_pct nad timto: polohove metriky neplati
BUFFERED_INVALID_PCT = 50.0     # gps_buffered_est_pct pod timto (stare nahravky): kotva hladovela
NA = 'neni v nahravce'


# --------------------------------------------------------------------------
# pomocne funkce
# --------------------------------------------------------------------------
def wrap(a):
    """Uhel do <-pi, pi)."""
    return (np.asarray(a) + np.pi) % (2 * np.pi) - np.pi


def yaw_q(x, y, z, w):
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _have(D, key, n=2):
    a = D.get(key)
    return a is not None and len(a) >= n


def pct(a, q):
    a = np.asarray(a, dtype=float)
    a = a[np.isfinite(a)]
    return float(np.percentile(a, q)) if len(a) else None


def interp_yaw(tq, t, yaw):
    """Linearni interpolace uhlu (pres unwrap)."""
    return np.interp(tq, t, np.unwrap(yaw))


def hold(tq, t, v):
    """Posledni hodnota v case <= tq (schodova funkce); pred prvni = prvni."""
    i = np.clip(np.searchsorted(t, tq, side='right') - 1, 0, len(t) - 1)
    return np.asarray(v)[i]


def wheel_path(wheel):
    """Draha kol = integral rychlosti vx (se znamenkem). wheel: N x 9
    (bag_t, stamp, x, y, z, yaw, vx, vy, wz). Vraci (stamp, s). Integruje se
    rychlost, ne poloha z kol: EKF dostava jen rychlost a poloha z kol je proti
    ni posunuta (klouzavy prumer v radici), coz by davalo falesne kroky."""
    dt = np.clip(np.diff(wheel[:, 1]), 0.0, 0.5)
    return wheel[:, 1], np.concatenate([[0.0], np.cumsum(wheel[:-1, 6] * dt)])


def calm_at(tq, wheel, imu=None, lim=0.1):
    """Maska 'klid kurzu' v casech tq: |w kol| i |w gyra| < lim."""
    ok = np.abs(np.interp(tq, wheel[:, 1], wheel[:, 8])) < lim
    if imu is not None and len(imu) > 2:
        ok &= np.abs(np.interp(tq, imu[:, 1], imu[:, 3])) < lim
    return ok


def standing_segments(t, still, min_s=2.0):
    """Souvisle useky still == True dlouhe aspon min_s -> [(i0, i1)] (i1 mimo)."""
    out, i, n = [], 0, len(t)
    while i < n:
        if still[i]:
            j = i
            while j < n and still[j]:
                j += 1
            if t[j - 1] - t[i] >= min_s:
                out.append((i, j))
            i = j
        else:
            i += 1
    return out


def standing_jitter(T, tau, wheel, yref_t, yref_yaw, sel):
    """Kolisani pozy v mape pri stani (oddil D). sel = maska tiku, ktere se
    berou (napr. owner=gps). Vraci dict nebo None, kdyz neni zadny usek."""
    v = np.interp(tau, wheel[:, 1], wheel[:, 6])
    w = np.interp(tau, wheel[:, 1], wheel[:, 8])
    still = (np.abs(v) < 0.005) & (np.abs(w) < 0.005) & sel
    segs = standing_segments(T['t'], still)
    if not segs:
        return None
    c, s = np.cos(T['myaw']), np.sin(T['myaw'])
    bx = T['mx'] + c * T['ox'] - s * T['oy']
    by = T['my'] + s * T['ox'] + c * T['oy']
    h = np.unwrap(T['myaw'] + T['oyaw']) - interp_yaw(tau, yref_t, yref_yaw)
    dev, p2p, dpos = [], 0.0, []
    for (a, b) in segs:
        d = h[a:b] - np.mean(h[a:b])
        dev.append(d)
        p2p = max(p2p, float(d.max() - d.min()))
        dpos.append(np.hypot(bx[a:b] - np.median(bx[a:b]), by[a:b] - np.median(by[a:b])))
    dev = np.concatenate(dev)
    return {'gps_standing_yaw_std_deg': math.degrees(float(np.sqrt(np.mean(dev ** 2)))),
            'gps_standing_yaw_p2p_deg': math.degrees(p2p),
            'gps_standing_pos_p95_cm': 100 * pct(np.concatenate(dpos), 95),
            'n_seg': len(segs), 'total_s': float(sum(T['t'][b - 1] - T['t'][a] for a, b in segs))}


def moving_at(tq, wheel):
    """Maska 'jizda' v casech tq (cas hlavicky kol)."""
    v = np.interp(tq, wheel[:, 1], wheel[:, 6])
    w = np.interp(tq, wheel[:, 1], wheel[:, 8])
    return (np.abs(v) > MOVE_V) | (np.abs(w) > MOVE_W)


# --------------------------------------------------------------------------
# metriky (ciste funkce nad poli numpy - testuje tools/test_nav_score.py)
# --------------------------------------------------------------------------
def pair_ticks(mo, ob):
    """Sparuje map->odom a odom->base do tiku. mo, ob: N x 6
    (bag_t, stamp, x, y, z, yaw). Paruje podle stejneho header.stamp; kdyz se
    stampy neshoduji (stare nahravky, dve zpravy), drzi se posledni map->odom.
    Vraci dict poli: t (cas bagu), stamp, mx, my, myaw, ox, oy, oz, oyaw."""
    _, im, io = np.intersect1d(mo[:, 1], ob[:, 1], return_indices=True)
    if len(io) >= 0.5 * len(ob):
        m, o = mo[im], ob[io]
    else:
        o = ob
        i = np.clip(np.searchsorted(mo[:, 0], ob[:, 0], side='right') - 1, 0, len(mo) - 1)
        m = mo[i]
    return dict(t=o[:, 0], stamp=o[:, 1], mx=m[:, 2], my=m[:, 3], myaw=m[:, 5],
                ox=o[:, 2], oy=o[:, 3], oz=o[:, 4], oyaw=o[:, 5])


def pose_times(T, ekf):
    """Cas pozy kazdeho tiku: header.stamp zpravy EKF se stejnou pozou (x, y
    presne shodne), hledano mezi zpravami kolem casu bagu tiku. Bez shody
    zustava stamp z /tf. Vraci (tau, podil_sparovanych)."""
    tau = T['stamp'].copy()
    if ekf is None or len(ekf) < 2:
        return tau, 0.0
    n = len(ekf)
    idx = np.searchsorted(ekf[:, 0], T['t'], side='right') - 1
    done = np.zeros(len(tau), dtype=bool)
    for off in (0, -1, 1, -2, 2, -3):
        j = np.clip(idx + off, 0, n - 1)
        hit = (~done) & (ekf[j, 2] == T['ox']) & (ekf[j, 3] == T['oy'])
        tau[hit] = ekf[j[hit], 1]
        done |= hit
    return tau, float(done.mean()) if len(done) else 0.0


def residual_steps(t, x, y, yaw, ref_t, ref_s, yref_t, yref_yaw):
    """Kroky pozy mimo odometrii. Pro kazdou dvojici sousednich vzorku vraci
    (pos, dyaw, ok): pos = |d(xy) - dl * smer kurzu| [m], dyaw = d(yaw) -
    d(referencni kurz) [rad], ok = krok ma rozumne dt (0 az 0,5 s; krok s
    dt == 0 je opakovana poza a je platny s nulovym pohybem)."""
    s = np.interp(t, ref_t, ref_s)
    dl = np.diff(s)
    yr = interp_yaw(t, yref_t, yref_yaw)
    ex = np.diff(x) - dl * np.cos(yaw[:-1])
    ey = np.diff(y) - dl * np.sin(yaw[:-1])
    dyaw = wrap(np.diff(yaw)) - np.diff(yr)
    dt = np.diff(t)
    ok = (dt >= 0) & (dt < 0.5)
    return np.hypot(ex, ey), wrap(dyaw), ok


def jump_parts(T, tau, ref_t, ref_s, yref_t, yref_yaw):
    """Rozklad skoku map->base za tik na cast map->odom (mo), odom->base (ob)
    a celek (mb). Vraci dict poli delky N-1: mo_pos, mo_yaw, ob_pos, ob_yaw,
    mb_pos, mb_yaw [m, rad], mo_dx, mo_dy [m], ok."""
    mx, my, myaw = T['mx'], T['my'], np.unwrap(T['myaw'])
    ox, oy, oyaw = T['ox'], T['oy'], np.unwrap(T['oyaw'])
    c, s = np.cos(myaw), np.sin(myaw)
    bx = mx + c * ox - s * oy
    by = my + s * ox + c * oy
    byaw = myaw + oyaw
    # cast map->odom: novy map->odom vs stary, oba aplikovane na NOVY odom->base
    bx_old = mx[:-1] + c[:-1] * ox[1:] - s[:-1] * oy[1:]
    by_old = my[:-1] + s[:-1] * ox[1:] + c[:-1] * oy[1:]
    mo_dx, mo_dy = bx[1:] - bx_old, by[1:] - by_old
    ob_pos, ob_yaw, ok = residual_steps(tau, ox, oy, oyaw, ref_t, ref_s, yref_t, yref_yaw)
    mb_pos, mb_yaw, _ = residual_steps(tau, bx, by, byaw, ref_t, ref_s, yref_t, yref_yaw)
    return dict(mo_pos=np.hypot(mo_dx, mo_dy), mo_yaw=wrap(np.diff(myaw)), mo_dx=mo_dx,
                mo_dy=mo_dy, ob_pos=ob_pos, ob_yaw=ob_yaw, mb_pos=mb_pos, mb_yaw=mb_yaw, ok=ok)


def jump_stats(pos, yaw, ok, moving, prefix, calm=None):
    """Souhrn skoku jedne casti (viz docstring modulu, oddil A). calm = maska
    tiku v klidu kurzu (kroky kurzu se berou jen z nich; None = vsechny)."""
    if calm is None:
        calm = np.ones(len(pos), dtype=bool)
    yaw, ymv = np.abs(yaw[ok & calm]), moving[ok & calm]
    pos, mv = pos[ok], moving[ok]
    return {
        prefix + '_pos_p95_cm': None if not mv.any() else 100 * pct(pos[mv], 95),
        prefix + '_pos_max_cm': 100 * float(pos.max()) if len(pos) else None,
        prefix + '_n_gt1cm': int((pos > 0.01).sum()),
        prefix + '_n_gt3cm': int((pos > 0.03).sum()),
        prefix + '_yaw_p95_deg': None if not ymv.any() else math.degrees(pct(yaw[ymv], 95)),
        prefix + '_yaw_max_deg': math.degrees(float(yaw.max())) if len(yaw) else None,
        prefix + '_n_yaw_gt03': int((yaw > math.radians(0.3)).sum()),
    }


def ekf_step_stats(ekf, wheel, yref_t, yref_yaw):
    """Oddil B: kroky EKF mimo kola na minutu jizdy. ekf, wheel: N x 9."""
    ref_t, ref_s = wheel_path(wheel)
    pos, _, ok = residual_steps(ekf[:, 1], ekf[:, 2], ekf[:, 3], np.unwrap(ekf[:, 5]),
                                ref_t, ref_s, yref_t, yref_yaw)
    mv = moving_at(ekf[:-1, 1], wheel) & ok
    dt = np.diff(ekf[:, 1])
    drive_min = float(dt[mv].sum()) / 60.0
    path = float(np.abs(np.diff(np.interp(ekf[:, 1], ref_t, ref_s)))[mv].sum())
    if drive_min <= 0:
        return {'ekf_steps_gt1cm_per_min': None, 'ekf_steps_gt3cm_per_min': None,
                'ekf_corr_cm_per_m': None}
    return {
        'ekf_steps_gt1cm_per_min': float((pos[mv] > 0.01).sum()) / drive_min,
        'ekf_steps_gt3cm_per_min': float((pos[mv] > 0.03).sum()) / drive_min,
        'ekf_corr_cm_per_m': 100 * float(pos[mv].sum()) / path if path > 0 else None,
    }


def zero_twist_pct(odom, wheel):
    """Podil vzorku odometrie s propadlou rychlosti [%]: |vx| < 20 % rychlosti
    kol, pocitano jen kdyz kola jedou > 0,1 m/s."""
    vw = np.interp(odom[:, 1], wheel[:, 1], wheel[:, 6])
    mv = np.abs(vw) > 0.1
    if not mv.any():
        return None
    zero = np.abs(odom[:, 6]) < 0.2 * np.abs(vw)
    return 100.0 * float((zero & mv).sum()) / float(mv.sum())


def owner_transitions(loc_t, owner):
    """Seznam zmen vlastnika [(t, z, na)]."""
    out = []
    for i in range(1, len(owner)):
        if owner[i] != owner[i - 1]:
            out.append((float(loc_t[i]), str(owner[i - 1]), str(owner[i])))
    return out


def owner_episodes(loc_t, owner, name):
    """Souvisle useky s danym vlastnikem [(t_od, t_do)]."""
    ep, start = [], None
    for i in range(len(owner)):
        if owner[i] == name and start is None:
            start = float(loc_t[i])
        elif owner[i] != name and start is not None:
            ep.append((start, float(loc_t[i])))
            start = None
    if start is not None:
        ep.append((start, float(loc_t[-1])))
    return ep


def gps_gap(T, ekf, gps, episodes):
    """Mezera |fuze - GPS| v mape v casech vzorku GPS (viz oddil C).
    Vraci (t_bag, gap_m, in_gps): vzorky do 20 s pred epizodou gps a v ni;
    in_gps = vzorek lezi v epizode."""
    tb, gap, ing = [], [], []
    for (a, b) in episodes:
        k = np.searchsorted(T['t'], b, side='right') - 1
        if k < 0:
            continue
        rx, ry, ryaw = T['mx'][k], T['my'][k], T['myaw'][k]
        m = (gps[:, 0] >= a - 20.0) & (gps[:, 0] <= b)
        g = gps[m]
        if not len(g):
            continue
        # fuze: map->odom(t) o EKF(cas mereni GPS)
        ex = np.interp(g[:, 1], ekf[:, 1], ekf[:, 2])
        ey = np.interp(g[:, 1], ekf[:, 1], ekf[:, 3])
        i = np.clip(np.searchsorted(T['t'], g[:, 0], side='right') - 1, 0, len(T['t']) - 1)
        c, s = np.cos(T['myaw'][i]), np.sin(T['myaw'][i])
        fx = T['mx'][i] + c * ex - s * ey
        fy = T['my'][i] + s * ex + c * ey
        gx = rx + math.cos(ryaw) * g[:, 2] - math.sin(ryaw) * g[:, 3]
        gy = ry + math.sin(ryaw) * g[:, 2] + math.cos(ryaw) * g[:, 3]
        tb.append(g[:, 0])
        gap.append(np.hypot(fx - gx, fy - gy))
        ing.append(g[:, 0] >= a)
    if not tb:
        return np.array([]), np.array([]), np.array([], dtype=bool)
    return np.concatenate(tb), np.concatenate(gap), np.concatenate(ing)


def rolling_median(t, v, tq, win_s=60.0):
    """Klouzavy median v(t) v okne +-win_s/2 kolem casu tq (t rostouci).
    Okno bez vzorku -> nejblizsi vzorek."""
    t, v, tq = np.asarray(t, dtype=float), np.asarray(v, dtype=float), np.atleast_1d(tq)
    a = np.searchsorted(t, tq - 0.5 * win_s, side='left')
    b = np.searchsorted(t, tq + 0.5 * win_s, side='right')
    out = np.empty(len(tq))
    for k in range(len(tq)):
        if b[k] > a[k]:
            out[k] = np.median(v[a[k]:b[k]])
        else:
            out[k] = v[int(np.clip(a[k], 0, len(v) - 1))]
    return out


def stamp_offset(tq, pvt=None, gps=None, win_s=60.0):
    """Posun hodin (systemovy cas - UTC prijimace) v casech bagu tq.
    pvt: N x >=5 (bag_t, ..., utc ve sloupci 4; NaN = neplatny cas); gps:
    /odometry/gps (bag_t, stamp, ...) jako nahrada. Vraci (pole, zdroj) nebo
    (None, None). Pocita se na mrizce po 5 s a linearne interpoluje."""
    src = None
    if pvt is not None and len(pvt) >= 3 and pvt.shape[1] >= 5:
        ok = np.isfinite(pvt[:, 4])
        if ok.sum() >= 3:
            t, d, src = pvt[ok, 0], pvt[ok, 0] - pvt[ok, 4], 'navpvt'
    if src is None and gps is not None and len(gps) >= 3:
        t, d, src = gps[:, 0], gps[:, 0] - gps[:, 1], 'gps'
    if src is None:
        return None, None
    grid = np.arange(t[0], t[-1] + 5.0, 5.0)
    return np.interp(tq, grid, rolling_median(t, d, grid, win_s)), src


def gps_gap_odom(ekf, gps, off, episodes, pvt=None):
    """Mezera EKF - GPS v ramu odom s opravenymi razitky (oddil C2).
    off = posun hodin pro kazdy vzorek gps (pole) nebo skalar. Vraci dict
    poli pres vzorky do 20 s pred epizodou gps a v ni: t (cas bagu), gap,
    along, cross [m], v [m/s], yaw [rad, kurz EKF], in_gps, fixed (prijimac
    mel v case prijeti vzorku RTK fix, carrSoln == 2; bez pvt vse True)."""
    off = np.broadcast_to(np.asarray(off, dtype=float), (len(gps),))
    sel = np.zeros(len(gps), dtype=bool)
    ing = np.zeros(len(gps), dtype=bool)
    for (a, b) in episodes:
        sel |= (gps[:, 0] >= a - 20.0) & (gps[:, 0] <= b)
        ing |= (gps[:, 0] >= a) & (gps[:, 0] <= b)
    g, ts = gps[sel], gps[sel, 1] + off[sel]
    inside = (ts >= ekf[0, 1]) & (ts <= ekf[-1, 1])
    g, ts, ing = g[inside], ts[inside], ing[sel][inside]
    dx = np.interp(ts, ekf[:, 1], ekf[:, 2]) - g[:, 2]
    dy = np.interp(ts, ekf[:, 1], ekf[:, 3]) - g[:, 3]
    yaw = np.interp(ts, ekf[:, 1], np.unwrap(ekf[:, 5]))
    c, s = np.cos(yaw), np.sin(yaw)
    fixed = np.ones(len(g), dtype=bool)
    if pvt is not None and len(pvt):
        fixed = hold(g[:, 0], pvt[:, 0], pvt[:, 2]) == 2
    return dict(t=g[:, 0], gap=np.hypot(dx, dy), along=dx * c + dy * s, cross=-dx * s + dy * c,
                v=np.interp(ts, ekf[:, 1], ekf[:, 6]), yaw=yaw, in_gps=ing, fixed=fixed)


def gap_odom_stats(G):
    """Souhrn oddilu C2 z vystupu gps_gap_odom (jen vzorky in_gps a fixed)."""
    keys = ['gps_gap_odom_med_cm', 'gps_gap_odom_p95_cm', 'gps_gap_standing_med_cm',
            'gps_along_med_cm', 'gps_along_out_med_cm', 'gps_along_back_med_cm',
            'gps_along_abs_p95_cm', 'gps_cross_med_cm', 'gps_cross_abs_p95_cm', 'gps_gap_fit_dt_s',
            'gps_gap_rtk_fixed_pct']
    M = {k: None for k in keys}
    m = G['in_gps']
    if not m.any():
        return M
    M['gps_gap_rtk_fixed_pct'] = 100.0 * float(G['fixed'][m].mean())
    m = m & G['fixed']
    if not m.any():
        return M
    gap, al, cr, v, yaw = G['gap'][m], G['along'][m], G['cross'][m], G['v'][m], G['yaw'][m]
    med = lambda a, k=1: 100 * float(np.median(a)) if len(a) >= k else None
    M['gps_gap_odom_med_cm'] = med(gap)
    M['gps_gap_odom_p95_cm'] = 100 * pct(gap, 95)
    M['gps_gap_standing_med_cm'] = med(gap[np.abs(v) < 0.02], 10)
    mv, fwd = np.abs(v) > 0.1, v > 0.1
    if mv.sum() >= 10:
        M['gps_along_abs_p95_cm'] = 100 * pct(np.abs(al[mv]), 95)
        M['gps_cross_med_cm'] = med(cr[mv])
        M['gps_cross_abs_p95_cm'] = 100 * pct(np.abs(cr[mv]), 95)
        if np.ptp(v[mv]) > 0.05:
            A = np.c_[v[mv], np.ones(int(mv.sum()))]
            M['gps_gap_fit_dt_s'] = float(np.linalg.lstsq(A, al[mv], rcond=None)[0][0])
    if fwd.sum() >= 10:
        M['gps_along_med_cm'] = med(al[fwd])
        first = np.flatnonzero(fwd)[:200]
        ref = math.atan2(float(np.mean(np.sin(yaw[first]))), float(np.mean(np.cos(yaw[first]))))
        d = np.cos(yaw - ref)
        M['gps_along_out_med_cm'] = med(al[fwd & (d > 0.7)], 20)
        M['gps_along_back_med_cm'] = med(al[fwd & (d < -0.7)], 20)
    return M


def takeover_stats(gt, gap, episodes):
    """Pro kazde prevzeti (zacatek epizody gps): mezera pred (median posledni
    1 s) a doba dorovnani (mezera < SETTLE_M po SETTLE_HOLD_S). Vraci seznam
    dictu {t, gap_cm, settle_s}; settle_s None = v epizode se nedorovnalo."""
    out = []
    for (a, b) in episodes:
        pre = gap[(gt >= a - 1.0) & (gt < a)]
        m = (gt >= a) & (gt <= b)
        t_in, g_in = gt[m], gap[m]
        settle = None
        below = g_in < SETTLE_M
        i = 0
        while i < len(g_in):
            if below[i]:
                j = i
                while j < len(g_in) and below[j]:
                    j += 1
                if t_in[j - 1] - t_in[i] >= SETTLE_HOLD_S or j == len(g_in):
                    settle = float(t_in[i] - a)
                    break
                i = j
            else:
                i += 1
        out.append({'t': a, 'gap_cm': 100 * float(np.median(pre)) if len(pre) else None,
                    'settle_s': settle})
    return out


def takeover_wander(T, tau, wheel, yref_t, yref_yaw, imu, tg):
    """Oddil C3 pro jedno prevzeti v case tg (cas bagu). Vraci dict {t,
    path_cm, needed_cm, yaw_wobble_deg, wander} nebo None, kdyz okno
    <tg - WANDER_PRE_S, tg + WANDER_POST_S> neni cele v nahravce."""
    t = T['t']
    a, b = tg - WANDER_PRE_S, tg + WANDER_POST_S
    if not len(t) or a < t[0] or b > t[-1]:
        return None
    i0 = max(int(np.searchsorted(t, a)) - 2, 0)
    i1 = min(int(np.searchsorted(t, b)) + 2, len(t))
    if i1 - i0 < 3:
        return None
    sl = slice(i0, i1)
    t, tq = t[sl], tau[sl]
    myaw = T['myaw'][sl]
    c, s = np.cos(myaw), np.sin(myaw)
    bx = T['mx'][sl] + c * T['ox'][sl] - s * T['oy'][sl]
    by = T['my'][sl] + s * T['ox'][sl] + c * T['oy'][sl]
    h = np.unwrap(myaw + T['oyaw'][sl])
    ref_t, ref_s = wheel_path(wheel)
    dl = np.diff(np.interp(tq, ref_t, ref_s))
    rx = np.concatenate([[0.0], np.cumsum(np.diff(bx) - dl * np.cos(h[1:]))])
    ry = np.concatenate([[0.0], np.cumsum(np.diff(by) - dl * np.sin(h[1:]))])
    n = int(round((WANDER_PRE_S + WANDER_POST_S) / WANDER_STEP_S))
    tb = a + WANDER_STEP_S * np.arange(n + 1)
    xb, yb = np.interp(tb, t, rx), np.interp(tb, t, ry)
    st = np.hypot(np.diff(xb), np.diff(yb))
    path = 100.0 * float(st[st > WANDER_MIN_M].sum())
    needed = 100.0 * math.hypot(xb[-1] - xb[0], yb[-1] - yb[0])
    hres = np.degrees(h - interp_yaw(tq, yref_t, yref_yaw))
    n = int(round((WANDER_PRE_S + WANDER_POST_S) / WANDER_YAW_STEP_S))
    th = a + WANDER_YAW_STEP_S * np.arange(n + 1)
    w = np.abs(np.interp(tq, wheel[:, 1], wheel[:, 8]))
    if imu is not None and len(imu) > 2:
        w = np.maximum(w, np.abs(np.interp(tq, imu[:, 1], imu[:, 3])))
    calm = np.interp(th, t, w) < 0.1
    dh = np.abs(np.diff(np.interp(th, t, hres)))
    wobble = float(dh[calm[1:] & calm[:-1]].sum())
    return {'t': float(tg), 'path_cm': path, 'needed_cm': needed, 'yaw_wobble_deg': wobble,
            'wander': path + WANDER_YAW_CM_PER_DEG * wobble,
            'step_max_cm': 100.0 * float(st.max()) if len(st) else 0.0}


def dock_edges(dock):
    """Hrany dock_status. dock: N x 2 (bag_t, hodnota). Vraci (vyjezdy, zadokovani)
    = casy prechodu 0 -> jine a jine -> 0."""
    v = dock[:, 1]
    ch = np.flatnonzero(np.diff(v) != 0)
    und = [float(dock[i + 1, 0]) for i in ch if v[i] == 0]
    dk = [float(dock[i + 1, 0]) for i in ch if v[i + 1] == 0]
    return und, dk


def yaw_handover(T, tau, yref_t, yref_yaw, t0, t_end, calm=None):
    """Oddil D pro jeden vyjezd (t0 = opusteni doku, t_end = konec hledani).
    Vraci dict nebo None, kdyz map->odom po vyjezdu natoceny nebyl / nedosel
    na nulu."""
    t, myaw = T['t'], T['myaw']
    i0 = int(np.searchsorted(t, t0))
    i1 = int(np.searchsorted(t, t_end))
    if i1 - i0 < 3 or myaw[i0] == 0.0:
        return None
    z = np.flatnonzero(myaw[i0:i1] == 0.0)
    if not len(z):
        return None
    iz = i0 + int(z[0])                       # prvni tik s presnou nulou
    ch = np.flatnonzero(np.diff(myaw[i0:iz + 1]) != 0) + i0 + 1   # tiky se zmenou
    if not len(ch):
        return None
    k = len(ch) - 1
    while k > 0 and t[ch[k]] - t[ch[k - 1]] <= 5.0:
        k -= 1
    ia = max(int(ch[k]) - 1, i0)
    ib = int(min(np.searchsorted(t, t[iz] + 3.0), len(t) - 1))
    h = np.unwrap(myaw[ia:ib + 1] + T['oyaw'][ia:ib + 1]) - interp_yaw(tau[ia:ib + 1], yref_t, yref_yaw)
    dh = np.diff(h)
    if calm is not None:
        dh = dh[calm[ia:ib] & calm[ia + 1:ib + 1]]
    out = {'t': float(t[ia]), 's': float(t[iz] - t[ia]),
           'turn_deg': abs(math.degrees(float(wrap(myaw[ia])))),
           'excess_deg': None, 'max_step_deg': None}
    if len(dh) >= 0.5 * (ib - ia):
        out['excess_deg'] = math.degrees(float(np.abs(dh).sum() - abs(dh.sum())))
        out['max_step_deg'] = math.degrees(float(np.abs(dh).max()))
    return out


def dock_jump(T, J, t_dock):
    """Oddil E pro jedno zadokovani. J = vystup jump_parts."""
    t = T['t'][1:]
    m = (t >= t_dock - 1.0) & (t <= t_dock + 6.0)
    if not m.any():
        return None
    return {'t': t_dock, 'jump_cm': 100 * float(J['mo_pos'][m].max()),
            'jump_deg': math.degrees(float(np.abs(J['mo_yaw'][m]).max())),
            'shift_cm': 100 * float(math.hypot(J['mo_dx'][m].sum(), J['mo_dy'][m].sum()))}


def first_sustained(t, cond, t0, hold_s):
    """Prvni cas >= t0, od ktereho cond plati aspon hold_s (nebo do konce dat)."""
    i = int(np.searchsorted(t, t0))
    n = len(t)
    while i < n:
        if cond[i]:
            j = i
            while j < n and cond[j]:
                j += 1
            if t[j - 1] - t[i] >= hold_s or j == n:
                return float(t[i])
            i = j
        else:
            i += 1
    return None


def undock_times(t0, t_next, dock, pvt, loc_t, owner, wheel):
    """Oddil F pro jeden vyjezd; t_next = dalsi zadokovani / konec nahravky."""
    out = {'t': t0, 'to_rtk_fixed_s': None, 'to_owner_gps_s': None, 'to_undocked_s': None,
           'to_depart_s': None, 'undock_standing_s': None}
    if pvt is not None and len(pvt):
        tf_ = first_sustained(pvt[:, 0], pvt[:, 2] == 2, t0, 10.0)
        if tf_ is not None and tf_ <= t_next:
            out['to_rtk_fixed_s'] = tf_ - t0
    if loc_t is not None and len(loc_t):
        m = (loc_t >= t0) & (loc_t <= t_next) & (np.asarray(owner) == 'gps')
        if m.any():
            out['to_owner_gps_s'] = float(loc_t[m][0] - t0)
    m3 = (dock[:, 0] >= t0) & (dock[:, 0] <= t_next) & (dock[:, 1] == 3)
    if m3.any():
        t3 = float(dock[m3][0, 0])
        out['to_undocked_s'] = t3 - t0
        if wheel is not None and len(wheel) > 2:
            wt = wheel[:, 0]
            mv = (np.abs(wheel[:, 6]) > MOVE_V) | (np.abs(wheel[:, 8]) > MOVE_W)
            seg = (wt >= t0) & (wt <= t3)
            dt = np.diff(wt, append=wt[-1])
            out['undock_standing_s'] = float(dt[seg & ~mv].sum())
            after = (wt >= t3) & (wt <= t_next) & mv
            if after.any():
                out['to_depart_s'] = float(wt[after][0] - t0)
    return out


def age_stats(age, sel, prefix):
    """Oddil G: p95, max a pocet epizod se starim > FREEZE_S pres vybrane vzorky."""
    a = np.asarray(age, dtype=float)
    ok = sel & np.isfinite(a)
    if not ok.any():
        return {prefix + '_age_p95_s': None, prefix + '_age_max_s': None, prefix + '_freezes': None}
    fr = (a > FREEZE_S) & ok
    n = int(fr[0]) + int((fr[1:] & ~fr[:-1]).sum())
    return {prefix + '_age_p95_s': pct(a[ok], 95), prefix + '_age_max_s': float(a[ok].max()),
            prefix + '_freezes': n}


def gps_buffered_est(gps, ekfw):
    """Odhad podilu vzorku /odometry/gps [%], ktere navi_transform bez opravy
    razitek ulozi do bufferu kotvy (viz docstring, gps_buffered_est_pct)."""
    if gps is None or ekfw is None or len(gps) < 3 or len(ekfw) < 3:
        return None
    shares = []
    for phi in np.arange(0.0, 0.1, 0.01):
        T = np.arange(gps[0, 0] + phi, gps[-1, 0], 0.1)
        if not len(T):
            return None
        newest = ekfw[np.clip(np.searchsorted(ekfw[:, 0], T, side='right') - 1, 0, len(ekfw) - 1), 1]
        gi = np.searchsorted(gps[:, 0], T, side='right') - 1
        ok = (gi >= 0) & (newest >= gps[np.clip(gi, 0, len(gps) - 1), 1])
        shares.append(len(np.unique(gi[ok])) / float(len(gps)))
    return 100.0 * float(np.median(shares))


def anchor_fallback(owner, mode, anchor_age, gps, starving=None, ekfw=None):
    """Oddil G: nouzovy rezim kotvy polohy. Vraci (metriky, priznak, duvod).
    owner, mode, anchor_age, starving = pole z loc_status (mode muze byt None
    nebo prazdne retezce = pole anchor_mode v nahravce neni; starving jen s
    nim); gps = /odometry/gps, ekfw = /odometry/ekf_wheel_odom."""
    M = {'anchor_fallback_pct': None, 'anchor_starving_pct': None, 'gps_rx_minus_stamp_s': None,
         'gps_buffered_est_pct': gps_buffered_est(gps, ekfw)}
    if gps is not None and len(gps) >= 3:
        M['gps_rx_minus_stamp_s'] = float(np.median(gps[:, 0] - gps[:, 1]))
    has_mode = False
    if owner is not None:
        sel = np.asarray(owner) == 'gps'
        fb = np.isnan(np.asarray(anchor_age, dtype=float)) if anchor_age is not None \
            else np.zeros(len(sel), dtype=bool)
        if mode is not None:
            m = np.asarray(mode)
            known = m != ''
            has_mode = bool((known & sel).any())
            fb = np.where(known, m == 'fallback_set_pose', fb)
            if has_mode and starving is not None:
                M['anchor_starving_pct'] = 100.0 * float(
                    np.asarray(starving, dtype=bool)[known & sel].mean())
        if sel.any():
            M['anchor_fallback_pct'] = 100.0 * float(fb[sel].mean())
    pct_fb, est = M['anchor_fallback_pct'], M['gps_buffered_est_pct']
    if pct_fb is not None and pct_fb > FALLBACK_INVALID_PCT:
        return M, True, 'kotva polohy %.0f %% casu owner=gps v nouzovem rezimu (set_pose na posledni GPS)' % pct_fb
    if M['anchor_starving_pct'] is not None and M['anchor_starving_pct'] > FALLBACK_INVALID_PCT:
        return M, True, ('kotva polohy %.0f %% casu owner=gps hladovi (vetsina vzorku GPS se '
                         'nedostane do jejiho bufferu)' % M['anchor_starving_pct'])
    if not has_mode and est is not None and est < BUFFERED_INVALID_PCT:
        return M, True, ('do bufferu kotvy polohy se podle razitek dostalo jen %.0f %% vzorku GPS '
                         '(prijeti - razitko %+.2f s): kotva hladovela'
                         % (est, M['gps_rx_minus_stamp_s']))
    if pct_fb is None and (est is None or has_mode):
        return M, None, 'nelze urcit (chybi %s nebo %s a %s)' % (
            TOPICS['loc'], TOPICS['gps'], TOPICS['ekfw'])
    return M, False, ''


def tf_gap_stats(T, tau, wheel):
    """Oddil H: mezery /tf a podil opakovanych poz za jizdy."""
    g = np.diff(T['t'])
    out = {'tf_gap_max_s': float(g.max()) if len(g) else None, 'tf_gap_p99_s': pct(g, 99),
           'tf_stale_pct': None}
    if wheel is not None and len(wheel) > 2 and len(g):
        mv = moving_at(tau[1:], wheel) & (np.abs(np.interp(tau[1:], wheel[:, 1], wheel[:, 6])) > 0.1)
        same = (np.diff(T['ox']) == 0) & (np.diff(T['oy']) == 0)
        if mv.any():
            out['tf_stale_pct'] = 100.0 * float((same & mv).sum()) / float(mv.sum())
    return out


# --------------------------------------------------------------------------
# skore z vytahu
# --------------------------------------------------------------------------
def score(D):
    """D = vytah z bagu (viz extract). Vraci dict {meta, metrics, notes, detail}."""
    M, notes, detail = {}, {}, {}

    def missing(keys, *topics):
        have = D.get('counts') or {}
        absent = [t for t in topics if t not in have]
        topics = absent or topics
        for k in keys:
            M[k] = None
            notes[k] = '%s (chybi %s)' % (NA, ', '.join(topics))

    wheel = D.get('wheel') if _have(D, 'wheel', 3) else None
    ekf = D.get('ekf') if _have(D, 'ekf', 3) else (D.get('odom') if _have(D, 'odom', 3) else None)
    imu = D.get('imu') if _have(D, 'imu', 3) else None
    if imu is not None:
        yref_t, yref_yaw, yref_src = imu[:, 1], imu[:, 2], 'gyro'
    elif wheel is not None:
        yref_t, yref_yaw, yref_src = wheel[:, 1], wheel[:, 5], 'kola'
    else:
        yref_t = yref_yaw = None
        yref_src = None
    has_tf = _have(D, 'mo', 3) and _have(D, 'ob', 3)
    T = tau = J = calm_t = None
    if has_tf:
        T = pair_ticks(D['mo'], D['ob'])
        tau, matched = pose_times(T, ekf)
    meta = {'yaw_ref': yref_src, 'topics': D.get('counts', {})}
    t_all = [a[:, 0] for k, a in D.items() if isinstance(a, np.ndarray) and a.ndim == 2 and len(a)]
    t_beg = min(float(a[0]) for a in t_all) if t_all else 0.0
    t_fin = max(float(a[-1]) for a in t_all) if t_all else 0.0
    meta['t0'] = t_beg
    meta['duration_s'] = t_fin - t_beg
    if wheel is not None:
        _, s = wheel_path(wheel)
        mvw = (np.abs(wheel[:, 6]) > MOVE_V) | (np.abs(wheel[:, 8]) > MOVE_W)
        meta['path_m'] = float(np.abs(np.diff(s)).sum())
        meta['drive_min'] = float(np.diff(wheel[:, 0], append=wheel[-1, 0])[mvw].sum()) / 60.0

    # A. skoky za tik
    keysA = [p + s for p in ('mo', 'ob', 'mb') for s in (
        '_pos_p95_cm', '_pos_max_cm', '_n_gt1cm', '_n_gt3cm', '_yaw_p95_deg', '_yaw_max_deg',
        '_n_yaw_gt03')]
    if has_tf and wheel is not None:
        meta['tf_pose_time_matched_pct'] = 100.0 * matched
        ref_t, ref_s = wheel_path(wheel)
        J = jump_parts(T, tau, ref_t, ref_s, yref_t, yref_yaw)
        mv = moving_at(tau[:-1], wheel)
        calm_t = calm_at(tau, wheel, imu)
        calm = calm_t[:-1] & calm_t[1:]
        ok_all = np.ones(len(J['ok']), dtype=bool)
        M.update(jump_stats(J['mo_pos'], J['mo_yaw'], ok_all, mv, 'mo'))
        M.update(jump_stats(J['ob_pos'], J['ob_yaw'], J['ok'], mv, 'ob', calm))
        M.update(jump_stats(J['mb_pos'], J['mb_yaw'], J['ok'], mv, 'mb', calm))
    else:
        missing(keysA, TOPICS['tf'], TOPICS['wheel'])

    # B. kroky EKF, nulove rychlosti
    keysB = ['ekf_steps_gt1cm_per_min', 'ekf_steps_gt3cm_per_min', 'ekf_corr_cm_per_m']
    if ekf is not None and wheel is not None:
        M.update(ekf_step_stats(ekf, wheel, yref_t, yref_yaw))
    else:
        missing(keysB, TOPICS['ekf'], TOPICS['wheel'])
    od = D.get('odom') if _have(D, 'odom', 3) else None
    if od is None and _have(D, 'ekf', 3):
        od = D['ekf']
        notes['odom_zero_twist_pct'] = 'z %s (v nahravce chybi %s; stejny obsah)' % (
            TOPICS['ekf'], TOPICS['odom'])
    if od is not None and wheel is not None:
        M['odom_zero_twist_pct'] = zero_twist_pct(od, wheel)
    else:
        missing(['odom_zero_twist_pct'], TOPICS['odom'], TOPICS['wheel'])

    # C. vlastnik pozy
    loc_t, owner = D.get('loc_t'), D.get('loc_owner')
    has_loc = loc_t is not None and len(loc_t) >= 2
    keysC1 = ['owner_changes', 'owner_dr_gps_per_hour']
    keysC2 = ['takeover_gap_med_cm', 'takeover_gap_max_cm', 'takeover_settle_med_s',
              'takeover_settle_max_s', 'gps_gap_med_cm', 'gps_gap_p95_cm']
    episodes = []
    if has_loc:
        tr = owner_transitions(loc_t, owner)
        episodes = owner_episodes(loc_t, owner, 'gps')
        hours = max(float(loc_t[-1] - loc_t[0]), 1e-9) / 3600.0
        n_dg = sum(1 for (_, a, b) in tr if {a, b} == {'dr', 'gps'})
        M['owner_changes'] = len(tr)
        M['owner_dr_gps_per_hour'] = n_dg / hours
        detail['owner_transitions'] = [{'t': round(t - t_beg, 2), 'from': a, 'to': b}
                                       for (t, a, b) in tr]
        dts = np.diff(loc_t, append=loc_t[-1])
        tot = max(float(dts.sum()), 1e-9)
        meta['owner_share_pct'] = {str(o): round(100 * float(dts[np.asarray(owner) == o].sum()) / tot, 1)
                                   for o in sorted(set(owner))}
    else:
        missing(keysC1, TOPICS['loc'])
    if has_loc and has_tf and ekf is not None and _have(D, 'gps', 3):
        if episodes:
            gt, gap, ing = gps_gap(T, ekf, D['gps'], episodes)
            tk = takeover_stats(gt, gap, episodes)
            gaps = [x['gap_cm'] for x in tk if x['gap_cm'] is not None]
            sett = [x['settle_s'] for x in tk if x['settle_s'] is not None]
            M['takeover_gap_med_cm'] = float(np.median(gaps)) if gaps else None
            M['takeover_gap_max_cm'] = max(gaps) if gaps else None
            M['takeover_settle_med_s'] = float(np.median(sett)) if sett else None
            M['takeover_settle_max_s'] = max(sett) if sett else None
            M['gps_gap_med_cm'] = 100 * pct(gap[ing], 50) if ing.any() else None
            M['gps_gap_p95_cm'] = 100 * pct(gap[ing], 95) if ing.any() else None
            if len(gaps) < len(tk):
                notes['takeover_gap_max_cm'] = 'u %d z %d prevzeti chybi %s pred prevzetim' % (
                    len(tk) - len(gaps), len(tk), TOPICS['gps'])
            for x in tk:
                x['t'] = round(x['t'] - t_beg, 2)
            detail['takeovers'] = tk
            if len(sett) < len(tk):
                notes['takeover_settle_max_s'] = '%d z %d prevzeti se v epizode nedorovnalo pod %g cm' % (
                    len(tk) - len(sett), len(tk), 100 * SETTLE_M)
        else:
            for k in keysC2:
                M[k] = None
                notes[k] = 'v nahravce neni zadna epizoda owner=gps'
    else:
        missing(keysC2, TOPICS['loc'], TOPICS['tf'], TOPICS['ekf'], TOPICS['gps'])
    # C2. mezera s opravou razitek, v ramu odom
    keysC3 = ['gps_stamp_offset_s', 'gps_gap_odom_med_cm', 'gps_gap_odom_p95_cm',
              'gps_gap_standing_med_cm', 'gps_along_med_cm', 'gps_along_out_med_cm',
              'gps_along_back_med_cm', 'gps_along_abs_p95_cm', 'gps_cross_med_cm',
              'gps_cross_abs_p95_cm', 'gps_gap_fit_dt_s', 'gps_gap_rtk_fixed_pct',
              'takeover_gap_odom_med_cm',
              'takeover_gap_odom_max_cm', 'takeover_settle_odom_med_s', 'takeover_settle_odom_max_s']
    if has_loc and ekf is not None and _have(D, 'gps', 3):
        off, off_src = stamp_offset(D['gps'][:, 0], D.get('pvt') if _have(D, 'pvt', 3) else None,
                                    D['gps'])
        for k in keysC3:
            M[k] = None
        M['gps_stamp_offset_s'] = float(np.median(off))
        meta['gps_stamp_offset_src'] = off_src
        meta['gps_stamp_offset_range_s'] = [round(float(off.min()), 3), round(float(off.max()), 3)]
        if off_src != 'navpvt':
            notes['gps_stamp_offset_s'] = ('z (prijeti - razitko) %s, chybi %s s casem UTC: '
                                           'obsahuje i zpozdeni navsatu' % (TOPICS['gps'], TOPICS['pvt']))
        if abs(M['gps_stamp_offset_s']) > 0.1:
            for k in keysC2:
                if M.get(k) is not None:
                    notes[k] = 'zkreslene posunem hodin %+.2f s' % M['gps_stamp_offset_s']
        if episodes:
            G = gps_gap_odom(ekf, D['gps'], off, episodes, D.get('pvt') if _have(D, 'pvt', 3) else None)
            M.update(gap_odom_stats(G))
            tk = takeover_stats(G['t'], G['gap'], episodes)
            gaps = [x['gap_cm'] for x in tk if x['gap_cm'] is not None]
            sett = [x['settle_s'] for x in tk if x['settle_s'] is not None]
            M['takeover_gap_odom_med_cm'] = float(np.median(gaps)) if gaps else None
            M['takeover_gap_odom_max_cm'] = max(gaps) if gaps else None
            M['takeover_settle_odom_med_s'] = float(np.median(sett)) if sett else None
            M['takeover_settle_odom_max_s'] = max(sett) if sett else None
            if len(sett) < len(tk):
                notes['takeover_settle_odom_max_s'] = (
                    '%d z %d prevzeti se v epizode nedorovnalo pod %g cm'
                    % (len(tk) - len(sett), len(tk), 100 * SETTLE_M))
            for x in tk:
                x['t'] = round(x['t'] - t_beg, 2)
            detail['takeovers_odom'] = tk
        else:
            for k in keysC3[1:]:
                notes[k] = 'v nahravce neni zadna epizoda owner=gps'
    else:
        missing(keysC3, TOPICS['loc'], TOPICS['ekf'], TOPICS['gps'])

    # D, E, F. dok
    keysD = ['yaw_handover_s', 'yaw_handover_turn_deg', 'yaw_handover_excess_deg',
             'yaw_handover_max_step_deg']
    keysE = ['dock_jump_cm', 'dock_jump_deg', 'dock_shift_cm']
    keysF = ['to_rtk_fixed_s', 'to_owner_gps_s', 'to_undocked_s', 'to_depart_s', 'undock_standing_s']
    keysW = ['takeover_wander_med', 'takeover_wander_max', 'takeover_path_map_cm',
             'takeover_needed_cm', 'takeover_yaw_wobble_deg',
             'takeover_wander_warm_med', 'takeover_wander_warm_max', 'takeover_warm_n',
             'takeover_step_max_cm', 'takeover_step_warm_max_cm']
    if _have(D, 'dock', 2):
        und, dks = dock_edges(D['dock'])
        meta['undocks'], meta['docks'] = len(und), len(dks)

        def nxt(t0):
            later = [d for d in dks if d > t0]
            return later[0] if later else t_fin

        def agg(rows, key):
            v = [r[key] for r in rows if r and r.get(key) is not None]
            return max(v) if v else None

        if has_tf and yref_t is not None and und:
            if calm_t is None and wheel is not None:
                calm_t = calm_at(tau, wheel, imu)
            hs = [yaw_handover(T, tau, yref_t, yref_yaw, t0, nxt(t0), calm_t) for t0 in und]
            hs = [h for h in hs if h]
            for src, dst in (('s', 'yaw_handover_s'), ('turn_deg', 'yaw_handover_turn_deg'),
                             ('excess_deg', 'yaw_handover_excess_deg'),
                             ('max_step_deg', 'yaw_handover_max_step_deg')):
                M[dst] = agg(hs, src)
                if M[dst] is None:
                    notes[dst] = ('predani probehlo v zatacce (malo tiku v klidu kurzu)' if hs else
                                  'po vyjezdu neprobehlo predani kurzu (map->odom nedosel na 0)')
            for h in hs:
                h['t'] = round(h['t'] - t_beg, 2)
            detail['yaw_handovers'] = hs
        elif not und:
            for k in keysD:
                M[k] = None
                notes[k] = 'v nahravce neni vyjezd z doku'
        else:
            missing(keysD, TOPICS['tf'], TOPICS['imu'])
        if J is not None and dks:
            dj = [x for x in (dock_jump(T, J, t) for t in dks) if x]
            M['dock_jump_cm'] = agg(dj, 'jump_cm')
            M['dock_jump_deg'] = agg(dj, 'jump_deg')
            M['dock_shift_cm'] = agg(dj, 'shift_cm')
            for x in dj:
                x['t'] = round(x['t'] - t_beg, 2)
            detail['dockings'] = dj
        elif not dks:
            for k in keysE:
                M[k] = None
                notes[k] = 'v nahravce neni zadokovani'
        else:
            missing(keysE, TOPICS['tf'], TOPICS['wheel'])
        if und:
            pvt = D.get('pvt') if _have(D, 'pvt', 2) else None
            rows = [undock_times(t0, nxt(t0), D['dock'], pvt, loc_t if has_loc else None,
                                 owner, wheel) for t0 in und]
            for k in keysF:
                M[k] = agg(rows, k)
            if pvt is None:
                notes['to_rtk_fixed_s'] = '%s (chybi %s)' % (NA, TOPICS['pvt'])
            if not has_loc:
                notes['to_owner_gps_s'] = '%s (chybi %s)' % (NA, TOPICS['loc'])
            for k in keysF:
                if M[k] is None and k not in notes:
                    notes[k] = 'po vyjezdu v nahravce nenastalo'
            for r in rows:
                r['t'] = round(r['t'] - t_beg, 2)
            detail['undocks'] = rows
        else:
            for k in keysF:
                M[k] = None
                notes[k] = 'v nahravce neni vyjezd z doku'
        # C3. pohyblive prevzeti polohy po vyjezdu
        for k in keysW:
            M[k] = None
        if not und:
            for k in keysW:
                notes[k] = 'v nahravce neni vyjezd z doku'
        elif not (has_tf and has_loc and wheel is not None and yref_t is not None):
            missing(keysW, TOPICS['tf'], TOPICS['loc'], TOPICS['wheel'])
        else:
            tgs = []
            for t0 in und:
                first = [a for (a, _) in episodes if t0 < a <= nxt(t0)]
                if first:
                    tgs.append((t0, first[0]))
            gps_t = D['gps'][:, 0] if _have(D, 'gps', 3) else None
            ws = []
            for t0, tg in tgs:
                x = takeover_wander(T, tau, wheel, yref_t, yref_yaw, imu, tg)
                if not x:
                    continue
                if gps_t is not None:       # v5: prvni vyjezd po startu uzlu
                    x['cold'] = not bool(np.any((gps_t > t0) & (gps_t < tg - 0.3)))
                else:
                    k = min(int(np.searchsorted(T['t'], t0)), len(T['t']) - 1)
                    x['cold'] = bool(abs(math.degrees(T['myaw'][k])) > WANDER_COLD_YAW_DEG)
                ws.append(x)
            tgs = [tg for _, tg in tgs]
            if ws:
                med = lambda key: float(np.median([x[key] for x in ws]))
                M['takeover_wander_med'] = med('wander')
                M['takeover_wander_max'] = max(x['wander'] for x in ws)
                M['takeover_path_map_cm'] = med('path_cm')
                M['takeover_needed_cm'] = med('needed_cm')
                M['takeover_yaw_wobble_deg'] = med('yaw_wobble_deg')
                warm = [x for x in ws if not x['cold']]
                M['takeover_warm_n'] = len(warm)
                M['takeover_step_max_cm'] = max(x['step_max_cm'] for x in ws)
                if warm:
                    M['takeover_wander_warm_med'] = float(np.median([x['wander'] for x in warm]))
                    M['takeover_wander_warm_max'] = max(x['wander'] for x in warm)
                    M['takeover_step_warm_max_cm'] = max(x['step_max_cm'] for x in warm)
                else:
                    for k in ('takeover_wander_warm_med', 'takeover_wander_warm_max',
                              'takeover_step_warm_max_cm'):
                        notes[k] = 'v nahravce je jen prvni vyjezd po startu uzlu'
                if len(ws) < len(und):
                    notes['takeover_wander_max'] = 'jen %d z %d vyjezdu (bez prevzeti nebo okno mimo nahravku)' % (
                        len(ws), len(und))
                for x in ws:
                    x['t'] = round(x['t'] - t_beg, 2)
                detail['takeover_wander'] = ws
            else:
                for k in keysW:
                    notes[k] = ('po vyjezdu v nahravce nenastalo prevzeti (owner -> gps)' if not tgs else
                                'okno prevzeti (-%g az +%g s) neni cele v nahravce'
                                % (WANDER_PRE_S, WANDER_POST_S))
    else:
        missing(keysD + keysE + keysF + keysW, TOPICS['dock'])

    keysS = ['gps_standing_yaw_std_deg', 'gps_standing_yaw_p2p_deg', 'gps_standing_pos_p95_cm']
    if has_tf and has_loc and wheel is not None and yref_t is not None:
        sj = standing_jitter(T, tau, wheel, yref_t, yref_yaw, hold(T['t'], loc_t, owner) == 'gps')
        if sj:
            for k in keysS:
                M[k] = sj[k]
            detail['gps_standing'] = {'useku': sj['n_seg'], 'celkem_s': round(sj['total_s'], 1)}
        else:
            for k in keysS:
                M[k] = None
                notes[k] = 'v nahravce robot pri owner=gps nestal (>= 2 s)'
    else:
        missing(keysS, TOPICS['tf'], TOPICS['loc'], TOPICS['wheel'])

    # G. kotvy
    keysG = ['anchor_age_p95_s', 'anchor_age_max_s', 'anchor_freezes',
             'heading_anchor_age_p95_s', 'heading_anchor_age_max_s', 'heading_anchor_freezes']
    if has_loc:
        sel = np.asarray(owner) == 'gps'
        M.update(age_stats(D['loc_anchor_age'], sel, 'anchor'))
        M.update(age_stats(D['loc_heading_age'], sel, 'heading_anchor'))
        for k in keysG:
            if M[k] is None:
                notes[k] = 'v nahravce neni zadny vzorek owner=gps s kotvou'
    else:
        missing(keysG, TOPICS['loc'])
    mG, invalid, invalid_why = anchor_fallback(
        owner if has_loc else None, D.get('loc_anchor_mode') if has_loc else None,
        D.get('loc_anchor_age') if has_loc else None, D.get('gps') if _have(D, 'gps', 3) else None,
        D.get('loc_anchor_starving') if has_loc else None,
        D.get('ekfw') if _have(D, 'ekfw', 3) else None)
    M.update(mG)
    if mG['anchor_starving_pct'] is None:
        notes['anchor_starving_pct'] = 'v nahravce neni pole anchor_starving (anchor_mode_report vypnuty)'
    if mG['anchor_fallback_pct'] is None:
        notes['anchor_fallback_pct'] = ('%s (chybi %s)' % (NA, TOPICS['loc']) if not has_loc
                                        else 'v nahravce neni zadny vzorek owner=gps')
    bs = D.get('loc_buffered_share') if has_loc else None
    M['anchor_buffered_share_med_pct'] = M['anchor_buffered_share_p05_pct'] = None
    if bs is not None:
        b_ok = bs[(np.asarray(owner) == 'gps') & np.isfinite(bs)]
        if len(b_ok):
            M['anchor_buffered_share_med_pct'] = 100 * float(np.median(b_ok))
            M['anchor_buffered_share_p05_pct'] = 100 * pct(b_ok, 5)
    if M['anchor_buffered_share_med_pct'] is None:
        for k in ('anchor_buffered_share_med_pct', 'anchor_buffered_share_p05_pct'):
            notes[k] = 'v nahravce neni pole anchor_buffered_share (anchor_mode_report vypnuty)'
    elif mG['gps_buffered_est_pct'] is not None:
        notes['gps_buffered_est_pct'] = 'jen odhad bez opravy razitek; plati anchor_buffered_share'
    if mG['gps_rx_minus_stamp_s'] is None:
        notes['gps_rx_minus_stamp_s'] = '%s (chybi %s)' % (NA, TOPICS['gps'])
    if mG['gps_buffered_est_pct'] is None:
        notes['gps_buffered_est_pct'] = '%s (chybi %s nebo %s)' % (NA, TOPICS['gps'], TOPICS['ekfw'])

    # H. ostatni
    if ekf is not None:
        z = ekf[:, 4]
        M['z_drift_m'] = float(z[-1] - z[0])
        M['z_range_m'] = float(z.max() - z.min())
    else:
        missing(['z_drift_m', 'z_range_m'], TOPICS['ekf'])
    if has_tf:
        M.update(tf_gap_stats(T, tau, wheel))
        if len(T['t']) > 2:
            detail['tf_gap_max_at_s'] = round(float(T['t'][int(np.argmax(np.diff(T['t'])))] - t_beg), 2)
    else:
        missing(['tf_gap_max_s', 'tf_gap_p99_s', 'tf_stale_pct'], TOPICS['tf'])
    ro = D.get('rosout_text')
    if ro is not None:
        low = [s.lower() for s in ro]
        M['no_progress_n'] = sum(1 for s in low if 'no goal progress' in s)
        M['rewindow_n'] = sum(1 for s in low if 'rewindow' in s or 're-window' in s)
    else:
        missing(['no_progress_n', 'rewindow_n'], TOPICS['rosout'])
    res = D.get('res_outcome')
    if res is not None:
        M['mbf_fail_n'] = int((np.asarray(res) != 0).sum())
        meta['mbf_results'] = int(len(res))
    else:
        missing(['mbf_fail_n'], TOPICS['res'])
    return {'tool': 'nav_score', 'version': VERSION, 'meta': meta, 'metrics': M,
            'notes': notes, 'detail': detail,
            'pos_metrics_invalid': invalid, 'pos_metrics_invalid_why': invalid_why}


# --------------------------------------------------------------------------
# vytah z bagu
# --------------------------------------------------------------------------
_U3 = struct.Struct('<III')
_D7 = struct.Struct('<7d')
_D6 = struct.Struct('<6d')
_D4 = struct.Struct('<4d')
_D3 = struct.Struct('<3d')
_U1 = struct.Struct('<I')


def _hdr(buf, off=0):
    """std_msgs/Header -> (stamp, frame_id, offset za hlavickou)."""
    _, sec, nsec = _U3.unpack_from(buf, off)
    n, = _U1.unpack_from(buf, off + 12)
    off += 16
    return sec + nsec * 1e-9, buf[off:off + n], off + n


def parse_odom(buf):
    """nav_msgs/Odometry (serializovana) -> (stamp, x, y, z, yaw, vx, vy, wz)."""
    st, _, off = _hdr(buf)
    n, = _U1.unpack_from(buf, off)
    off += 4 + n
    x, y, z, qx, qy, qz, qw = _D7.unpack_from(buf, off)
    off += 56 + 288
    vx, vy, _, _, _, wz = _D6.unpack_from(buf, off)
    return st, x, y, z, yaw_q(qx, qy, qz, qw), vx, vy, wz


def parse_imu(buf):
    """sensor_msgs/Imu (serializovana) -> (stamp, yaw, wz)."""
    st, _, off = _hdr(buf)
    qx, qy, qz, qw = _D4.unpack_from(buf, off)
    off += 32 + 72
    _, _, wz = _D3.unpack_from(buf, off)
    return st, yaw_q(qx, qy, qz, qw), wz


def parse_tf(buf):
    """tf2_msgs/TFMessage (serializovana) -> [(parent, child, stamp, x, y, z, yaw)]."""
    n, = _U1.unpack_from(buf, 0)
    off, out = 4, []
    for _ in range(n):
        st, parent, off = _hdr(buf, off)
        m, = _U1.unpack_from(buf, off)
        child = buf[off + 4:off + 4 + m]
        off += 4 + m
        x, y, z, qx, qy, qz, qw = _D7.unpack_from(buf, off)
        off += 56
        out.append((parent.lstrip(b'/'), child.lstrip(b'/'), st, x, y, z, yaw_q(qx, qy, qz, qw)))
    return out


def pvt_utc(year, month, day, hour, minute, sec, nano, valid=3):
    """Cas UTC zpravy NAV-PVT [s od 1970]; NaN, kdyz prijimac nema platne
    datum a cas (valid bity 0 a 1)."""
    if (valid & 3) != 3 or year < 2020:
        return float('nan')
    return calendar.timegm((year, month, day, hour, minute, sec)) + nano * 1e-9


def extract(bag_path):
    """Jedno cteni bagu -> dict poli (vytah). Vsechna 2D pole maji v prvnim
    sloupci cas bagu; odometrie: (bag_t, stamp, x, y, z, yaw, vx, vy, wz);
    mo/ob: (bag_t, stamp, x, y, z, yaw); imu: (bag_t, stamp, yaw, wz);
    gps: (bag_t, stamp, x, y, z); pvt: (bag_t, fixType, carrSoln, hAcc_m, utc);
    hrel: (bag_t, carrSoln, headValid, heading_deg, accHeading_deg);
    dock: (bag_t, hodnota). Chybejici tema = klic chybi."""
    import rosbag
    rows = {k: [] for k in ('mo', 'ob', 'ekf', 'odom', 'ekfw', 'wheel', 'gps', 'pvt', 'hrel',
                            'imu', 'dock')}
    loc, res, rosout = [], [], []
    by_topic = {v: k for k, v in TOPICS.items()}
    odo = {'ekf', 'odom', 'ekfw', 'wheel'}
    counts, n_datum = {}, 0
    with rosbag.Bag(bag_path) as bag:
        info = bag.get_type_and_topic_info().topics
        want = [t for t in TOPICS.values() if t in info]
        for t in want:
            counts[t] = info[t].message_count
        for topic, raw, t in bag.read_messages(topics=want, raw=True):
            ts = t.to_sec()
            k = by_topic[topic]
            buf = raw[1]
            if k == 'tf':
                for (p, c, st, x, y, z, yaw) in parse_tf(buf):
                    if p == b'map' and c == b'odom':
                        rows['mo'].append((ts, st, x, y, z, yaw))
                    elif p == b'odom' and c == b'base_link':
                        rows['ob'].append((ts, st, x, y, z, yaw))
            elif k in odo:
                rows[k].append((ts,) + parse_odom(buf))
            elif k == 'imu':
                rows[k].append((ts,) + parse_imu(buf))
            elif k == 'gps':
                o = parse_odom(buf)
                rows[k].append((ts, o[0], o[1], o[2], o[3]))
            elif k == 'tf_static':
                n_datum += sum(1 for x in parse_tf(buf) if b'utm' in (x[0], x[1]))
            else:
                m = raw[4]()
                m.deserialize(buf)
                if k == 'loc':
                    loc.append((ts, m.data))
                elif k == 'dock':
                    rows[k].append((ts, m.data))
                elif k == 'pvt':
                    rows[k].append((ts, m.fixType, (m.flags >> 6) & 3, m.hAcc * 1e-3,
                                    pvt_utc(m.year, m.month, m.day, m.hour, m.min, m.sec,
                                            m.nano, m.valid)))
                elif k == 'hrel':
                    rows[k].append((ts, (m.flags >> 3) & 3, (m.flags >> 8) & 1,
                                    m.relPosHeading * 1e-5, m.accHeading * 1e-5))
                elif k == 'res':
                    res.append((ts, m.result.outcome, m.result.message))
                elif k in ('rosout', 'rosout_agg'):
                    rosout.append(m.msg)
    D = {k: np.array(v, dtype=float) for k, v in rows.items() if v}
    D['counts'] = counts
    D['navsat_datums'] = n_datum
    if loc:
        js = [json.loads(s) for _, s in loc]
        fl = lambda v: float('nan') if v is None else float(v)
        D['loc_t'] = np.array([t for t, _ in loc])
        D['loc_owner'] = np.array([str(j.get('owner')) for j in js])
        D['loc_anchor_age'] = np.array([fl(j.get('anchor_age_s')) for j in js])
        D['loc_heading_age'] = np.array([fl(j.get('heading_anchor_age_s')) for j in js])
        D['loc_anchor_mode'] = np.array([str(j.get('anchor_mode') or '') for j in js])
        D['loc_anchor_starving'] = np.array([bool(j.get('anchor_starving')) for j in js])
        D['loc_buffered_share'] = np.array([fl(j.get('anchor_buffered_share')) for j in js])
    if TOPICS['res'] in counts:
        D['res_outcome'] = np.array([r[1] for r in res], dtype=int)
    if TOPICS['rosout'] in counts or TOPICS['rosout_agg'] in counts:
        D['rosout_text'] = np.array(rosout, dtype=str)
    return D


def load(bag_path, cache_dir=None):
    """Vytah z bagu; s cache_dir se uklada do / cte z <cache_dir>/<bag>.<velikost>.v<N>.npz
    (N = EXTRACT_VERSION)."""
    if not cache_dir:
        return extract(bag_path)
    st = os.stat(bag_path)
    f = os.path.join(cache_dir, '%s.%d.v%d.npz' % (os.path.basename(bag_path), st.st_size, EXTRACT_VERSION))
    if os.path.exists(f) and os.path.getmtime(f) >= st.st_mtime:
        Z = np.load(f, allow_pickle=False)
        D = {k: Z[k] for k in Z.files if k != 'counts_json'}
        D['counts'] = json.loads(str(Z['counts_json']))
        D['navsat_datums'] = int(D['navsat_datums'])
        return D
    D = extract(bag_path)
    os.makedirs(cache_dir, exist_ok=True)
    out = {k: v for k, v in D.items() if k != 'counts'}
    np.savez_compressed(f, counts_json=json.dumps(D['counts']), **out)
    return D


# --------------------------------------------------------------------------
# vypis a porovnani
# --------------------------------------------------------------------------
# (klic, popis, jednotka, smer: 'low' = mensi je lepsi, 'info' = nehodnoti se)
SPEC = [
    ('A. Skoky pozy za tik (map->odom | odom->base | celkem map->base)', None, None, None),
    ('mo_pos_p95_cm', 'map->odom: krok polohy p95 za jizdy', 'cm', 'low'),
    ('mo_pos_max_cm', 'map->odom: krok polohy max', 'cm', 'low'),
    ('mo_n_gt1cm', 'map->odom: kroku > 1 cm', '', 'low'),
    ('mo_n_gt3cm', 'map->odom: kroku > 3 cm', '', 'low'),
    ('mo_yaw_p95_deg', 'map->odom: krok kurzu p95 za jizdy', 'st', 'low'),
    ('mo_yaw_max_deg', 'map->odom: krok kurzu max', 'st', 'low'),
    ('mo_n_yaw_gt03', 'map->odom: kroku kurzu > 0,3 st', '', 'low'),
    ('ob_pos_p95_cm', 'odom->base: krok polohy p95 za jizdy', 'cm', 'low'),
    ('ob_pos_max_cm', 'odom->base: krok polohy max', 'cm', 'low'),
    ('ob_n_gt1cm', 'odom->base: kroku > 1 cm', '', 'low'),
    ('ob_n_gt3cm', 'odom->base: kroku > 3 cm', '', 'low'),
    ('ob_yaw_p95_deg', 'odom->base: krok kurzu p95 za jizdy', 'st', 'low'),
    ('ob_yaw_max_deg', 'odom->base: krok kurzu max', 'st', 'low'),
    ('ob_n_yaw_gt03', 'odom->base: kroku kurzu > 0,3 st', '', 'low'),
    ('mb_pos_p95_cm', 'map->base: krok polohy p95 za jizdy', 'cm', 'low'),
    ('mb_pos_max_cm', 'map->base: krok polohy max', 'cm', 'low'),
    ('mb_n_gt1cm', 'map->base: kroku > 1 cm', '', 'low'),
    ('mb_n_gt3cm', 'map->base: kroku > 3 cm', '', 'low'),
    ('mb_yaw_p95_deg', 'map->base: krok kurzu p95 za jizdy', 'st', 'low'),
    ('mb_yaw_max_deg', 'map->base: krok kurzu max', 'st', 'low'),
    ('mb_n_yaw_gt03', 'map->base: kroku kurzu > 0,3 st', '', 'low'),
    ('B. EKF', None, None, None),
    ('ekf_steps_gt1cm_per_min', 'kroky EKF > 1 cm na minutu jizdy', '/min', 'low'),
    ('ekf_steps_gt3cm_per_min', 'kroky EKF > 3 cm na minutu jizdy', '/min', 'low'),
    ('ekf_corr_cm_per_m', 'soucet kroku EKF na metr jizdy', 'cm/m', 'low'),
    ('odom_zero_twist_pct', 'propad rychlosti v /odometry/odom za jizdy', '%', 'low'),
    ('C. Vlastnik pozy', None, None, None),
    ('owner_changes', 'zmen vlastnika', '', 'low'),
    ('owner_dr_gps_per_hour', 'prepnuti dr<->gps za hodinu', '/h', 'low'),
    ('takeover_gap_med_cm', '(stare*) mezera pred prevzetim GPS, median', 'cm', 'low'),
    ('takeover_gap_max_cm', '(stare*) mezera pred prevzetim GPS, max', 'cm', 'low'),
    ('takeover_settle_med_s', '(stare*) doba dorovnani po prevzeti, median', 's', 'low'),
    ('takeover_settle_max_s', '(stare*) doba dorovnani po prevzeti, max', 's', 'low'),
    ('gps_gap_med_cm', '(stare*) |fuze - GPS| pri owner=gps, median', 'cm', 'low'),
    ('gps_gap_p95_cm', '(stare*) |fuze - GPS| pri owner=gps, p95', 'cm', 'low'),
    ('C2. Mezera EKF - GPS, razitka opravena o posun hodin, ram odom  (*stare = zkreslene pri posunu hodin)',
     None, None, None),
    ('gps_stamp_offset_s', 'posun hodin (system - UTC prijimace), median', 's', 'info'),
    ('gps_gap_odom_med_cm', '|EKF - GPS| pri owner=gps, median', 'cm', 'low'),
    ('gps_gap_odom_p95_cm', '|EKF - GPS| pri owner=gps, p95', 'cm', 'low'),
    ('gps_gap_standing_med_cm', '|EKF - GPS| ve stoje, median', 'cm', 'low'),
    ('gps_along_med_cm', 'podel drahy (+ EKF vpredu), median, jizda vpred', 'cm', 'abs'),
    ('gps_along_out_med_cm', 'podel drahy, smer TAM, median', 'cm', 'abs'),
    ('gps_along_back_med_cm', 'podel drahy, smer ZPET, median', 'cm', 'abs'),
    ('gps_along_abs_p95_cm', '|podel drahy| za jizdy, p95', 'cm', 'low'),
    ('gps_cross_med_cm', 'pricne (+ EKF vlevo), median za jizdy', 'cm', 'abs'),
    ('gps_cross_abs_p95_cm', '|pricne| za jizdy, p95', 'cm', 'low'),
    ('gps_gap_fit_dt_s', 'zbytkovy casovy posun z primky podel ~ v', 's', 'info'),
    ('gps_gap_rtk_fixed_pct', 'podil vzorku owner=gps s RTK fixem (jen ty vyse)', '%', 'info'),
    ('takeover_gap_odom_med_cm', 'mezera pred prevzetim GPS, median', 'cm', 'low'),
    ('takeover_gap_odom_max_cm', 'mezera pred prevzetim GPS, max', 'cm', 'low'),
    ('takeover_settle_odom_med_s', 'doba dorovnani po prevzeti, median', 's', 'low'),
    ('takeover_settle_odom_max_s', 'doba dorovnani po prevzeti, max', 's', 'low'),
    ('C3. Pohyblive prevzeti po vyjezdu: pohyb obrazu robota v mape mimo kola, okno -1 az +10 s',
     None, None, None),
    ('takeover_wander_med', 'M = draha + 0,5 x kolisani kurzu, median', '', 'low'),
    ('takeover_wander_max', 'M = draha + 0,5 x kolisani kurzu, max', '', 'low'),
    ('takeover_path_map_cm', 'draha obrazu v mape mimo kola, median', 'cm', 'low'),
    ('takeover_needed_cm', 'skutecne potrebna oprava polohy, median', 'cm', 'low'),
    ('takeover_yaw_wobble_deg', 'kolisani kurzu pri prevzeti, median', 'st', 'low'),
    ('takeover_wander_warm_med', 'M bez prvniho vyjezdu po startu uzlu, median', '', 'low'),
    ('takeover_wander_warm_max', 'M bez prvniho vyjezdu po startu uzlu, max', '', 'low'),
    ('takeover_warm_n', 'pocet vyjezdu mimo prvni po startu uzlu', '', 'info'),
    ('takeover_step_max_cm', 'nejvetsi pohyb obrazu za 0,1 s pri prevzeti', 'cm', 'low'),
    ('takeover_step_warm_max_cm', '... bez prvniho vyjezdu po startu uzlu', 'cm', 'low'),
    ('D. Kurz: predani po vyjezdu a klid ve stoje', None, None, None),
    ('yaw_handover_s', 'doba predani kurzu', 's', 'low'),
    ('yaw_handover_turn_deg', 'otoceni map->odom pri predani', 'st', 'info'),
    ('yaw_handover_excess_deg', 'nadbytecne otaceni (rozkmit)', 'st', 'low'),
    ('yaw_handover_max_step_deg', 'nejvetsi krok kurzu za tik', 'st', 'low'),
    ('gps_standing_yaw_std_deg', 'kolisani kurzu ve stoje (owner=gps), sm. odch.', 'st', 'low'),
    ('gps_standing_yaw_p2p_deg', 'kolisani kurzu ve stoje (owner=gps), rozkmit', 'st', 'low'),
    ('gps_standing_pos_p95_cm', 'kolisani polohy ve stoje (owner=gps), p95', 'cm', 'low'),
    ('E. Zadokovani', None, None, None),
    ('dock_jump_cm', 'skok polohy za tik pri zadokovani', 'cm', 'low'),
    ('dock_jump_deg', 'skok kurzu za tik pri zadokovani', 'st', 'low'),
    ('dock_shift_cm', 'celkovy posun pri zadokovani', 'cm', 'low'),
    ('F. Casy po opusteni doku (max pres vyjezdy)', None, None, None),
    ('to_rtk_fixed_s', 'do RTK fixu', 's', 'low'),
    ('to_owner_gps_s', 'do owner=gps', 's', 'low'),
    ('to_undocked_s', 'do dokonceni vyjezdu (dock_status 3)', 's', 'low'),
    ('to_depart_s', 'do rozjezdu po vyjezdu', 's', 'low'),
    ('undock_standing_s', 'stani behem vyjezdu', 's', 'low'),
    ('G. Kotvy pri owner=gps', None, None, None),
    ('anchor_age_p95_s', 'stari kotvy polohy p95', 's', 'low'),
    ('anchor_age_max_s', 'stari kotvy polohy max', 's', 'low'),
    ('anchor_freezes', 'zamrznuti kotvy polohy (> 5 s)', '', 'low'),
    ('heading_anchor_age_p95_s', 'stari kotvy kurzu p95', 's', 'low'),
    ('heading_anchor_age_max_s', 'stari kotvy kurzu max', 's', 'low'),
    ('heading_anchor_freezes', 'zamrznuti kotvy kurzu (> 5 s)', '', 'low'),
    ('anchor_fallback_pct', 'kotva polohy v nouzovem rezimu (set_pose)', '%', 'low'),
    ('anchor_starving_pct', 'kotva polohy hladovi (malo vzorku GPS)', '%', 'low'),
    ('gps_rx_minus_stamp_s', '/odometry/gps: prijeti - razitko, median', 's', 'info'),
    ('gps_buffered_est_pct', 'odhad: vzorky GPS do bufferu bez opravy razitek', '%', 'info'),
    ('anchor_buffered_share_med_pct', 'vzorky GPS v bufferu kotvy (loc_status), median', '%', 'info'),
    ('anchor_buffered_share_p05_pct', 'vzorky GPS v bufferu kotvy (loc_status), p05', '%', 'info'),
    ('H. Ostatni', None, None, None),
    ('z_drift_m', 'drift vysky EKF (konec - zacatek)', 'm', 'abs'),
    ('z_range_m', 'rozsah vysky EKF', 'm', 'low'),
    ('tf_gap_max_s', 'nejvetsi mezera v /tf', 's', 'low'),
    ('tf_gap_p99_s', 'mezera v /tf p99', 's', 'low'),
    ('tf_stale_pct', 'opakovana poza v /tf za jizdy', '%', 'low'),
    ('no_progress_n', 'hlasek "no goal progress"', '', 'low'),
    ('rewindow_n', 'hlasek "rewindow"', '', 'low'),
    ('mbf_fail_n', 'neuspesnych exe_path', '', 'low'),
]


def _fmt(v):
    if v is None:
        return None
    if isinstance(v, (int, np.integer)):
        return '%d' % v
    a = abs(v)
    return '%.0f' % v if a >= 100 else ('%.1f' % v if a >= 10 else ('%.2f' % v if a >= 0.1 else '%.3f' % v))


def print_table(R, out=sys.stdout):
    m, meta = R['metrics'], R['meta']
    w = out.write
    w('nav_score v%d  %s\n' % (R['version'], R.get('bag', '')))
    w('delka %.0f s' % meta.get('duration_s', 0))
    if 'path_m' in meta:
        w(', draha %.1f m, jizda %.1f min' % (meta['path_m'], meta['drive_min']))
    if 'undocks' in meta:
        w(', vyjezdu %d, zadokovani %d' % (meta['undocks'], meta['docks']))
    w('\n')
    if 'owner_share_pct' in meta:
        w('vlastnik pozy [%% casu]: %s\n' % ', '.join(
            '%s %.0f' % kv for kv in sorted(meta['owner_share_pct'].items(), key=lambda kv: -kv[1])))
    if R.get('pos_metrics_invalid'):
        w('POZOR: polohove metriky teto jizdy NEPLATI - %s\n' % R.get('pos_metrics_invalid_why', ''))
    for key, label, unit, _ in SPEC:
        if label is None:
            w('\n%s\n' % key)
            continue
        v = _fmt(m.get(key))
        note = R['notes'].get(key)
        if v is None:
            w('  %-46s %s\n' % (label, note or NA))
        else:
            w('  %-46s %9s %-4s%s\n' % (label, v, unit, ('  [%s]' % note) if note else ''))


def compare(A, B, out=sys.stdout):
    """Vypise rozdily pred (A) / po (B). Zlepseni/zhorseni se hlasi, kdyz se
    hodnota zmeni o vic nez 10 % a zaroven o vic nez nejmensi zobrazitelny krok."""
    w = out.write
    w('nav_score: porovnani\n  pred: %s\n  po:   %s\n' % (A.get('bag', '?'), B.get('bag', '?')))
    for tag, X in (('pred', A), ('po', B)):
        if X.get('pos_metrics_invalid'):
            w('POZOR: polohove metriky jizdy "%s" NEPLATI - %s\n'
              % (tag, X.get('pos_metrics_invalid_why', '')))
    if bool(A.get('pos_metrics_invalid')) != bool(B.get('pos_metrics_invalid')):
        w('POZOR: jizdy nejsou v poloze srovnatelne (kroky EKF, mezera fuze - GPS, kolisani '
          'polohy): jedna bezela v nouzovem rezimu kotvy\n')
    n_up = n_dn = 0
    for key, label, unit, better in SPEC:
        if label is None:
            w('\n%s\n' % key)
            continue
        a, b = A['metrics'].get(key), B['metrics'].get(key)
        if a is None or b is None:
            w('  %-46s %9s -> %-9s %-4s  nelze porovnat\n' % (label, _fmt(a) or '-', _fmt(b) or '-', unit))
            continue
        xa, xb = (abs(a), abs(b)) if better == 'abs' else (a, b)
        d = xb - xa
        tol = max(0.10 * max(abs(xa), abs(xb)), 1e-3)
        if better == 'info' or abs(d) <= tol:
            verdict = ''
        elif d < 0:
            verdict, n_up = 'ZLEPSENI', n_up + 1
        else:
            verdict, n_dn = 'ZHORSENI', n_dn + 1
        w('  %-46s %9s -> %-9s %-4s  %s\n' % (label, _fmt(a), _fmt(b), unit, verdict))
    w('\ncelkem: zlepseni %d, zhorseni %d\n' % (n_up, n_dn))
    return n_up, n_dn


def _jsonable(o):
    if isinstance(o, dict):
        return {str(k): _jsonable(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_jsonable(v) for v in o]
    if isinstance(o, (np.integer,)):
        return int(o)
    if isinstance(o, (float, np.floating)):
        f = float(o)
        return None if not math.isfinite(f) else round(f, 4)
    return o


def main(argv=None):
    ap = argparse.ArgumentParser(description='Skore jizdy z rosbagu (viz docstring).')
    ap.add_argument('bag', nargs='?', help='cesta k .bag')
    ap.add_argument('--json', action='store_true', help='JSON na stdout misto tabulky')
    ap.add_argument('--json-out', metavar='F', help='JSON navic ulozit do souboru')
    ap.add_argument('--cache', metavar='DIR', help='adresar pro mezivysledek .npz')
    ap.add_argument('--compare', nargs=2, metavar=('PRED.json', 'PO.json'))
    a = ap.parse_args(argv)
    if a.compare:
        with open(a.compare[0]) as f1, open(a.compare[1]) as f2:
            compare(json.load(f1), json.load(f2))
        return 0
    if not a.bag:
        ap.error('chybi cesta k bagu (nebo --compare)')
    R = score(load(a.bag, a.cache))
    R['bag'] = os.path.abspath(a.bag)
    R = _jsonable(R)
    if a.json_out:
        with open(a.json_out, 'w') as f:
            json.dump(R, f, indent=1, ensure_ascii=False)
    if a.json:
        json.dump(R, sys.stdout, indent=1, ensure_ascii=False)
        sys.stdout.write('\n')
    else:
        print_table(R)
    return 0


if __name__ == '__main__':
    sys.exit(main())
