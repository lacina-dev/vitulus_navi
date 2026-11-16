# Mapa navigace a lokalizace (Vitulus)

Stav k 2026-11-15, větev `main` @ `30eca80`. Tento dokument je jediný pravdivý obraz; starší `STAV_*.md`, `LOCALIZATION_V2_PLAN.md`, `AUDIT_2026-08-01.md` a paměťové poznámky jsou historie a místy neplatí (viz kap. 7.4).

Zkratky odkazů: **NT** = `vitulus_navi/nodes/navi_transform`, **EKF** = `vitulus_navi/config/ekf/ekf_base_outdoor.yaml`, **ML** = `vitulus_navi/launch/navi_man.launch`, **ST** = `vitulus_navi/src/mower_unit_smach/states.py`, **MI** = `…/mission.py`, **GL** = `vitulus_navi/nodes/gloc_server`, **MUX** = `vitulus_base/nodes/cmd_vel_mux`. Čísla řádků platí pro uvedený commit.

Zdroje čísel: **[B]** = `~/bags/nav_test/nav_baseline_2026-11-15_a.bag` (2× dok → TEST → dok, bez nože, 880 s, 36 m), **[M]** = `~/bags/mag_test/magtest3_2026-11-14.bag` (sečení, 836 s, 104 m), **[S]** = stopy sečení `~/.vitulus/mapping_v3/10_2026/mow_load/2026110[89]_*, 2026111*` (13 jízd, 2 444 m), **[živě]** = `rosparam`/`rostopic info` na robotu 2026-11-15. Skóre obou nahrávek: `/home/vitulus/.claude/jobs/9e12fd1c/tmp/nav/score/*.json`.

Úroveň ověření: kapitoly 1–3, 6 (část `navi_transform`) a 7 jsem ověřil čtením celého `navi_transform` a měřením z nahrávek. Kapitoly 4, 5 a části o GNSS, doku, gloc a mapování jsou ověřené čtením kódu pomocnými průzkumy s namátkovou kontrolou řádků (launch, `states.py`, mux, yaml); u nich je možná odchylka řádku o jednotky.

---

## Stav vypínačů úprav (16. 11. 2026 07:45)

| Úprava | Parametr | Stav | Důkaz | Vrácení |
|---|---|---|---|---|
| D1 rychlost odometrie z kol při výpadku EKF | `odom_twist_from_wheels` | **zapnuto** | jízda: výpadky nulové rychlosti 2,0 % → 0,03 % | `git revert 3ef52fc` |
| D2 kurz mezi vzorky RTK z IMU | `heading_imu_interp` | **zapnuto** | 3 jízdy proti 3 výchozím: skoky kurzu mapy nad 0,3° 31–40 → 4–24, zamrznutí kotvy kurzu 5–7 → 2, zastaralé TF 3–10 % → 0,7–2,8 % | `git revert 91a24d7` |
| D3 dotáčení při předání podle EKF | `handover_follow_ekf` | **zapnuto** | série 4 a 5 (5 jízd proti 5): skoky kurzu mapy nad 0,3° 15–40 → 10–18 (v každém kole méně než výchozí), největší krok při předání 0,71–0,85° → 0,43–0,52°, nadbytečné otáčení 9–34° → 6–10°; rozkmit kurzu ve stoje beze změny (std 0,31–0,42° proti 0,36–0,47°) | `git revert 2502d6b` |
| D4 plynulý první zápis dokové pózy | `dock_engage_slew_mps` 0,10 | **zapnuto** | série 4 (2 proti 2): skok při zadokování 16,1 a 6,4 cm → 0,37 a 0,38 cm; 3,3 a 2,4° → 0,38 a 0,28° | `git revert 856fd6b` |
| E5 navsat ve vlastním vlákně | `navsat_async` | **zapnuto** | série 4 (2 proti 8): největší mezera `/tf` 0,95–1,36 s → 0,13–0,14 s | `git revert c998e8d` |
| E6 vyhlazení cíle kurzu | `heading_target_smooth` | vypnuto | série 4 a 5 (4 platné jízdy proti 5): skoky kurzu mapy 9–29 proti 15–40, rozkmit ve stoje 2,4–4,1° proti 2,8–5,7°; jednou ustálení po převzetí 65 s (jinde do 9 s) – příčina neznámá, doměřit a prověřit | – |
| Hlídač kurzu magnetometrem (T35) | `mag_check_enable`, `mag_disagree_deg` 15 | **zapnuto**, jen hlásí | zpětný přepočet jízd: 0 planých poplachů i při 10° | `git revert d1f73c9` |
| M1 hlídač kurzu i při výjezdu a příjezdu (T35) | `mag_check_wide` | **zapnuto**, jen hlásí | přepočet 18 jízd: pokrytí 6,9 → 70,7 % času mimo dok, 0 poplachů i při 6°; práh nesnižovat pod ~12° (rezerva mimo nahraná místa neověřená) | vrátit commit „M1 zapnuto“ v `navi_man.launch` (kód `3129059`) |
| M2 hlášení RTK kurz × magnetometr (T35) | `rtk_mag_report`, 6° / 0,3 s | **zapnuto**, jen hlásí (pole `rtk_mag_*`) | přepočet: příznak u 9 z 36 příjezdů, medián 0,8 s před koncem RTK | `git revert fe70f0e` (kód `0b205dd`) |
| F1 párování GPS odolné vůči posunu hodin (T36) | `gps_stamp_offset_track` | **zapnuto** | série 7 (2 jízdy proti 2, hodiny −0,68 s): nouzový režim kotvy 78–87 → 2,3 % času, kroky EKF nad 3 cm 30–45 → 5–11/min, odchylka podél dráhy −3 až −4 cm → 0 cm (po opravě razítek, v rámu odom); 0 planých skoků hodin, odhad posunu do ±0,024 s proti navpvt | `git revert f99b293` (kód `94ffd9a`) |
| F2 hlášení režimu kotvy a posunu hodin (T36) | `anchor_mode_report` | **zapnuto**, jen hlásí (pole `anchor_mode`, `anchor_starving`, `gps_clock_offset_s`); `nav_score` označí jízdu s neplatnými polohovými metrikami | – | vrátit commit „F2 zapnuto“ v `navi_man.launch` (kód `8d0fe28`) |
| G1 zpoždění kurzu GNSS proti IMU (T36) | `heading_latency_s` (0 = vypnuto; návrh 0,04) | vypnuto | přepočet 10 jízd (6 809 vzorků kurzu): chyba cíle kurzu při otáčení p95 4,1° → 2,3° (std 2,0 → 1,5°), ověřených vzorků do 5 s po otáčení 93,6 → 95,5 %; jízdou neměřeno | `git revert 6422167` |
| H1 TF s razítkem EKF (T36) | `tf_ekf_stamp` (+ `_max_age_s` 0,1; `_hold_s` 0,05) | vypnuto | odom→base_link a map→odom jdou jednou za zprávu navigačního EKF s jejím razítkem; po 0,05 s bez použitelné zprávy vysílá smyčka 30 Hz jako dosud. Ověřit jízdou s otáčením, couváním a dokem: v logu `[tf_ekf_stamp]` podíl ekf ≥ 99 %, loop jen při výpadku EKF; nepřibudou hlášky „extrapolation into the future“, „Could not transform the global plan“, „Cannot transform obstacles cloud“, TF_REPEATED_DATA/TF_OLD_DATA; doba dojezdu D4 a předání D3 beze změny. Kritériem není „odom proti kolům“ (EKF se s koly a gyrem rozchází až o 0,2–0,3 m i bez skoku, T10) | `git revert bcf5cab` |
| H2 hlášení jízdy naslepo (T36) | `anchor_freeze_report` (+ `_after_s` 10; `_min_dist_m` 0,5; `_dock_radius_m` 6; `_stop_s` 3) | vypnuto | jen hlásí: `loc_blind`, `loc_blind_near_dock`, `_reason`, `_s`, `_m`, `_dock_m`, `_count` v `/nav_tf/loc_status`, když robot ujel ≥ 0,5 m na kotvě kurzu nebo polohy starší 10 s, bez ověřeného kurzu nebo jen na odometrii. Ověřit jízdou s RTK: `loc_blind` nesvítí (ani v nouzovém režimu kotvy), u doku jen `near_dock`; po zakrytí GPS mimo dok se rozsvítí do ~10 s a 0,5 m, po zastavení do 3 s zhasne. Známé: s neznámým bodem DOCKED se `loc_blind` nerozsvítí ani daleko od doku | `git revert 80bd273` |
| J1 hlášení kroků pózy EKF (T36) | `odom_step_report` (+ `_xy_m` 0,01; `_yaw_deg` 3; `_window_s` 60; `_set_pose_s` 0,25; `_spike_mps` 1) | vypnuto | jen hlásí: pole `odom_step_*` v `loc_status` a událost na `/nav_tf/odom_step` (krok nevysvětlený koly a gyrem, příčina `set_pose:<cesta>` / `ekf_twist_spike` se zdrojem / `heading_anchor` / `unknown`); na nahrané jízdě magtest3 53 kroků/min, největší 5,1 cm; jízdou neměřeno. Ověřit jízdou: `odom_step_causes` a `odom_step_n` za minutu, hláška `[odom_step] nav-EKF twist spike` se zdrojem (hledá se původ špiček 1,3–3,5 m/s), CPU uzlu beze změny | `git revert d37e5c7` |
| J2 úhlová rychlost gyra do navigačního EKF (T36) | `nav_ekf_gyro` (soubor `config/ekf/ekf_nav_gyro.yaml` přes základní) | vypnuto | přehrání EKF nad `nav_s7_f1_1/2` (jen kola + `nav_fused`, bez VO/lidaru a bez `set_pose`): okna 0,4 s při otáčení EKF − gyro p95 2,2 a 2,6° → 0,7 a 1,5° (max 4,0 a 4,4 → 1,5 a 1,9°), zpoždění za gyrem 0,05–0,06 → 0,00–0,01 s; rovně proti `nav_fused` p95 0,73 a 0,55 → 0,65 a 0,65°; ve stoje 0,23 a 0,17 → 0,30 a 0,27°; jízdou neměřeno. PŘED zapnutím: otočka na místě a kontrola, že `nav_fused.angular_velocity.z` má stejné znaménko jako kola (C++ ovladač IMU). Ověřit jízdou s běžícím VO a lidarem (oba dávají `vyaw` také): otáčení na místě, oblouky, stání 2 min; okna 0,4 s EKF − gyro při otáčení p95 pod 1,5°, zpoždění pod 0,02 s, kurz ve stoje nejvýš o 0,1° neklidnější, žádné nové skoky kurzu mapy nad 0,3°, předání D3 beze změny | `git revert 253f4af` |

Past měření (15. 11.): běh `nav_s3_d3_1` jel ve skutečnosti s D2, měřicí skript restartoval cizí proces (hledání PID bez ukotvení). Skóre toho běhu se počítá k D2.

Série 4 (15.–16. 11., 10 jízd, výchozí = D1+D2): zátěž stroje 14–43 (první kolo souběžně s testy agentů), závěry jen z rozdílů shodných v obou kolech. Skóre `nav_s4_*`.

Série 5 (16. 11. 01:12–02:22, 9 jízd, výchozí = D1+D2+D4+E5, zátěž 15–39): běh `nav_s5_e6_2` neplatí (agent misi uzavřel bez jízdy jako už hotovou). Ve třetím kole u D3 i E6 výrazně víc kroků EKF nad 1 cm (93 a 55 /min proti 14–36) – výchozí jízda téhož kola je v normě, příčina neurčena. Skóre `nav_s5_*`.

Nález T35 (rozbor 18 jízd): „platný“ RTK kurz je v posledních 4–9 s před ztrátou u garáže (1,3–3,3 m od doku) mimo u 35 z 36 příjezdů (medián 6,3°, max 27°). Brána magnetometrem (M3, `rtk_mag_gate`) kontrolou NEPROŠLA a není v `main` (práh v šumu, plané zamítání při otáčení, cyklus při vadném magnetometru; patch odložen). Lepší kandidát k měření jízdou: delší `heading_anchor_lag_s` (dnes 2,5 s) – přepočet: 8 s → chyba kurzu při zadokování RMS 1,13°, 12 s → 0,90° proti dnešním 1,64°, bez zamítání; cena = každá korekce kurzu přijde později.

Nález T36 (16. 11.): systémové hodiny robota nejsou synchronizované a zpožďují se o 0,093 s/h proti UTC z GPS (`/gnss/fix` má razítko přijímače, ostatní uzly systémový čas). Jakmile rozdíl přeroste ~0,2 s (2–3 h po startu), vzorky GPS se přestanou ukládat do bufferu kotvy polohy a uzel jede v nouzové větvi `set_wheel_base_nav_pose()` 10× za sekundu (podélná pila ±2,5 cm, bez ochrany kotvy a rewindu). Polohové metriky jízd `nav_s5_*_3` (kromě base) a celé série 6 jsou proto neplatné; rozhodnutí o D3, D4, E5 se opírají o dřívější kola. Kořen = synchronizace času (chrony), F1 je pojistka.

`nav_score` v3 (16. 11.): staré `gps_gap_*` a `takeover_gap_*`/`takeover_settle_*` jsou při rozejitých hodinách zkreslené (pózu EKF hledají na razítku UTC a GPS převádějí přes `map→odom` z konce epizody) – zůstávají jen kvůli `--compare`, ve výpisu mají značku „stare*“. Platí oddíl C2: razítka GPS posunutá klouzavým mediánem (čas bagu − UTC) z `/gnss/navpvt`, mezera EKF − GPS v rámu odom jen ze vzorků s RTK fixem, rozklad podél dráhy (tam/zpět) a příčně: `gps_gap_odom_*`, `gps_along_*`, `gps_cross_*`, `takeover_*_odom_*`; skutečné plnění bufferu kotvy `anchor_buffered_share_*`. Přepočet všech jízd: `…/nav/score/v3/`. Zjištění z přepočtu: EKF je za GPS o posun hodin × rychlost už od ~0,1 s (−0,1 s → −2,5 cm podél dráhy v obou směrech, strop ~−4 cm v nouzovém režimu; s F1 0 ± 1 cm); „ustálení 65 s“ u E6 (`nav_s5_e6_1`) byl artefakt staré metriky (nově 2,4 s).

Rozbor IMU (T36, 16. 11.): `/bno085/imu` nese nová data 20× za sekundu ve 100 Hz zprávách, stáří hodnoty ve zprávě průměr 21 ms, p95 54 ms. Kurz GNSS je proti zprávám IMU zpožděný o ~45 ms (proti skutečným vzorkům IMU o ~65 ms), takže při otáčení cíl kurzu ujíždí o rychlost × zpoždění; zastaralé IMU tuto chybu z části zakrývá a samo ji nezpůsobuje. Ve stoje a při rovné jízdě (78 % vzorků kurzu pod 0,1 rad/s) je vliv pod 0,15°.

## 1. Rámce a kdo je píše

### 1.1 Transformace

| Transformace | Kdo publikuje | Jak často | Z čeho |
|---|---|---|---|
| `map → odom` | `navi_transform` (uzel `/navi_transform`) | 30 Hz, v jedné zprávě `/tf` s `odom → base_link`, stejné razítko (NT:3932-3968) | posun = `map_odom_transform.translation`, natočení = jen `map_odom_yaw` (NT:3250-3271); `z = −z(odom→base)`, takže výška v mapě je 0 (NT:3262) |
| `odom → base_link` | `navi_transform` | 30 Hz | poslední zpráva `/odometry/ekf_wheel_nav_odom` beze změny (NT:3079-3091); tatáž zpráva jde ven jako `/odometry/odom` (NT:3095) |
| `odom → utm` (statická) | `navsat_transform` | při startu navsatu | datum z prvního platného fixu, natočení z `/odometry/for_gps` (`config/ekf/navsat_transform.yaml:13-14`: `use_odometry_yaw true`, `wait_for_datum false`) |
| `base_link → gnss`, `gnss_heading`, `chassis`… | URDF | statické | antény (−0,16; ±0,21) m, rozteč 0,42 m napříč robotem [B `/tf_static`] |
| `map → dock` | `dock_detector` | při dokování | mapa doku ze skenu |

- Oba EKF mají `publish_tf: false` (EKF:16, 98); kola mají `enable_odom_tf: false` (`vitulus_base/config/base_common.yaml:52`). Jediný, kdo píše `map→odom` a `odom→base_link`, je `navi_transform`.
- Je-li `is_indoor == True`, `navi_transform` nevysílá ani jednu z obou transformací (NT:3071, 3243). Výchozí hodnota je `True` (NT:530); na `False` ji přepne `navi_man` přes `/navi_manager/is_indoor` (NT:1664). Po 30 s bez přepnutí uzel varuje (NT:3800).
- rtabmap jako lokalizace neběží [živě: jen kontejner `RTABMAP_ODOM_SYNC` pro hloubkové překážky]; `rtabmap_on_start: false` (`config/navi_manager.yaml:26`).

### 1.2 EKF (robot_localization, oba 30 Hz, `two_d_mode: false`, rámec `odom`)

Spouští je `navi_transform` při svém startu (NT:498, 1135-1140; `launch/vitulus_ekf_outdoor.launch`).

| EKF | Výstup | Vstupy | Kdo volá `set_pose` |
|---|---|---|---|
| `ekf_wheel_odometry` (EKF:10-55) | `/odometry/ekf_wheel_odom` | `odom0` kola: jen `vx, vy` (EKF:26-30); `imu0` `/bno085/imu`: roll, pitch, yaw **relativně** (EKF:43-51) | nikdo. Slouží jako čistá odometrie pro rozdíly (kotva, zkouška GPS, rozpočet trackeru). |
| `ekf_wheel_nav_odometry` (EKF:92-202) | `/odometry/ekf_wheel_nav_odom` → `/odometry/odom` → TF | `odom0` kola: `vx, vy`; `odom1` `/vo/odom`: `vx`, `vyaw` (EKF:149-159); `odom2` `/licp/odom_cov`: `vx`, `vyaw` (EKF:180-190); `imu0` `/gnss_heading/nav_fused`: roll, pitch, yaw **absolutně** (EKF:192-200) | `navi_transform`: kotva GPS 10 Hz (NT:2856, 3379), ruční `/nav_tf/set_pose` (NT:1175, 3304) |

- Do žádného EKF nevstupuje úhlová rychlost ani poloha z GNSS. Poloha z GNSS se do navigačního EKF dostává jen voláním služby `set_pose`.
- `set_pose` v robot_localization vymaže fronty měření a nastaví stav jen z pózy, rychlosti zůstanou nulové (`other/robot_localization/src/ros_filter.cpp`, `setPoseCallback`, ~2098-2130). Měřeno: v 18–23 % případů má vzorek hned po kroku > 1 cm přesně nulovou rychlost [B, M].
- Výška `z` nemá žádné měření; vzniká integrací `vx` přes náklon z IMU. Proč zůstává 3D, vysvětluje komentář EKF:2-9 (svahy, DR výška pro mapování).
- VO a lidarová odometrie jsou v doku pozastavené (`nodes/dock_idle`), venku do navigačního EKF vstupují jako rychlosti. Jejich podíl na výsledku: neověřeno.

### 1.3 Kurz v `odom → base_link`

- `fused = imu_yaw + diff_yaw` (NT:3656-3691), publikuje se s každou zprávou IMU (~97 Hz [B]) jako `/gnss_heading/nav_fused`. Do první kotvy (`init == True`) se místo `diff_yaw` přičítá `set_pose_yaw` (NT:3668).
- `diff_yaw` mění jen: kurz GNSS (NT:3508-3542) a `/nav_tf/set_heading` (NT:3585). Druhé nikdo nevolá [živě: `dock_set_heading false`, `vitulus_dock/launch/dock_detector.launch:34`].

**Invariant (platí):** absolutní kurz žije buď v `odom→base` (`diff_yaw`, GNSS), nebo v `map→odom` (`map_odom_yaw`: dok, tracker, relock), nikdy v obou. Každé převzetí kurzu GNSS proto `map_odom_yaw` dovede na 0 (NT:3479-3506) a georef čeká, dokud tam není (NT:1774-1791).

---

## 2. Zdroje polohy a kurzu a jejich přednost

### 2.1 Zdroje

| Zdroj | Téma | Rychlost [B] | Kam vstupuje | Brána |
|---|---|---|---|---|
| Kola | `/mobile_base_controller/odom` | 50 Hz | oba EKF (`vx`) | – |
| IMU BNO085 (bez magnetometru) | `/bno085/imu` | 97 Hz | relativní kurz, náklon | – |
| GNSS poloha (F9P) | `/gnss/fix` → navsat → `/odometry/gps` | 5 Hz | `set_pose` do navigačního EKF; georef `map→odom` | RTK fixed (`status == 2`) a kovariance xy ≤ 4e-4, z ≤ 1e-3 (NT:2595-2600) |
| GNSS kurz (druhý F9P) | `/gnss_heading/navheading` + `navrelposned` | 3,7 Hz průměr | `diff_yaw` | viz 2.3 |
| Dok | `/dock_smach/dock_status` (0 = v doku, podle napájení), bod `DOCKED` | 10 Hz | `map→odom` 30 Hz | viz 2.4 |
| Lidar tracker (gloc) | `/nav_tf/set_map_pose_tracker` | 5 Hz | `map→odom` plynule | jen bez použitelného fixu (`track_active_mode fix_lost`, GL:403), rozpočet driftu |
| Lidar relock (gloc) | `/nav_tf/set_map_pose_relock` | – | `map→odom` | **vypnuto** (`relock_enable false`, ML:58); zbývá ruční `/gloc/apply` |
| rtabmap | `/nav_tf/set_map_pose_rtabmap` | – | – | neběží |
| Magnetometr | `/bno085/mag_heading` | 7,7 Hz | nikam | jen publikuje; hlídač `mag_check_enable` **zapnut 15. 11.** v ML (práh 15°, jen hlásí do `loc_status`) (NT:2350) |
| VO, lidar ICP | `/vo/odom`, `/licp/odom_cov` | – | navigační EKF (rychlosti) | most do pózy vypnut (`~bridge/live_enable`, `~bridge/compute_enable` false, NT:2023, 2030) |

### 2.2 Vlastník pózy (`owner` v `/nav_tf/loc_status`)

`owner` je odvozený údaj pro pozorování (NT:2534-2545, 2291-2298), ne stavová proměnná. Vyhodnocuje se v tomto pořadí:

| Pořadí | `owner` | Podmínka | Co skutečně zapisuje |
|---|---|---|---|
| 1 | `gps` | `_gps_stable()` (NT:1206-1236) | kotva do EKF 10 Hz + georef `map→odom` |
| 2 | `dock` | `dock_status == 0` | držení doku 30 Hz (po 2 s) |
| 3 | `relock` / `tracker` / `rtabmap` / `legacy` | zdroj přijat v posledních 5 s (`map_pose_hold_s`, NT:842) | `callback_set_map_pose` |
| 4 | `dr` | nic z výše | nikdo: jen kola + IMU + poslední `diff_yaw` |

`loc_status` vychází 1 Hz (NT:3793) a navíc hned při změně vlastníka (NT:2547, voláno 10 Hz z NT:3929).

**`_gps_stable()` je pravda, když platí vše:**

| Podmínka | Kde | Časový limit / parametr |
|---|---|---|
| robot není v doku | NT:1224 | – |
| GPS prošla zkouškou (`_gps_proven`) | NT:1232, 2945-3029 | jízdou: ≥ 1 m podle kol a `|d_gps − d_kola| ≤ 0,1 m + 20 %` (NT:3016-3018); stáním: 10 s v klidu, rozptyl RTK ≤ 5 cm a kurz v pořádku (NT:2974-2997) |
| poloha použitelná souvisle ≥ 5 s | NT:1234-1235 | `map_coords_hysteresis_s` 5,0 |
| od získání fixu byl aspoň jednou použit kurz GNSS | NT:1236, 3542 | příznak `_heading_good_after_acq` se nastaví už při prvním dílčím kroku `diff_yaw` |

**Co stav shodí:**
- Výpadek použitelné polohy delší než 8 s (`gps_dropout_grace_s`, NT:2631-2644): zruší dobu „dobré" polohy, příznak kurzu i výsledek zkoušky. Kratší výpadek nic neshodí, ale kotva se nezapisuje (NT:3829 chce 3 použitelné zprávy po sobě) a georef ustoupí hned (NT:1761).
- Hrana výjezdu z doku (`dock_status` 0 → jiné): zruší zkoušku i dobu (NT:1255-1262). Důvod: odrazy pod střechou doku.
- Selhání zkoušky jízdou: `_gps_proven = False` (NT:3024-3029).

### 2.3 Kurz GNSS → `diff_yaw` (NT:3412-3459)

Vzorek `/gnss_heading/navheading` se přijme, když platí vše:
1. razítko vzorku je do **6 ms** od razítka poslední zprávy IMU (NT:3418, 3422);
2. `fix_usable` a `heading_fix_usable` (druhý přijímač: `status == 2`, kovariance pod pevnými mezemi NT:3033-3036);
3. `NAV-RELPOSNED` téhož přijímače: `carrSoln == 2`, kurz platný, stáří ≤ 0,3 s (NT:3390-3410);
4. rozptyl kurzu v pásmu 0,00043–0,00079 rad² (σ 1,19–1,61°; NT:3439, 648-649).

Cíl `target = rtk_yaw − imu_yaw + 90°` (NT:3453). Dál:
- **Kotva kurzu v2** (NT:3544-3583): cíl jde do zásobníku; použije se nejnovější vzorek starý ≥ 2,5 s, pokud okno 2,0 s před ním má ≥ 5 vzorků, rozpětí ≥ 1,0 s a největší odchylku od průměru ≤ polovina `heading_anchor_spread_max_deg`. Ten je v launchi **6,0** (ML:16), ne výchozích 1,5. Jinak `diff_yaw` zamrzne.
- **Použití** (NT:3508-3542): chyba ≤ 2° se zapíše naráz, větší po krocích `heading_slew_dps × 0,1` = 0,5° na vzorek. Během předání z doku (`map_odom_yaw ≠ 0`) vždy po krocích a `map_odom_yaw` se dotáčí kolem robota tak, aby součet `diff_yaw + map_odom_yaw` šel k cíli (NT:3479-3506).
- Úplně první kotva po startu uzlu (`init`) se zapíše naráz (NT:3454-3459). Držení doku `init` shodí (NT:1456), takže po startu v doku se už nepoužije.

### 2.4 Zápisy do `map → odom`

Všechny pod zámkem `_map_odom_lock` (NT:568).

| Kdo | Kdy | Jak | Řádky |
|---|---|---|---|
| Držení doku | `dock_status == 0` aspoň 2 s, čerstvé (< 5 s), bod `DOCKED` známý | každý tik 30 Hz: `map→odom = DOCKED ∘ inv(odom→base)`, celý kurz do `map_odom_yaw`. První zápis epizody je **skok bez omezení**; další odmítne nad 3 m. | NT:1395-1469 |
| `set_map_pose_*` | zpráva od zdroje | arbitr `dock > relock > tracker > rtabmap > legacy`: nižší zdroj se zahodí, byl-li vyšší přijat v posledních 5 s (NT:1494-1505). Kolem doku (stav 0/1/2 a 8 s po něm, i při chybějícím nebo starém `dock_status`) se nedokový zdroj zahodí nad 5 cm / 1° (NT:1264-1297, 1545-1567). Při otáčení > 0,3 rad/s se odloží (NT:1525). Párování s `odom→base` v čase zprávy (NT:1517). | NT:1471-1662 |
| – tracker | | rozpočet driftu: 0,15 m + 6 % ujeté dráhy + 0,003 m/° otočení, mínus už spotřebované; kurz 4° + 5 % otočení (NT:918-942, 1585-1613). Do 0,30 m plynule 0,15 m/s a 5°/s, do 0,6 m / 15° rychle 0,5 m/s a 15°/s, nad to skok (NT:1614-1634, 3185-3232). | |
| – dok, relock, ostatní | | naráz | NT:1635-1639 |
| Georef GPS | `/navi_manager/map_coords` nebo 1 Hz datum místa (NT:1720-1729), když `_gps_stable()`, poloha použitelná a `map_odom_yaw == 0` | cíl = `utm→odom` (TF) proti počátku mapy v UTM; posun plynule 0,10 m/s ve 30 Hz (NT:3158-3183); skok jen nad 1,0 m nebo na `map→odom`, které ještě nikdo neumístil | NT:1731-1886 |
| Dorovnání zbytku kurzu | zbytek ≤ 0,5°, nebo předání stojí 10 s | `map_odom_yaw → 0` kolem robota, ≤ 0,2° za tik | NT:1774-1791, 3135-3156 |

Rozpočet trackeru znovu naplní jen dok a georef GPS (NT:1465-1469, 1882-1885).

### 2.5 Kotva polohy GPS → navigační EKF (NT:3808-3929, smyčka 10 Hz se jménem `loop3sec`)

- Podmínka: 3 použitelné zprávy fixu po sobě, `_gps_stable()`, navsat běží, kurz v pořádku (nebo do 3 s po výpadku, NT:3043-3047) a `/odometry/gps` mladší 2 s (NT:3829-3837).
- Zásobník: jeden záznam na nový vzorek GPS, spárovaný s čistou odometrií a kurzem **interpolovanými na razítko GPS** (NT:3851-3866).
- Cíl = GPS(t − 2 s) + rozdíl čisté odometrie od té doby, otočený o `nav_yaw − wheel_yaw` (NT:2778-2830). Kotva postoupí jen na vzorek, u kterého pohyb GPS a kol za 1,5 s souhlasí (NT:2763-2776); jinak zůstane poslední ověřená.
- Zápis (NT:2832-2856): odchylka < 1,5 cm se nezapíše vůbec; větší se zapíše `set_pose`, nejvýš 5 cm na volání (`pos_slew_mps 0,5 × 0,1`). Výška zůstává z EKF (NT:1160-1168).
- Dokud zásobník nemá 2 s historie, zapisuje se přímo `rtk(teď)` se stejným omezením (NT:3885, 3341-3371).
- Navsat se spouští až tehdy, když jsou poloha i kurz čerstvé < 1 s (NT:1149-1157), tedy typicky při prvním získání GPS po startu uzlu. Při přechodu indoor → outdoor se navsat zastaví a spustí znovu s novým datem (NT:1668-1683).

---

## 3. Průběh typické jízdy

Čísla z [B] (dva výjezdy) a [M] (jeden).

| Krok | Co se děje s polohou a kurzem | Parametry | Měřeno |
|---|---|---|---|
| 1. V doku | `owner=dock`. `map→odom` se každý tik přepočítá z bodu `DOCKED`; póza v mapě stojí. Po čerstvém startu uzlu nese celý absolutní kurz `map_odom_yaw`. GPS se nepoužívá, tracker a ostatní jsou potlačeni. | `dock_anchor_*` (ML:26-28) | `map_odom_yaw` −15,7° [B], 16,4° [M] po čerstvém startu; −0,8° při druhém výjezdu [B] |
| 2. Výjezd | `dock_status` 0 → 1. Držení se pustí, `map→odom` zůstane stát, robot jede na kola + IMU. `owner` je `dock`, pak `dr`. Zkouška GPS začíná znovu. Tracker je potlačen (stav 1). | `tracker_dock_grace_s` 8 | `owner=dr` 5 s po hraně |
| 3. Čekání na RTK | Robot stojí před dokem. `dock_smach` UNDOCK i mise čekají na `/nav_tf/odom_status`: `SAT` a stáří polohy i kurzu < 3 s (`vitulus_dock/nodes/dock_smach:1220, 1274-1283`; ST:206-270, rozpočet 600 s + 900 s). | – | RTK fix polohy za 41 s, 45 s a **374 s** od hrany (přijímač 366 s ve FLOAT [B]) |
| 4. Převzetí | Po fixu: 3 zprávy → navsat se spustí (při prvním převzetí) → kurz GNSS začne posouvat `diff_yaw` a `map_odom_yaw` jde k 0 → zkouška GPS (stáním 10 s nebo 1 m jízdy) → 5 s hystereze → `owner=gps`. Pak kotva táhne EKF k GPS (≤ 0,5 m/s) a georef posouvá `map→odom` (0,1 m/s). | `map_coords_hysteresis_s`, `stationary_probe_s`, `heading_slew_dps`, `pos_slew_mps`, `georef_slew_mps` | fix → `owner=gps`: 7,7 s, 8,3 s, 14,4 s; předání kurzu 12,8 s / 16,1° [B], 14,3 s / 16,4° [M]; mezera před převzetím 6,5–6,9 cm, dorovnání 0,2–10,2 s [B] |
| 5. Jízda na GPS | Kotva 10 Hz zapisuje do EKF kroky 1,5–5 cm, kurz se opravuje každým přijatým vzorkem. `map→odom` je konstantní (dané datem navsatu). | `anchor_lag_s` 2, `anchor_deadband_m` 0,015 | `|fúze − GPS|` medián 1,8 cm, p95 17 cm [B]; 83–93 kroků EKF > 1 cm za minutu jízdy na GPS |
| 6. Výpadek | Poloha nepoužitelná: kotva přestane hned, georef ustoupí, po 5 s smí tracker. `owner` zůstává `gps` do 8 s, pak `dr`. Návrat: nová zkouška + 5 s. | `gps_dropout_grace_s` 8 | přepnutí `dr↔gps` 25×/h [B], 17×/h [M] |
| 7. Zadokování | `dock_status` 2: tracker potlačen. Robot couvá na kontakty podle reflektoru nebo mapy doku. Napájení → `dock_status` 0 → po 2 s první zápis držení doku naráz. | `dock_anchor_debounce_s` 2 | skok 19,0 cm / 0,8° a 17,1 cm / 2,3° [B], 23,8 cm / 2,0° [M], vždy v jednom tiku |

---

## 4. Plánování a řízení

### 4.1 Co běží

- Řetěz: `vitulus.service` → `vitulus/launch/vitulus_start.launch` → `navi_man.launch` (navi_man, navi_transform, gloc_server, dock_idle, mower_unit_smach, smach_exe_path, smach_exe_goal). Navigační zásobník (`vitulus_navi_outdoor.launch` → `vitulus_navi_mbf.launch`) spouští `navi_man` za běhu (`nodes/navi_man:2180-2205`).
- `move_base_flex`: plánovač `GlobalPlanner` (`config/planners.yaml:5-6`), řadič `TebLocalPlannerROS` (první v `config/controllers.yaml:9-12`), obnova jen `ClearCostmapRecovery` (`config/recovery_behavior.yaml:1-7`). Žádný klient plugin nevybírá. `TrajectoryPlannerROS` je registrovaný a nepoužitý.
- Řadič 15 Hz, `controller_patience` 6 s, `oscillation_timeout` 10 s / 0,2 m (`controllers.yaml:17-22`); plánovač 1 Hz, `planner_patience` 5 s (`planners.yaml:10-11`).

### 4.2 Globální plán

- **Sečení:** dráhy vyrábí `vitulus_planner/nodes/node_planner` v rámci `map` (rovnoběžné řádky + obrys). Globální plánovač po nich nejede: mise dráhu převzorkuje po 0,03 m (ST:557-611) a posílá ji po oknech 200 póz přímo do `exe_path`.
- **GlobalPlanner** (Dijkstra, `config/global_planner_params.yaml`) se volá jen pro přejezd na začátek zóny a dráhy, objížďku, dosečení a jízdu na pojmenovaný bod (dok, goto přes `smach_exe_goal`).

### 4.3 TEB (`config/base_teb_local_planner_params.yaml`)

- Rychlosti přepisuje `navi_man` podle profilu (`nodes/navi_man:1501-1572`). [živě] profil SLOW: `max_vel_x` 0,25, `max_vel_theta` 0,9, `acc_lim_x` 0,01, `acc_lim_theta` 0,015.
- `xy_goal_tolerance` 0,3 m, `yaw_goal_tolerance` 0,3 rad (:127-128); `min_obstacle_dist` 0,5, `inflation_dist` 0,6 (:67, :55); `global_plan_viapoint_sep` 0,1, `weight_viapoint` 480 (:50, :125); `max_global_plan_lookahead_dist` 2,5 (:59); `odom_topic /odometry/odom`, `map_frame map`, `transform_tolerance` **0,05 s** (:80, :58, :102).

### 4.4 Costmapy

| | Globální (`config/global_costmap_params.yaml`) | Lokální (`config/local_costmap_params.yaml`) |
|---|---|---|
| Rámec | `map` | `odom`, klouzavé okno 7×7 m, 0,05 m |
| Obnova | 1 Hz | 10 Hz, publikace 6 Hz |
| Vrstvy | statická `/navi_manager/map`; voxely bez paměti: `/rplidar/scan_5m` (3 m) a `/obstacles_cloud` (3,5 m); inflace 0,85 m | statická mapa; `/local_costmap/cloud_map`; `/local_costmap/lidar_map`; inflace 0,5 m |
| `transform_tolerance` | 0,3 s [živě] | 1,0 s |

- `/obstacles_cloud` vyrábí `vitulus_depth_obstacles` z hloubky D435 (dosah 2,8 m).
- `/local_costmap/cloud_map` (`vitulus_local_costmap`) je pevná mřížka 20×20 m s počátkem (−10, −10) v `odom`, která se neposouvá (`src/cloud_costmap_layer.cpp:18-19, 37-38`). Dopad dál než 10 m od počátku `odom`: **neověřeno** (kandidát na vadu, viz backlog 16).
- Vrstva sjízdnosti (`local_costmap_traversability.yaml`) [živě] načtená není.

### 4.5 „No goal progress", re-window, objížďky

Vše v `ExecutePathWithFeedback` (ST:~1018-1226), limity v `config/mower_smach.yaml:81-88`:

| Mechanismus | Podmínka | Výsledek |
|---|---|---|
| Bez pokroku k cíli + překážka | `dist_to_goal` se 3 s nezlepšil o 5 cm a `check_path_cost` hlásí překážku | zrušit, objížďka (ST:1091-1098) |
| Bez pokroku k cíli | 6 s bez zlepšení | ujeto ≥ 0,30 m → re-window (nové okno z aktuální polohy); jinak NAV_RECOVERY (ST:1099-1111) |
| Stání | 5 s bez pohybu > 8 cm | totéž rozhodování |
| Blízko konce kusu | `dist_to_goal < 0,6 m` | re-window |
| Zámek muxu | hold aktivní | zrušit, počkat, re-window |

- V hlášce „no goal progress %.1fs … re-window" je čas od začátku kusu (`elapsed_d`), ne doba bez pokroku (ST:1099-1104).
- NAV_RECOVERY: nůž vypnout, 6 s, `clear_costmap`, opakovat; pak couvnutí 0,4 m; po 2 pokusech objížďka (`recovery.py:293-534`). Objížďka zkouší napojení 1,5 / 2,5 / 4 / 6 m dál; po 5 přeskocích po sobě mise končí (MI:1155-1192).
- **Index postupu po dráze jde jen dopředu:** `current_index = max(current_index, prev_index)` (ST:722-724, 1321-1323), hledá se v okně −5…+400 póz. Poloha se bere z TF `map→base_link`.
- [S]: 195 re-window na 2 444 m = 8,0 na 100 m; na jedno připadá průměrně 7,1 s.

### 4.6 Mux rychlostí a kdo robota zastaví

Mux `vitulus_base/nodes/cmd_vel_mux`: `ds4` 100 > `teleop` 90 > `primitive` 50 > `nav` (`/cmd_vel`) 10 (MUX:96-102), výstup `/mobile_base_controller/cmd_vel`. Bez příkazu zastaví kola `cmd_vel_timeout` 0,25 s.

| Kdo | Kdy | Jak |
|---|---|---|
| Člověk (páka DS4, teleop) | kdykoli | převezme mux, zruší cíle MBF, zastaví misi a dok (MUX:417-433) |
| HOLD muxu | `/cmd_vel_mux/hold` (mower_link, gnss_watchdog; ten je vypnutý) | nuly 10 Hz, bez časového limitu, přežije restart |
| LIMIT muxu | `map_rules_guard`: zakázaná zóna, nebo TF `map→base_link` starší než limit | ořez, případně nula |
| `safety_node` | motory bez dat 2 s, porucha, > 70 °C | vypne napájení motorů |
| `safety_node` | náklon > 30° | vypne **jen nůž**, pohon jede dál |
| GlobalStop | `/mower_smach/stop` | nůž, mise, dok |

Nárazník, zvednutí a tlačítko STOP v softwaru nenalezeny (hardwarový STOP: neověřeno).

---

## 5. Konzumenti polohy a co jim uškodí

| Konzument | Co čte | Co mu uškodí |
|---|---|---|
| Mise sečení: index postupu, okna, „stání", objížďka | TF `map→base_link` | **Skok** vpřed po dráze: index přeskočí a nevrátí se, úsek zůstane neposečený. Skok vytvoří falešný „pohyb" nebo „pokrok". Za jízdy mise nemá bránu na skok pózy ani ztrátu GPS; lokalizaci kontroluje jen před startem (WAIT_FOR_GPS_FIX). |
| TEB | `/odometry/odom` (rychlost), TF `map→odom→base_link` | Propad rychlosti v odometrii po `set_pose`; kroky pózy 1,5–5 cm (honí je); TF starší než 0,05 s → ztracený cyklus řadiče. |
| Globální costmapa, plán v `map` | TF | skok `map→odom` posune překážky bez paměti i robota vůči plánu |
| Lokální costmapa (`odom`) | TF `odom→base_link` | skok `map→odom` jí nevadí; kroky v `odom` (kotva GPS) posouvají uložené překážky |
| `map_rules_guard` | TF `map→base_link` | zpoždění TF → nulové rychlosti; drift → chybně vyhodnocená zóna |
| `dock_smach`, `docking_controller` | TF, `/odometry/odom` | cíl výjezdu je zamčený v `odom` (`vitulus_dock/nodes/docking_controller:830-846`): krok kurzu v `odom` během výjezdu ho pootočí |
| gloc tracker | `/nav_tf/loc_status`, TF | zapíná se podle `fix_usable`; špatná póza = špatný start hledání |
| Mapování (`map_world_recorder`, `terrain_recorder`, `mapping_manager`) | TF, `loc_status`, `z` odometrie | skok pózy = rozmazaná vrstva; drift výšky = křivý terén |
| UI, agent | `loc_status`, `odom_status`, TF | jen zobrazení a rozhodování agenta |
| Uložené body (`DOCKED`, waypointy) | póza v okamžiku uložení | uložení při nesouhlasu dok/GPS zapeče chybu do každého dalšího zadokování |

---

## 6. Parametry, které chování řídí

[živě] má `/navi_transform` nastaveno jen 10 parametrů z ML:16-40; vše ostatní běží na výchozích hodnotách z kódu.

| Parametr | Hodnota | Co dělá | Kde |
|---|---|---|---|
| `fix_cov_xy_max` / `fix_cov_z_max` | 4e-4 / 1e-3 m² | práh přesnosti fixu polohy | NT:517-518 |
| `gps_probation_travel_m` | 1,0 | dráha zkoušky GPS jízdou | NT:620 |
| `probation_tol_abs_m` / `_frac` | 0,1 / 0,2 | tolerance GPS proti kolům (zkouška i ověření kotvy) | NT:831-832 |
| `stationary_probe_s` / `_max_scatter_m` | 10 / 0,05 | zkouška GPS ve stoje | NT:631-633 |
| `map_coords_hysteresis_s` | 5,0 | souvislá dobrá poloha před převzetím | NT:612 |
| `gps_dropout_grace_s` | 8,0 | výpadek, který ještě neshodí GPS | NT:817 |
| `heading_grace_s` | 3,0 | tolerovaný výpadek fixu druhého přijímače | NT:514 |
| `heading_var_min` / `_max` | 0,00043 / 0,00079 | přijímané pásmo rozptylu kurzu | NT:648-649 |
| `heading_require_relpos_fixed`, `relpos_max_age_s` | true, 0,3 | kurz jen při RTK fixu kurzu | NT:659-661 |
| `heading_anchor_v2`, `_lag_s`, `_window_s`, `_min_samples`, `_min_span_s` | true, 2,5, 2,0, 5, 1,0 | zpožděná ověřená kotva kurzu | NT:677-683 |
| `heading_anchor_spread_max_deg` | **6,0** (launch) | rozptyl okna kotvy kurzu | ML:16 |
| `heading_slew_dps` | 5,0 | rychlost dorovnání kurzu (0,5° na vzorek) | NT:823 |
| `anchor_lag_s` | 2,0 | zpoždění kotvy polohy | NT:707 |
| `anchor_deadband_m` | 0,015 | menší odchylka se nezapisuje | NT:829 |
| `pos_slew_mps` | 0,5 | největší krok kotvy = 5 cm na volání | NT:824 |
| `rtk_fresh_s` | 2,0 | stáří `/odometry/gps` pro kotvu | NT:836 |
| `georef_slew_mps` / `georef_kidnap_m` | 0,10 / 1,0 | posun `map→odom` k GPS / skok | NT:795-797 |
| `georef_max_map_yaw_deg`, `georef_stall_s` | 0,5 / 10 | dorovnání zbytku kurzu `map→odom` | NT:587-601 |
| `map_pose_hold_s` | 5,0 | okno arbitru zdrojů | NT:842 |
| `map_pose_slew_*`, `map_pose_gentle_below_m`, `map_pose_slew_snap_m/deg` | 0,5 m/s, 15°/s; 0,30 m → 0,15 m/s, 5°/s; 0,6 m / 15° | plynulost oprav trackeru | NT:870-898 |
| `tracker_budget_*` | 0,15 m, 6 %/m, 0,003 m/°, 4°, 5 % | rozpočet driftu trackeru | NT:918-927 |
| `tracker_dock_grace_s` | 8,0 | potlačení po dokovém manévru | NT:1030 |
| `set_map_pose_max_wz` | 0,3 rad/s | při otáčení se oprava odloží | NT:821 |
| `dock_anchor_enable` / `_debounce_s` / `_max_hold_m` | true / 2,0 / 3,0 | držení doku | ML:26-28 |
| `dock_gate_non_dock`, `dock_micro_m/deg` | true, 0,05 / 1,0 | potlačení ostatních u doku | ML:32-34 |
| `dock_status_fail_closed`, `_stale_s` | true, 5,0 | bez stavu doku se opravy zahazují | ML:38-39 |
| `rewind_enable`, `bridge/live_enable`, `bridge/compute_enable` | false | vypnuté větve | NT:749, 2023, 2030, 2350 |
| gloc `relock_enable` | **false** (launch) | automatický relock | ML:58 |
| gloc `track_rate_hz`, `track_active_mode` | 5, `fix_lost` | kdy tracker opravuje | GL:135, 403 |
| EKF `frequency`, `two_d_mode` | 30, false | – | EKF:11-13, 93-95 |
| navsat `use_odometry_yaw`, `wait_for_datum` | true, false | datum = první fix + kurz fúze | `navsat_transform.yaml:13-14` |
| mise `exe_goal_stall_time`, `exe_goal_stall_obstacle_s`, `exe_stall_time`, `exe_progress_eps` | 6,0 / 3,0 / 5,0 / 0,30 | „no goal progress", stání | `config/mower_smach.yaml:82-88` |
| MBF `controller_frequency`, TEB `transform_tolerance` | 15 / 0,05 | – | `controllers.yaml:17`, TEB:102 |

Pevně v kódu, bez parametru: 6 ms mezi kurzem a IMU (NT:3422), prahy fixu druhého přijímače (NT:3033-3035), 2° pro zápis kurzu naráz (NT:3526), 3 zprávy po sobě (NT:3829).

---

## 7. Známé slabiny a příčiny

### 7.1 Výchozí skóre (`tools/nav_score.py`, 2026-11-15)

| Metrika | [B] baseline | [M] sečení |
|---|---|---|
| délka / dráha / jízda | 880 s / 36 m / 3,8 min | 836 s / 104 m / 9,1 min |
| vlastník pózy (% času) | dr 58, dock 23, gps 18 | gps 81, dock 10, dr 8 |
| `map→odom`: největší krok polohy / kurzu za tik | 19,0 cm / 2,26° | 23,8 cm / 2,00° |
| `map→base`: kroků > 1 cm / > 3 cm | 162 / 30 | 762 / 130 |
| `map→base`: kroků kurzu > 0,3° (mimo otáčení) | 542 | 476 |
| kroky EKF > 1 cm / > 3 cm na minutu jízdy | 36,9 / 6,8 | 75,4 / 12,8 |
| propad rychlosti v `/odometry/odom` za jízdy | 2,0 % | 5,3 % |
| změn vlastníka; `dr↔gps` za hodinu | 12; 24,6 | 8; 17,2 |
| mezera před převzetím GPS (medián / max) | 6,7 / 6,9 cm | není v nahrávce |
| doba dorovnání po převzetí (medián / max) | 6,8 / 10,2 s | není v nahrávce |
| `|fúze − GPS|` při `owner=gps` (medián / p95) | 1,8 / 17,0 cm | není v nahrávce |
| předání kurzu: doba / otočení / nadbytečné otáčení | 12,8 s / 16,1° / 40,4° | 14,3 s / 16,4° / 47,3° |
| kolísání kurzu ve stoje na GPS (sm. odch. / rozkmit) | 0,37° / 2,77° | 0,43° / 2,83° |
| kolísání polohy ve stoje na GPS (p95) | 10,8 cm | 4,7 cm |
| skok při zadokování (za tik) | 19,0 cm / 2,26° | 23,8 cm / 2,00° |
| od opuštění doku: RTK fix / `owner=gps` / rozjezd | 374 / 382 / 397 s | 45 / 60 / 79 s |
| stání během výjezdu | 345 s | 36 s |
| stáří kotvy polohy (p95 / max), zamrznutí | 3,8 / 11,3 s, 1 | 0,09 / 26,7 s, 1 |
| stáří kotvy kurzu (p95 / max), zamrznutí | 8,9 / 14,3 s, 6 | 4,7 / 29,6 s, 5 |
| drift výšky EKF / rozsah | −0,04 / 0,11 m | +0,47 / 0,67 m |
| největší mezera v `/tf` | 1,11 s | 1,36 s |
| opakovaná póza v `/tf` za jízdy | 10,5 % | 10,5 % |
| „no goal progress", re-window | není v nahrávce (chybí `/rosout`) | není v nahrávce |

U [B] jsou časy po opuštění doku maximum ze dvou výjezdů; druhý výjezd: 41 / 50 / 73 s, stání 28 s.

### 7.2 Slabiny s příčinou

| # | Slabina | Číslo | Příčina v kódu |
|---|---|---|---|
| S1 | Póza v `odom` cuká na GPS | 83–93 kroků > 1 cm za minutu jízdy na GPS, medián kroku 1,8–1,9 cm, p95 4,5 cm [B, M] | Kotva zapisuje `set_pose` až od 1,5 cm, tedy každá oprava je krok ≥ 1,5 cm (NT:2845-2856). `set_pose` navíc nuluje rychlosti EKF. |
| S2 | Kurz ve stoje kolísá | rozkmit 2,8°, sm. odch. 0,4° [B, M] | Každý přijatý vzorek kurzu (σ 1,33° [B]) do 2° se zapíše do `diff_yaw` naráz (NT:3526); kotva v2 vybírá jeden vzorek, neprůměruje. |
| S3 | Polovina vzorků kurzu se zahodí | brána 6 ms projde u 49–52 % vzorků [B, M] | Razítko kurzu se porovnává s poslední zprávou IMU (100 Hz, náhodná fáze; NT:3418-3422). Důsledek: řidší okno kotvy v2, pomalejší předání, zamrzání. |
| S4 | Předání kurzu po čerstvém startu cuká | 12,8–14,3 s, nadbytečné otáčení 40–47° při potřebných 16° [B, M] | `map_odom_yaw` se dotáčí hned, EKF dobíhá `diff_yaw` se zpožděním; kroky 0,5° po vzorku (NT:3479-3536). |
| S5 | Skok při zadokování | 17–24 cm, 0,8–2,3° v jednom tiku [B, M] | První zápis držení doku je bez omezení (NT:1428-1440). Velikost = nesouhlas bodu `DOCKED` s GPS + drift za 40–50 s jízdy na `dr` před dokem. |
| S6 | `/tf` stojí při prvním převzetí GPS | 1,11 s a 1,36 s, přesně v okamžiku `owner → gps` [B, M] | `roslaunch_navsat()` se volá z 30Hz smyčky, která vysílá TF (NT:3971 → 1142-1147). |
| S7 | Před prvním převzetím neexistuje `/odometry/gps` | první zpráva 1,6 s po `owner=gps` [B] | Navsat se spouští až při první čerstvé poloze + kurzu (NT:1149-1157); kotva do té doby nemá cíl. |
| S8 | `/tf` opakuje starou pózu | 10,5 % tiků za jízdy [B, M] | TF 30 Hz bere poslední zprávu EKF 30 Hz z jiné smyčky a dává jí čas „teď" (NT:3079-3095, 3945); chyba až v × 33 ms. |
| S9 | Dlouhé čekání po výjezdu | 60–79 s do rozjezdu; z toho RTK fix 41–45 s a fix → `owner=gps` 8–14 s [B, M]. V [B] první výjezd 374 s jen čekání na fix (přijímač ve FLOAT). | Zkouška GPS a hystereze se na hraně výjezdu ruší (NT:1255-1262) a kurz GNSS má vlastní zpoždění 2,5 s + okno. Doba do fixu je věc příjmu, ne softwaru. |
| S10 | Kotvy zamrzají | kurz až 29,6 s, poloha až 26,7 s [M] | Kotva kurzu: okno neprojde (málo vzorků kvůli S3 a pásmu rozptylu). Kotva polohy: pohyb GPS a kol nesouhlasí. Robot jede dál na poslední kotvě; mise o tom neví. |
| S11 | Drift výšky | +0,47 m za 14 min sečení [M]; −0,04 m [B] | `z` v EKF nemá měření (EKF:26-30, 193-197); integruje se `vx` × náklon. |
| S12 | Re-window při sečení | 8,0 na 100 m, 7,1 s na jedno [S] | „No goal progress" 6 s (ST:1099); podíl příčin (konec kusu, cukání pózy, TEB) nerozlišen: neověřeno. |
| S13 | Časté přepínání `dr↔gps` | 17–25 za hodinu [B, M] | Výpadek fixu > 8 s shodí vše a návrat stojí zkoušku + 5 s. |
| S14 | Mise za jízdy nehlídá lokalizaci | – | Lokalizaci čte jen WAIT_FOR_GPS_FIX před startem; index postupu jde jen dopředu (ST:722-724). |
| S15 | Smyčka kotvy může viset | – | `rospy.wait_for_service` bez limitu v 10Hz časovači (NT:3338). |
| S16 | `_heading_good_after_acq` naskočí při prvním dílčím kroku kurzu | – | NT:3542; georef je ale od T7d chráněn čekáním na `map_odom_yaw == 0`. Navsat může vzít datum s kurzem, který ještě dobíhá: dopad neověřen. |

### 7.3 Mrtvé a vypnuté větve

- `rewind_enable false`: starý splice po výpadku, nahrazen průběžnou kotvou (NT:747-749, 2701-2761).
- Most VO/lidar do pózy (`~bridge/*`, NT:1948-2270, 2561-2581): vypnutý i jeho výpočet. Rychlosti z VO a ICP ale do navigačního EKF vstupují dál (EKF:149-190).
- `relock_enable false` (ML:58; teleport 4,68 m 2026-07-19). Následek: robot ztracený bez GPS se sám nenajde.
- rtabmap: kanály `set_map_pose_rtabmap`, `rtabmap_confident`, `/rtabmap/initialpose` existují, nikdo je neplní.
- `/nav_tf/set_heading`, `/nav_tf/set_dock_pose` (NT:1180, 3585): bez volajícího [živě].
- `gnss_watchdog` vypnutý (`vitulus_gnss/launch/vitulus_ardusimple_gnss.launch:28`); `mag_check_enable` od 15. 11. zapnutý (jen hlásí).
- `src/vitulus_navi/smach_states/*` nikdo neimportuje. `TrajectoryPlannerROS`, parametry DWA/Neo/Graceful, blok `laser_layer` (`local_costmap_params.yaml:29-46`): načtené, nepoužité.
- `dock_smach:722` `# TODO: Implement lidar map navigation` (stav vrací úspěch po 1 s).
- Zavádějící jména: `loop3sec` běží 10 Hz (NT:3808-3814); uzel se v kódu hlásí jako `GNSS_node`, v grafu je `/navi_transform`.

### 7.4 Co ze starších podkladů už neplatí / stále platí

| Tvrzení | Stav |
|---|---|
| Paměť 10-17: „NT:2645 a NT:2713 nulují `map_odom_yaw` bez přepočtu posunu a bez zámku → skok metry po výjezdu" | **Neplatí.** Od T7d se `map_odom_yaw` mění kolem robota a pod zámkem (NT:3465-3506). Zbylo cukání S4. |
| Paměť 10-17: „georef skáče při prvním volání" | **Neplatí.** Skok jen na neumístěný `map→odom` (NT:1845). |
| Paměť 10-17: čísla řádků NT | **Neplatí** (soubor má 4 006 řádků). Invariant kurzu, seznam vypnutých větví a zámek indexu **platí**. |
| NT:982-986 komentář „IMU nemá magnetometr, absolutní kurz jen zvenku" | Platí pro `/bno085/imu`; magnetometr se čte zvlášť (`mag_heading`) a nikam nevstupuje. |
| Rozbor 11-15: skok při zadokování 23,8 cm / 2,0°, otočení 16,4°, cukání 14,6 s, +59 s do GPS, výpadek kotvy kurzu 29,6 s | **Potvrzeno** [M]: 23,8 cm / 2,00°, 16,4°, 14,3 s, 59,8 s, 29,6 s. |
| Rozbor: „757 kroků EKF > 1 cm za 615 s" | **Potvrzeno řádově**: 683 kroků > 1 cm za 8,2 min jízdy na GPS [M] (rozbor měl práh 0,8 cm a jinou referenci kol). |
| Rozbor: „`set_pose` nuluje rychlosti" | **Potvrzeno** v kódu robot_localization i daty (propad rychlosti 2–5 % vzorků za jízdy). |
| Rozbor: „drift výšky +0,55 m za 14 min" | **Částečně**: konec − začátek +0,47 m, rozsah 0,67 m [M]. |
| Rozbor: „`dr↔gps` až 30× za hodinu" | **Částečně**: 17 a 25 za hodinu v obou nahrávkách; 30 neověřeno. |
| Rozbor: „no goal progress 195× na 2 444 m" | **Potvrzeno** [S]; jde o události `rewindow`. |
| Rozbor: „nesoulad dok/GPS/lidar 14–54 cm" | **Částečně**: skok při zadokování 17–24 cm [B, M]; 54 cm neověřeno. |
| Zadání: „D1–D4 už běží" | **Neplatí pro živý robot.** D1–D4 jsou commity ve větvi `t36-d` (`09b2090`, `60d4c10`, `9278038`, `41dacd9`), vše výchozí vypnuto; živý uzel běží z `main` @ `30eca80` a parametry nemá nastavené [živě]. | **Stav 15. 11. 12:35:** D1–D4 jsou v `main`; v ML zapnuto D1 (`3ef52fc`) a D2 (`91a24d7`), D3 a D4 vypnuté do změření.
| Paměť: „restart navi_man = sirotek depth_obstacles", „catkin stub" | Neověřováno (mimo rozsah); ber jako platné pasti. |

---

## 8. Co je ověřené a čím, a co ne

**Testy v `tools/` (čisté funkce, bez robota):**

| Test | Co hlídá |
|---|---|
| `test_t7d_yaw_handover.py` | předání kurzu kolem robota, georef čeká na nulu |
| `test_t28_carr_gate.py`, `test_t28_z_keep.py` | kurz jen při RTK fixu kurzu; výška z EKF |
| `test_t35_mag_check.py` | hlídač magnetometru |
| `test_tf_atomic.py` | obě transformace v jedné zprávě |
| `test_anchor_helpers.py` | interpolace historie pro párování kotvy |
| `test_dockfix_gates.py` | držení doku, potlačení, rozpočet trackeru |
| `test_georef_rotation.py`, `test_georef_status.py` | georef v osách mapy, stav georefu |
| `test_hold_gate.py`, `test_stop_preempt.py`, `test_global_stop.py`, `test_iterator_preempt.py`, `test_mission_undock_gate.py` | mise: zámek, stop, výjezd |
| `test_nav_score.py` (32 testů) | metriky skóre |

Pozor: testy fieldu a navi pouštět po souborech.

**Polní data:** [B], [M], [S] viz kap. 7.1. Obě nahrávky jsou po čerstvém startu uzlu (první výjezd s `map_odom_yaw` ~16°).

**Neověřeno:**
- jízda s D1–D4 zapnutými (žádná nahrávka);
- chování při ztrátě GPS uprostřed sečení na delší dobu (tracker jako jediný zdroj) v listopadových datech: tracker vlastnil pózu 0,6 % času;
- podíl VO a ICP na navigačním EKF;
- dopad pevné mřížky `cloud_map` dál než 10 m od počátku `odom`;
- zda navsat bere datum s nedoběhlým kurzem (S16);
- latence proudu korekcí mezi přijímači, skutečná konfigurace přijímačů;
- hardwarový STOP, nárazník;
- příčiny re-window po jednotlivých případech (S12);
- kapitoly 4 a 5 nebyly měřeny, jen čteny.

---

## 9. Backlog zlepšení

Seřazeno podle přínosu a rizika. „Vratná" = jde udělat jako jeden commit s vlastním vypínačem, nezávisle na ostatních. Čísla „dnes" z kap. 7.1.

| # | Úprava | Dnes → cíl | Riziko | Místo | Vratná | Stav |
|---|---|---|---|---|---|---|
| 1 | **D2** kurz GNSS párovat s IMU interpolovaným na razítko vzorku | přijato 50 % vzorků → ~100 %; předání 12,8–14,3 s → ~7 s | nízké | NT:3418-3459 | ano (`heading_imu_interp`) | ve větvi `t36-d`, vypnuto, v poli neměřeno |
| 2 | **D3** při předání dotáčet `map→odom` podle skutečného natočení EKF | nadbytečné otáčení 40–47° → < 5° | střední (zásah do vysílaného TF) | NT:3479-3506, 3234 | ano (`handover_follow_ekf`) | ve větvi `t36-d`, vypnuto |
| 3 | **D4** první zápis držení doku rozložit plynule | skok 17–24 cm za tik → < 1 cm za tik | nízké | NT:1428-1440 | ano (`dock_engage_slew_mps`) | ve větvi `t36-d`, vypnuto |
| 4 | **D1** rychlosti v `/odometry/odom` brát z kol | propad rychlosti 2–5 % → 0 % | nízké až střední (TEB čte rychlost odtud) | NT:3093-3095 | ano (`odom_twist_from_wheels`) | ve větvi `t36-d`, vypnuto |
| 5 | Spouštět navsat mimo smyčku TF (vlákno, nebo hned při startu s čekáním na datum) | mezera `/tf` 1,1–1,4 s → < 0,1 s | nízké | NT:1142-1157, 3971 | ano | v `main` (1f14df6), vypnuto, `navsat_async`; roslaunch na odděleném masteru: mezera smyčky 0,80–0,86 s → 0,04 s; jízdou neměřeno. Před trvalým zapnutím: navsat zastavený během startu může zůstat běžet bez vlastníka (díra je i ve starém kódu), opakování po chybě nemá strop |
| 6 | Vyhladit cíl kurzu (průměr ověřeného okna místo jednoho vzorku; malé chyby neaplikovat naráz) | rozkmit ve stoje 2,8° → < 1°; kroků kurzu > 0,3°: 476–542 → < 50 | střední (zpoždění kurzu v zatáčkách) | NT:3526, 3556-3571 | ano | v `main` (bc5652f), vypnuto, `heading_target_smooth`; přehrání 3 jízd: kroků > 0,3° 147–515 → 22–79, rozkmit ve stoje 1,8–2,7° → 0,75–1,7°; jízdou neměřeno. Sledovat: změna odchylky pod 1° se dorovnává ~12 s místo 2,5 s (při rychlém driftu gyra pila do 1°), podíl „smoothed / old way“ v logu |
| 7 | Kotvu polohy zapisovat bez kroků (malý podíl každý tik, nebo opravu nést v `map→odom` a EKF nechat spojitý) | kroky EKF > 1 cm: 83–93/min na GPS → < 5/min | vysoké (jádro lokalizace; druhá varianta mění, co je `odom`) | NT:2832-2856, 3337-3383 | ano, za vypínačem | nové, nejdřív návrh |
| 8 | TF `odom→base_link` vysílat s razítkem zprávy EKF (z callbacku EKF) | opakovaná póza 10,5 % → 0 % | střední (TEB `transform_tolerance` 0,05 s, časování konzumentů) | NT:3064-3097, 3932-3968 | ano | nové |
| 9 | Zkrátit fix → `owner=gps` po výjezdu (zkoušku jízdou započítat už z výjezdu, hystereze od fixu) | 8–14 s → < 5 s | střední (past odrazů u doku, incident 2,4 m 2026-07-05) | NT:1255-1262, 2945-3029 | ano | nové |
| 10 | **D5** přelít kurz doku do `diff_yaw` ve stoje před výjezdem | otočení při předání 16° → 0° | střední až vysoké (invariant kurzu, cíl výjezdu v `odom`) | NT:1436, 3585-3612 | ano | odloženo |
| 11 | Zaměřit bod `DOCKED` z průměru GPS a hlídat nesouhlas | posun při zadokování 17–24 cm → < 5 cm | nízké (data, ne kód), nutná jízda | waypoint `DOCKED`, NT:1321-1355 | ano | nové |
| 12 | Rozbor a úprava „no goal progress" (podle příčin; opravit i čas v hlášce) | 8,0 re-window / 100 m → < 2 | střední | ST:1080-1111, `mower_smach.yaml:85-88` | ano | nové, nejdřív měření |
| 13 | Zamrznutí kotev hlásit misi (zpomalit / zastavit po N s) | max 29,6 s kurz, 26,7 s poloha → reakce do 10 s | střední (falešná zastavení) | NT:2299-2326; MI | ano | nové |
| 14 | Výška: ukotvit `z` (pomalé tažení k nule ve stoje nebo `two_d_mode` jen pro navigační EKF) | drift +0,47 m / 14 min → < 0,1 m | střední (terénní mapa čte `z` z `odom`) | EKF:92-97 | ano | nové |
| 15 | Stráž indexu postupu proti skoku pózy | kroků `map→base` > 3 cm: 30–130 za jízdu; přeskočené úseky dnes neměřeny | nízké | ST:722-724, 1321-1323 | ano | nové |
| 16 | Ověřit a případně posouvat mřížku `cloud_map` s robotem | neověřeno → jasná odpověď | nízké (ověření), střední (oprava) | `vitulus_local_costmap/src/cloud_costmap_layer.cpp:18-38` | ano | nové |
| 17 | Časový limit na `wait_for_service` v 10Hz smyčce | 0 → limit 1 s | nízké | NT:3338 | ano | nové |
| 18 | Zkrátit dorovnání po převzetí (rychlejší georef při malé mezeře) | 6,8–10,2 s → < 3 s | nízké | NT:3158-3183 | ano | nové |
| 19 | Nahrávat pro skóre i `/rosout_agg` a `/gnss/fix` | 3 metriky „není v nahrávce" → 0 | žádné | recept nahrávání | ano | nové |
| 20 | Zapnout hlídač magnetometru (jen hlásí) | 0 nezávislých kontrol kurzu → 1 | nízké | NT:2347-2500, launch | ano (`mag_check_enable`) | v `main`, vypnuto |

---

## 10. Jak dokument udržovat

1. **Kdy aktualizovat:** v témže commitu jako každá změna `navi_transform`, EKF, gloc, mise, costmap nebo launch parametrů. Změní-li se řádky, oprav odkazy v dotčené kapitole; změní-li se chování, oprav kapitolu 2, 3 a 6.
2. **Jedna úprava = jeden řádek backlogu.** Po nasazení přepiš „Stav", po polní jízdě doplň změřené číslo a přesuň slabinu z 7.2 do „vyřešeno" s datem.
3. **Čísla jen ze skóre nebo s uvedeným zdrojem.** Co není změřeno ani přečteno v kódu, označ „neověřeno".
4. **Jak pustit skóre** (bez ROS mastera, ~20 s na 260MB nahrávku):
   ```
   source /opt/ros/noetic/setup.bash; source ~/catkin_ws/devel/setup.bash
   nice -n 15 python3 tools/nav_score.py JIZDA.bag --json-out VYSLEDEK.json
   python3 tools/nav_score.py --compare PRED.json PO.json
   python3 tools/test_nav_score.py
   ```
   Definice metrik jsou v docstringu `tools/nav_score.py`. Výchozí stav: `/home/vitulus/.claude/jobs/9e12fd1c/tmp/nav/score/nav_baseline_2026-11-15_a.json` a `magtest3_2026-11-14.json`.
5. **Srovnatelnost jízd:** porovnávej stejnou trasu (dok → TEST → dok) a stejný stav uzlu. První výjezd po startu uzlu má předání kurzu ~16°, další ne. Časy do RTK fixu závisí na příjmu, ne na softwaru; hodnoť rozdíl fix → `owner=gps`.
6. **Nahrávka pro plné skóre** musí obsahovat: `/tf`, `/tf_static`, `/nav_tf/loc_status`, `/odometry/odom`, `/odometry/ekf_wheel_nav_odom`, `/odometry/gps`, `/mobile_base_controller/odom`, `/bno085/imu`, `/gnss/navpvt`, `/gnss_heading/navrelposned`, `/dock_smach/dock_status`, `/move_base_flex/exe_path/result`, `/rosout_agg`.
7. **Kapitolu 7.1 přepiš** vždy, když se změní výchozí stav v `main` (nová baseline nahrávka), a starou tabulku nech v historii gitu.
