# Lokalizace v2 — jednotná, robustní, indoor+outdoor (plán 2026-07-10)

_Vzniklo z 4-agentního auditu kódu + živého systému (arbitr, EKF vrstva,
gloc+mapping v3, dock stack). Vše níže ověřeno proti kódu s file:line,
ne proti starým poznámkám. Navazuje na STAV_2026-07-06.md; nahrazuje
rtabmap-centrický UNIFIED_LOCALIZATION_PLAN.md (2026-06-21) jako aktuální
plán — sekce o docku a relokalizaci z něj zůstávají platné jako reference._

## Cíl (zadání uživatele)

Přesná, robustní a spolehlivá lokalizace indoor i outdoor, plynulé přechody.
GNSS-RTK je finální hlavní určení polohy, když má kvalitní signál; vše je
přepočítáno na něj (georef/datum). Wheel odometrie = krátkodobá pravda,
koriguje výstřelky VO/ICP/GNSS. VO + lidar-ICP + scan-to-map nahrazují GNSS
tam, kde není signál. Dock = absolutní kotva bez GPS.

## Ověřený současný stav (souhrn auditu)

> **STAV PO 2026-07-11 (deploy WP1–WP4 + review).** Níže popsaný „audit" stav
> je z velké části PŘED-WP2; tento odstavec shrnuje živě ověřenou realitu po
> nasazení a review 2026-07-11 (nepřepisuje historii níže). Ověřeno živě
> proti běžícímu robotu (v doku, idle):
> - **EKF (nav):** odom0=/mobile_base_controller/odom (thr **50**),
>   odom1=/vo/odom (thr **2.45**, fúzuje vx+vyaw), odom2=**/licp/odom_cov**
>   (thr **2.45**, vx+vyaw) — souvislé, bez děr; imu0=/gnss_heading/nav_fused.
>   Práh 1.0σ z auditu je NAHRAZEN 2.45 (chi2 0.95, 2 DOF). Živé rosparamy =
>   soubor. odom2 má PRÁVĚ JEDEN živý publisher (`licp_odom_cov_fix` relay);
>   cov floored: twist cov[0]=2.8e-4, cov[35]=1.5e-4 (ověřeno na zprávě).
> - **Arbitr (navi_transform):** JE zaveden — source-tagged set_map_pose
>   (dock/tracker/rtabmap/legacy) s prioritou v callbacku (NE last-writer-wins),
>   `_map_pose_hold_s`=5 s; pořadí DOCKED > GPS(_gps_stable) > tracker > rtabmap
>   > DR. `/nav_tf/loc_status` (JSON, ~1 Hz), `/nav_tf/bridge_status.gps_good`
>   = `_gps_stable()` + `fix_usable`. G1/G2 z auditu = VYŘEŠENO.
>   rtabmap_confident = jen observability (mimo pózovou cestu).
> - **site_map (WP1):** `/mapping_manager/site_map` EXISTUJE, latched
>   OccupancyGrid (garden2_v1, 384×512 @ 0.05 m); gloc jej konzumuje
>   (map_topic v navi_man.launch). G3 z auditu (site_map neexistuje) = NEPLATÍ.
> - **gloc tracker:** track_defer_to_rtabmap=True → gated OFF (živě
>   applied=false, state=poor_match); NEZAPISUJE set_map_pose_tracker.
> - **Dock:** rtabmap→póza bridge VYPNUT (bridge_enable=false default + log
>   „BRIDGE DISABLED"); guard cross-check /dock_manager/is_in_dock_confirmed
>   (confirm_fresh_s=15, confirm_timeout_s=10); waypoint „docked" přítomen.
> - **Datum symlink:** `~/.vitulus/saves/site_datum.yaml` → garden2/datum.yaml
>   (= servírovaná mapa), navi_transform jej aplikuje jen bez živého map_coords.
>   REVIEW FIX 2026-07-11: `link_site_datum` se nyní volá i ze serve cesty
>   (`_publish_site_map`), ne jen z mapping start — servírování jiného site už
>   nenechá datum ukazovat na dřív mapovaný site (garden vs garden2 = 0.23 m/0.9°).
> - **Úklid:** živě odregistrovány mrtvé zombie uzly (`odom_cov_fix` starý
>   název relaye, `ekf_navigation_node`, stray tf_echo) přes `rosnode cleanup`.
> - **respawn:** kritické uzly (navi_man, navi_transform, gloc_server, dock_*,
>   mapping_manager, webnode) = respawn=true. licp/VO/relay = BEZ respawn
>   (EKF se ale sám odtlumí na staleness — degradace, ne pád) — doporučení.

**Vrstva 1 (odom→base, spojitá):** `ekf_wheel_nav_odometry`
(config/ekf/ekf_base_outdoor.yaml:84) fúzuje wheel twist (Mahalanobis 50σ)
+ VO twist (/vo/odom, ~5 Hz) + licp twist (/licp/odom, ~10 Hz; oba práh 1.0σ)
+ fúzovaný heading (/gnss_heading/nav_fused). `ekf_wheel_odometry` = čistá
wheel+IMU reference, nikdy se nekoriguje (rewind ji používá jako deltu).
TF odom→base vysílá navi_transform (30 Hz), NE robot_localization.
POZOR: produkční launch řetěz je vitulus_start.launch → navi_man/navi_transform
si roslaunchují EKF/VO/licp/rtabmap Z PYTHONU; vitulus_bringup XML strom je
z velké části mrtvý (ekf_outdoor.yaml, ekf_base*.yaml, ekf_global.yaml,
navsat.yaml = orphan).

**Vrstva 2 (map→odom + diskrétní korekce):** navi_transform (1728 ř.) —
GPS-first arbitr: RTK pozice přes set_pose do nav EKF (10 Hz smyčka),
probation (1 m jízdy, shoda Δgps/Δodom), heading pásmo var 0.00043–0.00079,
rewind splice (2 s, cooldown 20 s), slew. map→odom translace z georefu,
yaw z map_odom_yaw. Bez GPS: korekce přes /nav_tf/set_map_pose od DVOU
nesynchronizovaných zapisovatelů — dock rtabmap bridge
(dock_localization_seed:335) a gloc EDT tracker (gloc_server:300).
Všechny parametry arbitru = code defaults (nic v launch/rosparam).

**EDT tracker (gloc_server):** scan-to-EDT GN 1 Hz, brány rms<0.20,
corr<0.7 m/6°, eig degenerace, min 60 bodů; mapa = /rtabmap/grid_map;
track_defer_to_rtabmap=True. Živě 10.7.: state=corr_too_big, corr 0.81 m/10°,
rms 0.15 — tracker vidí ~0.8m offset pózy, ale neaplikuje (brána + defer).

**Dock:** cold-start seed automatický (latched dock_status==0 →
get_point_pose("docked") → /rtabmap/initialpose + set_heading/set_map_pose),
ALE: waypoint "docked" se tvoří ručně a nic nevynucuje jeho existenci
(chybí → tichý no-op); seed_docked věří jen power signálu (force=True, bez
guardu, necheckuje /dock_manager/is_in_dock_confirmed); legacy cesta přes
waypoint "dock" (approach bod!) žije paralelně; seed_detected (korekce z
vidění doku za jízdy) je napsaný, ale default OFF a nevalidovaný;
AlignScanInMap (/odometry/dock_odom) je zcela osiřelý subsystém.

## Konsolidované mezery (číslované, dle závažnosti)

G1. **gps_good má dvě sémantiky**: /nav_tf/bridge_status publikuje
    bool(position_fix_usable) (navi_transform:686), ale skutečné vlastnictví
    pózy řídí _gps_stable() (docked/probation/hysteresis/heading). Downstream
    (dock bridge, gloc) se řídí tou slabší. Navíc přenos jako substring
    "gps_good=1" v human-readable Stringu.
G2. **Dva zapisovatelé /nav_tf/set_map_pose bez arbitráže** (dock bridge
    1.5 s + EDT tracker 1 Hz) — last-writer-wins na map_odom_yaw; dock bridge
    neustupuje trackeru.
G3. **EDT tracker závisí na /rtabmap/grid_map**; /mapping_manager/site_map
    neexistuje ani v kódu (pgmio má jen save, žádný loader); ŽÁDNÝ site nemá
    uložený rastr ani garden.ot (jen dem.npz z testů 6.7.).
G4. **is_indoor = tvrdý kill-switch TF** (navi_transform:965,1016) — indoor
    přechod je útes, ne blend.
G5. **seed_docked bez guardu** (power-only, force=True, tight cov) +
    nevynucený waypoint "docked" + duplicitní legacy "dock" cesta.
G6. **rtabmap_confident se v navi_transform čte, ale nikde nepoužívá**
    (mrtvě zapojený pivot).
G7. **Zdravotní stav zdrojů roztříštěný**: VO/licp bez status topicu (jen
    covariance[0], dvě různé implementace gatingu — EKF Mahalanobis vs
    bridge), wheel covariance statická lež (base_common.yaml:8-9), vel kanál
    wheel odom neprochází continuity guardem (jen pos[]), /bno085/imu_status
    nikdo nekonzumuje.
G8. **RTK pozice vstupuje jen přes set_pose kroky** (bespoke logika v 1700 ř.
    Pythonu), ne jako EKF měření — funkční, ale mimo nativní gating.
G9. **Bootstrap problém garáže**: mapa pro korekci driftu pod střechou se
    staví z pózy, která tam právě driftuje (insertion_gate fused mód nemá
    absolutní inovační bránu).
G10. **Magické konstanty natvrdo** (heading grace 3 s, probation tolerance
    0.1+0.2·d, bridge brány, datum okno 10 s, rtk freshness 2 s…) — ladění
    v poli = editace kódu.
G11. Mrtvé konfigy/launche (viz výše) — riziko editace mrtvého kódu.
G12. Band 0.10–0.60 m vs výška roviny lidaru se nikde nekontroluje.

## Cílová architektura

Dvouvrstvá struktura ZŮSTÁVÁ (je správně):

- **Vrstva 1 — spojitá lokální odometrie (nikdy neskáče):** nav EKF fúzuje
  wheel+IMU+VO+licp twisty; každý zdroj dostane jednotné zdraví (freshness,
  covariance, cross-check vs wheel). Wheel = krátkodobá kotva.
- **Vrstva 2 — JEDEN arbitr absolutní pózy (navi_transform):** explicitní
  priorita vlastníků: **DOCKED > GPS(proven) > EDT tracker > rtabmap bridge >
  dead-reckoning**. Korekce vstupují source-tagged, arbitr rozhoduje (ne
  last-writer-wins). Jeden typový stavový topic (vlastník, gps_good=_gps_stable,
  zdraví zdrojů) místo string-parsingu.
- **Mapa pro scan-to-map = mapping v3 rastr** (site_map server), rtabmap
  postupně OUT z pózové cesty (zůstane dočasně jako fallback, pak vypnout).
- **Georef:** vše kotveno na UTM přes datum.yaml (už funguje) — mapy stavěné
  v RTK módu jsou tím „přepočítané na satelity".
- **Dock:** hardened absolutní kotva — guard přes is_in_dock_confirmed,
  vynucený/auto-zachycený waypoint "docked", jednotná seed cesta; později
  seed_detected (vidění doku za jízdy) po validaci.

Přístup: **chirurgické úpravy, ne přepis** — navi_transform obsahuje polně
zkalibrované chování (heading pásmo, probation, rewind), to se nesmí ztratit.

## Fáze a pracovní balíčky

### Fáze 1 — kód, bez pole (agenti, zálohy před každou změnou, import-test)
- **WP1 site_map server** (Sonnet): pgmio loader (pgm+yaml→OccupancyGrid),
  mapping_manager publikuje latched /mapping_manager/site_map pro zvolený
  site/rastr (standalone, bez octomap_server); gloc ~map_topic přepínatelný.
  Akceptace: gloc tracker žere site_map, stav ok/aligned na syntetice.
- **WP2 arbitr — sjednocení sémantiky a arbitráž** (Opus):
  (a) bridge_status gps_good = _gps_stable() + nový typový topic
  /nav_tf/loc_status (JSON nebo msg: owner, gps_good, zdroje);
  (b) source-tagged set_map_pose (dock/tracker/rtabmap) + priorita v
  callbacku; (c) rtabmap_confident buď zapojit, nebo vyhodit;
  (d) magické konstanty → ~params (beze změny defaultů!).
  Akceptace: chování při dobrém GPS bit-identické; bez GPS vyhrává dock,
  pak tracker, pak rtabmap.
- **WP3 dock guard** (Sonnet): seed_docked cross-check is_in_dock_confirmed
  (s timeoutem/fallbackem), warning při chybějícím "docked" waypointu do
  loc_status, sjednotit/odstranit legacy "dock" set_dock_pose cestu.
- **WP4 úklid** (Sonnet): mrtvé EKF konfigy/launche označit/odstranit,
  README aktualizovat na skutečný launch řetěz.

### Fáze 2 — pole (uživatel + dohled přes probes)
- Zmapovat zahradu VČETNĚ garáže (Fused mód; garáž krátkými průjezdy
  začínajícími/končícími v RTK kvalitě kvůli G9), Snapshot → rastr + .ot.
- Přepnout gloc na site_map, polní test trackeru pod střechou (REC),
  očekávání: drift 0.9 m → cm–dm.
- Pak track_defer_to_rtabmap=false → rtabmap mimo pózovou cestu.

### Fáze 3 — plynulost a dock
- is_indoor: zrušit TF kill-switch (TF vysílat vždy; is_indoor jen pro
  navsat lifecycle + costmapy). Ověřit navsat restart edge-cases.
  POZOR (nález WP4, 10.7.): indoor režim spouští vitulus_navi_indoor.launch
  → ekf_indoor.launch → samostatný robot_localization s publish_tf: true
  (navi_man:793-797, přepíná se suffixem mapy ***env*INDOOR). Kill-switch
  v navi_transform existuje PROTO — indoor má jiného vlastníka TF.
  Sjednocení = převést indoor na stejný ekf_base_outdoor stack + arbitr,
  ne jen odstranit podmínku. Detaily: doc/LAUNCH_CHAIN.md.
- Validovat a zapnout seed_detected (bearing→orientace, standoff offset).
- Zvážit zapojení AlignScanInMap jako druhé dock kotvy.

### Vyšetřování driftu pod střechou (2026-07-11, 3 agenti: bag+kód+živé měření)

VERDIKT: drift 0.4–0.9 m/průjezd je HEADING-dominantní (cross-track), ne prokluz.
Řetěz: pod střechou heading = čistá IMU integrace od posledního RTK anchoru
(diff_yaw zamrzlý, navi_transform:1457/1616), drift ~0.7°/min za jízdy
(v klidu jen 0.025°/min — bag 22-17 + still bag), cross-track 15 m × 3° ≈ 0.8 m.
Ty viditelné 0.4–0.6 m „skoky" = splice korekce při RTK reacquisition (symptom).

**REWIND v2 — motion-verified anchor + rotace delty do map frame (2026-07-11,
nasazeno).** Uživatelská polní diagnóza symptomu „skoků" výše, ověřená na bagu
`~/rosbags/loc_eval/mapfree_drive_2026-07-11_17-23-32.bag`. Dvě vady starého
splice `pose = rtk(T_a) + [wheelEKF(now)−wheelEKF(T_a)]`:
- **(A) rotace delty:** `ekf_wheel_odometry` free-runuje od bootu a nikdy se
  GPS-nekotví; delta ruší jeho pozdíl pozice, ale NE jeho heading drift vůči map
  frame → wheel delta je pootočená o naakumulovaný nesoulad wheel-EKF-vs-map
  (bag: 9–14° na 1–4 m pohybu → 0.4–1.0 m chyba splice). FIX: buffer nově ukládá
  per tick wheel-EKF yaw a live nav-pose (map) yaw; při splice se delta pootočí
  o `theta = nav_yaw(T_a) − wheel_yaw(T_a)`.
- **(B) motion-verified anchor:** kvalita GPS ocasu NELZE soudit z kovariance
  (multipath drží těsnou cov, přitom pozice utíká) — jediný platný rozhodčí je
  shoda pohybu odometrie. FIX: místo pevného `bad_since − rewind_s` se jde
  ZPĚTNĚ od výpadku (krok `~rewind_verify_step_s`=0.5 s přes `~rewind_verify_s`=5 s)
  a anchor = NEJNOVĚJŠÍ vzorek, kde `|Δgps| ≈ |Δwheel|` na trailing sub-okně
  (probation tolerance 0.1+0.2·d). Držela-li shoda až k výpadku → anchor ≈ live →
  ~žádný skok; utekl-li ocas (multipath) → anchor tam, kde byl GPS naposledy
  motion-konzistentní → zkažený ocas se opravdu ustřihne; nic neverifikuje →
  fallback na staré chování (warn).
Nové ~params (defaulty bezpečné): `~rewind_verify_s`=5.0, `~rewind_verify_step_s`
=0.5, `~rewind_verify_subwin_s`=1.5; reuse `~probation_tol_abs_m/frac`. Všechny
staré guardy (min ocas, max 2 m, cooldown 20 s, grace 8 s, slew) zachovány. Log:
`anchor age, verified/fallback, theta deg, splice distance`.

Rekonstrukce na mapfree bagu (5 událostí, časy sedí na 17:27:11.6 / 28:43.8 /
30:27.2 / 30:51.8 / 32:03.2 — přesná shoda s forenzikou):

| # | čas | OLD splice | NEW splice | theta | anchor |
|---|-----|-----------|-----------|-------|--------|
| 1 | 17:27:11.6 | 0.37 m | 0.05 m | −11.8° | verified |
| 2 | 17:28:43.8 | 0.57 m | 0.10 m |  −8.8° | verified |
| 3 | 17:30:27.2 | 0.75 m | 0.08 m | −14.0° | verified |
| 4 | 17:30:51.8 | 1.04 m | 0.07 m | −13.2° | verified |
| 5 | 17:32:03.2 | 0.50 m | 0.17 m | −10.6° | verified |

Všech 5 motion-verified (ocasy byly motion-konzistentní → skutečný skok patřil
téměř celý rotaci delty, ne ustřižení ocasu). Multipath případ (bag
`gnss_2026-07-06_22-17-24`, jiná sada topiců, bez `/gnss/fix` → plná replay
trigger neproveditelná): GPS-vs-wheel motion shoda drží 201/202 sub-oken S
POHYBEM — walk zůstane na dobrém vzorku; degradovaný ocas u multipath je typicky
při MALÉM pohybu (těsná cov, pozice utíká) → sub-okno s pohybem tam neverifikuje
a anchor se nezvolí uvnitř zkaženého ocasu (referee = pohyb, ne self-report).

Nálezy s čísly:
1. HEADING (hlavní): cross-track >> along-track ve všech RTK segmentech
   (head-eq 6–17°); scale/prokluz jen ~3 % (|v|/gSpeed medián 1.03).
   KOREKCE (2. agent, flag-decode navrelposned): navheading NEBYL rozbitý —
   cov=1000 jen 10.5 % zpráv (RTK konvergence ~90 s + STÁNÍ: u-blox heading
   invaliduje při isMoving=false; za jízdy 325 zpráv validních, acc 1.44°).
   Hardware OK. Doporučení: (i) zálohovat receiver flash config do YAML
   (config_on_startup:true — teď je flash jediný zdroj pravdy), (ii) alert jen
   na „invalid ZA JÍZDY", (iii) zdroj RTCM korekcí base přijímače nezdokumentován.
2. FÚZOVACÍ BRÁNA (reálná chyba konfigurace, byť v tomto bagu ne dominantní):
   odom1/2_twist_rejection_threshold=1.0 = 1σ joint gate na 2 DOF (vx,vy)
   → zahazuje 61 % licp vzorků i při dokonalé shodě (chi2, ověřeno ve zdrojáku
   robot_localization filter_base.cpp:388); při prokluzu ≳15–20 mm/s zahazuje vše
   (přesně kdy je licp potřeba). Fix: threshold ~2.45 (chi2 0.95) + vy nefúzovat
   (nonholonomní; 1 DOF gate). ekf_base_outdoor.yaml:139,151.
3. LICP kvalita: yaw šum 1.4°/s std (2σ=2.8 — proto se yaw nefúzuje), publikovaná
   cov ~3–10× podhodnocená (vx skutečně 28.5 mm/s std vs deklarovaných 8.4).
   Příčina: bez deskewingu (10 Hz rotační lidar, 100 ms smear), PointToPlane=false,
   MaxCorrespondenceDistance=0.1 těsný. Fix licp config → pak lze fúzovat i vyaw
   (stabilizace headingu bez GPS). POZN: „bias −45 mm/s v klidu" (jeden vzorek)
   VYVRÁCEN 120s měřením (mean 0.065 mm/s) — je to šum, ne bias.
4. Rozdíl EKF drah v bagu (13–16°) NENÍ důkaz přínosu licp — obě EKF mají JINÝ
   heading zdroj (wheel: raw /bno085/imu bez kotvy; nav: nav_fused s RTK kotvou).
5. VO v noci mrtvé (cov 9999) — korektně vyřazeno, ale licp je pak single point
   of failure pro DR.
6. nav_fused orientation cov statická 0.025 (σ≈9°) — neroste s časem od kotvy;
   zvážit růst pod denialem (přesnější váha při reacquisition).

Pořadí oprav: (a) brána 1.0→2.45 + vx-only [levné, bezpečné, kód hned];
(b) licp deskewing/PointToPlane/corr-dist [kód hned, pole ověřit];
(c) prověřit dual-antenna navheading výpadek [diagnostika];
(d) po (b): fúzovat licp vyaw; (e) dlouhodobě: EDT tracker (Fáze 2) = absolutní
kotva headingu i pozice — řeší kořen, (a)–(d) zmenšují chybu mezi kotvami.

**AKCE 2026-07-11 — rtabmap→póza bridge VYPNUT (rozhodnutí uživatele).**
Bag garden2_drive_2026-07-11_13-33-35: pod střechou (GPS-denied) rtabmap
korigoval live pózu **77×** jako map_pose_source. Kvantifikace z bagu:
- V DOKU a FYZICKY NEHYBNĚ (0–70 s) fúzovaný senzorový odhad (VO+licp+wheel)
  seděl ustáleně **~1.56 m (1.52–1.60 m)** od live map pózy, kterou bridge řídil
  → celý ten ~1.56 m je čistá dezlokalizace vlitá do live pózy (smeared garáž
  v rtabmap mapě), robot se přitom nehýbal.
- Při rtabmap ownershipu (72–77 s) → předání DR: `div_from_live` spadl z ~1,4 m
  na ~0,2 m během ~8 s, jak fúzovaná dráha stáhla pózu zpět = rtabmap držel pózu
  ~1,4 m mimo. (Globální peak div 6,08 m v DR segmentu je heading/RTK-splice, ne
  rtabmap — viz výše.)
ZMĚNA: `~bridge_enable` default → **false** v kódu (dock_localization_seed) +
explicitně `bridge_enable=false` v launch/dock_detector.launch. Zdroje pózy nyní
= **fúze odom EKF (wheel+IMU+licp+VO) + GPS-RTK (proven) + DOCK seed only**;
rtabmap už pózu NEZAPISUJE. PONECHÁNO: dock one-shot seed (/rtabmap/initialpose —
rtabmap smí seed PŘIJÍMAT), /nav_tf/rtabmap_confident (jen observability).
EDT tracker (defer/poor_match, gated OFF) zůstává plánovaná budoucí map-kotva
(Fáze 2) — NE rtabmap. Re-enable bridge až po de-smeared mapě + tracker anchoru.

### Fáze 4 — zdraví zdrojů a sezónnost
- Jednotný health modul (wheel slip cross-check, imu_status, VO/licp
  staleness) → loc_status + gating.
- Band vs lidar výška check (G12); sezónní refresh rastru (re-drive→compare).

## Pravidla práce
- Před každou editací .bak_<label>_<timestamp> záloha (konvence repa).
- Python nody v catkin běží přes exec-relay stuby — editovat src, NIKDY
  necpat cp přes stub.
- Žádný restart služby bez souhlasu uživatele; napřed cmd_vel/motory check.
- Po fázi 1: commitnout (v repu je i necommitnutá práce z 6.7.!).
