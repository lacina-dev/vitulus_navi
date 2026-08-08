# Analýza validační jízdy 2026-08-02 (bag) — cukání na GPS + plavání mapy na trackeru

Bag: `~/rosbags/loc_eval/validacni_jizda_2026-08-02_22-19-28.bag` (29,5 min).
Analyzováno 2026-08-07. Skripty: scratchpad session (jerk_analysis.py, jump_structure.py,
drive_trace.py, latency_check.py) — postup reprodukovatelný z tohoto dokumentu.

## A) „Na GPS se robot dost cukal" — ROZŘEŠENO, kauzální řetězec kompletní

### Co bag ukazuje

- Za GPS ownershipu skáče póza nav EKF (`/odometry/ekf_wheel_nav_odom`) o **40–95 mm
  v jednom 30Hz kroku**, 2–4× za sekundu (rezidua vůči wheel odom: RMS 10–11 mm/krok
  vs. 0,3–1,5 mm na trackeru/DR). RTK je přitom **100 % FIXED, hAcc 14 mm** — kvalita
  GPS za to NEMŮŽE.
- Skoky chodí v **10Hz taktu** (interval medián 0,100 s) a v **89 % střídají znaménko**;
  98 % jich je podélných (ve směru jízdy). Oscilační index 0,05 → 95 % pohybu korekcí
  se vzájemně vyruší (čistá pila, žádné užitečné dotahování).
- V klidu (v=0) póza stojí perfektně; občas jediný diskrétní skok ~80 mm (posun kotvy).
- Surový navsat výstup `/odometry/gps` je hladký (medián rezidua 6 mm) — šum vzniká
  až v korekční cestě.
- Nav EKF jede systematicky **~57 mm ZA** navsat pozicí (p90 161 mm) ≈ v × latence.

### Mechanismus (kód + čísla)

1. `loop3sec` (běží 10 Hz, navi_transform:2358) každý tik appenduje do rewind bufferu
   pár (`rtk_odom_msg`, `ekf_wheel_odom_msg`) — tj. **poslední přijatou** GPS pozici
   s **čerstvou** wheel pózou (navi_transform:2395).
2. Jenže `/odometry/gps` chodí 5 Hz a v okamžiku 10Hz tiku je jeho poslední zpráva
   **96–196 ms stará** — tik od tiku se stáří mění o ±100 ms (změřeno). Párovací chyba
   záznamu = v × (stáří + latence): při 0,4 m/s **±40 mm jitter** mezi sousedními
   záznamy + systematický posun ~57 mm dozadu. (Latence `/odometry/gps`
   receive−stamp: 37–137 ms, bimodální.)
3. `_apply_continuous_anchor` (navi_transform:1508) z takto jitterujícího záznamu
   (Ta = now − 2 s) spočítá cíl → cíl jitteruje ±40–100 mm.
4. `_set_anchor_pose_slewed` (navi_transform:1561): korekce ≤ `max(pos_slew_mps*0.1,
   **0.10 m**)` se aplikuje **PŘÍMO, bez slew**, službou `/ekf_wheel_nav_odometry/set_pose`.
   Naše skoky (40–95 mm) jsou VŠECHNY pod 10cm prahem → póza se teleportuje 10× za
   sekundu. (set_pose navíc krátce naruší rychlostní stav — nvx propady ~0,05 m/s
   na korekčních ticích.)
5. TEB vidí 10Hz pilu ±40–90 mm podélně → cmd_vel na ni reaguje → **fyzické cukání**.

Sedí všechna pozorování: amplituda = v × ±100 ms; podélný směr (chyba ∝ rychlosti);
střídání znaménka (aliasing 10Hz tik vs. 5Hz GPS); klid = 0 (v=0 → párovací chyba 0);
offset 57 mm = v × střední latence.

### Doporučené opravy (v pořadí důležitosti)

1. **Párovat podle časových značek (root fix)**: záznam bufferu tvořit z GPS pozice
   a wheel-EKF pózy **interpolované v čase `header.stamp` GPS zprávy** (držet krátkou
   historii wheel póz; append ideálně při příchodu GPS zprávy = 5 Hz, ne v 10Hz tiku).
   Odstraní jitter ±40 mm i systematických ~57 mm zpoždění.
2. **Zrušit 10cm floor pro přímou aplikaci** v `_set_anchor_pose_slewed`: deadband
   ~10–15 mm, nad ním VŽDY slew (stávající cap 5 cm/tik stačí; klidně snížit).
   Zbytkový šum cíle pak pózu nikdy neteleportuje. Chrání i proti 80mm skokům
   při posunu kotvy v klidu.
3. Volitelně: korekci aplikovat jen když přišel nový GPS vzorek (5 Hz místo 10 Hz)
   — poloviční počet set_pose zásahů do EKF.

Očekávaný efekt: póza hladká na úrovni RTK šumu (±14 mm), cukání zmizí.
Validace po opravě: tatáž metrika (rezidua kroků vs. wheel odom) musí za GPS
ownershipu spadnout z RMS ~11 mm na ~1 mm; jízdou ověřit subjektivně.

## B) „Na trackeru mapa chvílemi dost plavala" — z bagu jen částečně, nalezeny 3 podezřelé mechanismy

Bag NEobsahuje /tf, /nav_tf/set_map_pose_tracker ani gloc status → map→odom slew
nelze rekonstruovat přímo. Co ale bag ukázal:

1. **Za tracker ownershipu je odom-EKF dokonale hladký** (rezidua 0,3–1,5 mm) →
   veškeré viditelné „plavání" je pohyb map→odom transformace, tedy tracker korekce
   samotné (žádný vliv EKF).
2. **Ownership je INVERZNÍ vůči očekávání** (změřeno z rychlostí):
   - tracker vlastní pózu V KLIDU: 11,43–11,60 (v=0!), 16,31–16,63 (v=0!)
   - a ZTRÁCÍ ji ZA JÍZDY: 11,60–11,70, 11,92–12,39, 13,04–13,97 (v≈0,33) = dr
   Standstill gate (gloc_server:1337, Rule 2) má korekce v klidu POTLAČIT — ale
   owner=tracker v klidu znamená, že korekce v klidu TEKLY. Buď `_is_standstill`
   nezabírá (detekce?), nebo ownership drží stará korekce. K tomu za jízdy tracker
   nepublikuje (poor_match při rozmazaném scanu? persistence gate 'settling'?
   deadband 'aligned' → nepublikuje → owner spadne na dr i při zdravém trackingu
   = artefakt zobrazení ownershipu, gloc_server:1391).
   → Vzor „korekce chodí v dávkách kolem zastavení/zpomalení" přesně odpovídá
   „mapa CHVÍLEMI plavala": drift naakumulovaný za jízdy se slewuje po dávkách.
3. **Chybí kompenzace stáří scanu** (gloc_server:1169–1186): scan smí být až 1,5 s
   starý, ale zarovnává se z AKTUÁLNÍ pózy (`pose0 = _current_pose()`), bez motion
   kompenzace → za jízdy korekce obsahuje falešný člen −v × stáří_scanu (při
   0,3 m/s a 10Hz lidaru ~3 cm, s jitterem stáří osciluje). Stejná třída chyby
   jako (A). EMA to tlumí, ale nezruší střední tah dozadu.

### Než se cokoli změní — INSTRUMENTACE (příští jízda)

Rozšířit recorder (vitulus_rosbag/config/recorders.yaml, profil loc_eval) o:
- `/tf` (map→odom!), `/nav_tf/set_map_pose_tracker`, `/cmd_vel`,
- gloc status/info topic (stavy: aligned/settling/poor_match/standstill/engage_delay
  + rms, corr_m — ať jde rozlišit, PROČ tracker za jízdy nepublikuje),
- ideálně i `/scan` (ověření smear/deskew hypotézy).
Pak teprve ladit (deadband/EMA/slew rychlosti, standstill detekci, scan-age korekci).

## IMPLEMENTOVÁNO 2026-08-07 večer (tentýž den)

Opravy A nasazeny do `navi_transform` (záloha: `navi_transform.bak_anchorpair_20260807_224701`):

1. **Párování podle stampů**: nové historie `_wheel_hist`/`_nav_yaw_hist` (5 s @ 30 Hz,
   plněné v callbaccích), buffer entry vzniká JEN pro nový GPS vzorek (5 Hz),
   s časem = `header.stamp` GPS zprávy a wheel/nav stavem interpolovaným v tom
   čase (`_hist_at` + `_interp_scalar`/`_interp_angle`, yaw po nejkratším oblouku).
2. **Deadband + vždy-slew**: `_set_anchor_pose_slewed` i legacy větev
   `set_wheel_base_nav_pose`: pod `~anchor_deadband_m` (default 0,015 m) se
   set_pose NEvolá vůbec (šetří i rychlostní stav EKF), nad ním vždy slew
   cap `pos_slew_mps*0.1` (10cm direct-apply floor odstraněn).

**Offline A/B validace nad validačním bagem** (pairing_sim, GPS segment 3–9 min):
jitter cíle kotvy (reziduum kroku vs. wheel odom) RMS **28,3 → 5,7 mm**,
p99 56 → 21 mm, tiky >30 mm **45,1 % → 0,3 %**. Zbytek je pod deadbandem.

Nasazení: rosnode kill → respawn OK (22:50), log čistý, loc_status teče
(owner=dock). **Polní validace při příští jízdě nutná** (metrika: rezidua kroků
nav EKF za GPS ownershipu RMS ~11 mm → ~1 mm + subjektivně cukání).

Recorder rozšířen (`vitulus_claude/tools/loc_eval_record`): +`/tf`, `/cmd_vel`,
`/mobile_base_controller/cmd_vel_out`, `/gloc/track_status`, `/gloc/status`,
`/nav_tf/set_map_pose*` (4 kanály), `/scan` → příští bag umožní analýzu (B).

## C) Bonus nálezy

- dr segment 9,41–9,60: reziduum max 8,5 m = známá outage relokace (v pořádku, rewind).
- dock na konci (26,15+): jediný skok 442 mm = dock relock (v pořádku).
- `/odometry/gps` latence je bimodální (37/137 ms) — navsat zpracování dávkuje;
  po opravě A1 (párování podle stampů) je to jedno.
- GPS segment 15,99–16,31 měl jen 56 % FIXED (float 44 %) — večerní konstelace;
  heading anchor freeze fungoval (frozen 16,17–16,31 dle loc_status).
