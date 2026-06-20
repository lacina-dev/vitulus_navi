# Mower SMACH — Dokumentace chování robota při sekání

## Přehled

Stavový automat (`mower_unit_smach`) řídí celý životní cyklus autonomního sekání — od přijetí programu, přes vyjíždění z doku, sekání zón, až po návrat a řešení chyb. Běží jako ROS uzel a využívá framework SMACH.

---

## Hlavní stavový diagram

```
WAIT_FOR_PROGRAM → PRE_START_CHECK → MISSION_CONCURRENCE → RETURN_TO_DOCK
                                           │                      │
                                           ↓                      ↓
                                     CRITICAL_ERROR ←────────────┘
                                           │
                                           ↓
                                     TERMINAL_ERROR
```

---

## 1. Čekání na program (`WAIT_FOR_PROGRAM`)

- Robot je v klidu, čeká na příkaz z webového rozhraní.
- Sleduje **dva topicy**:
  - `/web_plan/program_active` — nový program
  - `/web_plan/program_active_unfinished` — obnovení po pádu/restartu
- Při obnovení se parsuje `last_result` a nastaví se resumovací pozice (zóna, cesta, window index).

---

## 2. Kontrola před startem (`PRE_START_CHECK`)

Před spuštěním mise robot ověří:

| Kontrola | Podmínka odmítnutí |
|---|---|
| **Déšť** | Prší nyní, pršelo v posledních 60 min, nebo předpověď deště v nowcastu |
| **Baterie** | Kapacita < 40 % |

Pokud podmínky nesplněny → program odmítnut, zpět na `WAIT_FOR_PROGRAM`.

---

## 3. Mise (`MISSION_CONCURRENCE`)

Hlavní sekací logika běží **souběžně se 4 monitory**. Pokud jakýkoli monitor spustí alarm, mise se okamžitě přeruší.

### 3.1 Souběžné monitory

| Monitor | Topic | Podmínka přerušení | Výsledek |
|---|---|---|---|
| **Počasí** | `/weather_alert/rain_alert` | Déšť předpovězen v 2+ ze 3 nowcast kroků | → návrat do doku |
| **Baterie** | `/pm/power_status` | Kapacita ≤ 25 % | → návrat do doku |
| **Teplota motoru** | `/mower/status` | Stav = `TEMP` | → návrat do doku |
| **STOP signál** | `/mower_smach/stop` | `True` | → TERMINAL_ERROR (karanténa) |

**Priorita monitorů:** STOP > Teplota > Baterie > Počasí

### 3.2 Průběh mise (MISSION_CHILD)

```
CHECK_IF_DOCKED → UNDOCK → LOAD_MAP → WAIT_FOR_GPS/RTABMAP → WAIT_FOR_PLANNER
    → POWER_ON_MOWER → ZONE_IT (iterátor zón) → POWER_OFF_MOWER
```

#### Vyjíždění z doku

1. **CHECK_IF_DOCKED** — zjistí stav z `/dock_smach/dock_status`
   - `0` (docked) → spustí undocking
   - `3` (undocked) → přeskočí na nahrání mapy
   - Timeout (5× bez odpovědi) → předpokládá undocked
2. **GET/SEND_UNDOCK_PROGRAM** — pošle undock příkaz do dock_smach
3. **WAIT_FOR_UNDOCKED** — čeká až dock_smach nahlásí status 3 nebo 4 (max 180 s)

#### Nahrání mapy

- **LOAD_MAP** — podle formátu `map_name` (obsahuje `***env*INDOOR`/`OUTDOOR`) rozhodne typ:
  - **Indoor** → `/navi_manager/load_map_rtabmap` → čeká na RTABMAP
  - **Outdoor** → `/navi_manager/load_map` → čeká na GPS

#### Čekání na GPS (outdoor)

- Používá topic `/nav_tf/odom_status` (typ `Navi_transform`)
- **Podmínky pro pokračování** (všechny musí platit):
  - `status == "SAT"` (navsat transformace běží)
  - `fused_fix_time < 3 s` (pozice z GPS 1 je čerstvá)
  - `fused_nav_time < 3 s` (heading z GPS 2 je čerstvý)
- Timeout: 600 s → abort mise

#### Čekání na plánovač

- **WAIT_FOR_PLANNER** — čeká až se načte costmapa pro danou mapu (max 120 s)

---

## 4. Sekání zón (`ZONE_IT`)

Iterátor projde všechny zóny definované v programu. Pro každou zónu:

```
GET_ZONE_DATA → GET_PATH_TO_START → EXE_PATH_TO_START → VERIFY_AT_ZONE_START
    → SET_CUT_HEIGHT → SET_RPM → SET_MOTOR_ON → WAIT_FOR_SET_RPM → PATH_IT
```

### 4.1 Příprava zóny

1. **GET_ZONE_DATA** — načte parametry zóny (výška, RPM, cesty)
   - Při obnovení po pádu přeskočí již dosekané zóny
2. **Navigace na start zóny** — plánuje cestu přes MBF (`get_path` + `exe_path`)
   - Ověří příjezd přes TF (tolerance 0.5 m)
   - Při selhání: recovery (clear costmap) + retry (max 3×)
3. **SET_CUT_HEIGHT** — nastaví výšku sečení, čeká na READY (max 120 s)
4. **SET_RPM + MOTOR_ON** — nastaví otáčky a zapne motor, čeká na dosažení RPM (±100)

### 4.2 Sekání cest (`PATH_IT`)

Iterátor projde všechny cesty v zóně. Pro každou cestu:

```
GET_PATH_DATA → GET_PATH_TO_BEGIN → CHECK_DISTANCE → EXE_PATH_TO_BEGIN
    → CHECK_PLANNER_PATH → WINDOW_PLANNER_PATH ⟷ EXE_PLANNER_PATH
```

#### Zpracování cesty

1. **GET_PATH_DATA** — segmentizuje surovou cestu na body po ~3 cm
   - **Outline cesty** (uzavřený polygon): `LineString.length == 0` → obrys
   - **Infill cesty** (přímky): segmentizace rovných čar
2. **Navigace na začátek cesty** — plánuje a jede na start
   - `CHECK_DISTANCE` — pokud robot < 0.2 m od startu → přeskočí jízdu
3. **CHECK_PLANNER_PATH** — ověří průjezdnost přes costmapu
   - Při selhání: `TRIM_AND_RETRY` (ořeže konec cesty, max 5× po 15 bodech)
   - Po vyčerpání trimů: bypass a jede přímo

#### Windowování a exekuce

- **WINDOW_PLANNER_PATH** — z celé cesty vyřízne chunk 200 bodů od aktuální TF pozice
- **EXE_PLANNER_PATH** — jede chunk přes MBF s real-time feedbackem
  - Když zbývá < 0.4 m do konce chunku → plynulý přechod na další window
  - Detekuje `BLOCKED` a `ERR` stav motoru za jízdy
- Crash recovery: při každém novém chunku uloží pozici do `program.last_result`

---

## 5. Recovery (obnova po chybě)

### 5.1 BLOCKED motor (4-fázová kaskáda)

Když se motor zasekne (tráva, drát, překážka):

| Fáze | Akce | Pokusy |
|---|---|---|
| **1** | Restart motoru na místě | 2× |
| **2** | Zvednutí nože na max výšku + restart | 2× |
| **3** | Únik ~2 m dopředu bez nože, pak restart | 1× |
| **4** | Selhání → CRITICAL_ERROR | — |

Po fázi 2 se nastaví `restore_height_pending` → výška se vrátí po dalším úspěšném chunku.

### 5.2 Navigační selhání → objížďka (drží se trasy, objede překážku)

Když MBF/TEB nedokáže projet cestu (překážka na sekací lince):

| Fáze | Stav | Akce |
|---|---|---|
| **1 – dynamická** | `NAV_RECOVERY` | Vypnutí nože + čekání ~6 s (mapy překážek samy odeznívají) + clear costmap + couvnutí 0,4 m + retry **téhož** chunku (2×). Pomáhá u procházejícího člověka/zvířete. |
| **2 – objížďka** | `DETOUR` | Když překážka přetrvá: najde první dosažitelný bod na lince **za** překážkou, naplánuje globální objezd (`get_path`, globální costmapa nově vidí živé překážky), objede ho s **vypnutým nožem** a od bodu napojení pokračuje v sekání. Neposekaný zůstane jen úsek u překážky, ne celá linka. |
| **3 – přeskok linky** | `DETOUR_ESCALATE` | Jen když objížďka není možná: přeskočí **tuto linku** (ne celou zónu) a jede dál; inkrementuje `consecutive_nav_failures`. |
| **4 – vzdání** | → `NAVIGATION_ABORTED` | Až po **5** po sobě neposekatelných linkách (bez jediného úspěšného posekání mezi nimi — počítadlo se nuluje při každém dokončeném chunku). Vede na CRITICAL_ERROR, který **nejdřív zkusí návrat do doku** (ne rovnou karanténu). |

---

## 6. Ukončení zóny

Po dosekání všech cest v zóně:

1. **SET_MOTOR_OFF_AND_HOME** — bezpečné vypnutí nože (`safe_blade_shutdown`)
2. Zvýšení nože na `max_height`
3. Čekání na stav `READY`
4. → další zóna nebo konec

---

## 7. Vypnutí sekačky (`POWER_OFF_MOWER`)

Po dosekání všech zón:

1. `safe_blade_shutdown()` — vypne motor, zvedne nůž na max
2. `set_power(False)` — vypne sekačku
3. Uloží `last_result = 'succeeded'` a dobu trvání v minutách
4. → `RETURN_TO_DOCK`

---

## 8. Návrat do doku (`RETURN_TO_DOCK`)

Spouští se po dokončení mise NEBO při alarmu z monitorů.

```
SAFE_SHUTDOWN → CHECK_DOCK_POINT → GET_DOCK_PROGRAM → SEND_DOCK_PROGRAM → WAIT_FOR_DOCKED
```

1. **SAFE_SHUTDOWN** — bezpečné vypnutí nože + power off
2. **CHECK_DOCK_POINT** — ověří, zda existuje dokovací bod
3. **GET/SEND_DOCK_PROGRAM** — získá a odešle dokovací program do dock_smach
4. **WAIT_FOR_DOCKED** — čeká na potvrzení zadokování

Celý návrat běží **souběžně se STOP monitorem** — pokud přijde STOP signál během dokování → TERMINAL_ERROR.

---

## 9. Chybové stavy

### CRITICAL_ERROR (rozhodovací uzel)

Rozhoduje, zda zkusit návrat do doku nebo jít rovnou do karantény:

| Důvod chyby | Akce |
|---|---|
| `DOCKING_TIMEOUT`, `DOCKING_FAILED` | → TERMINAL_ERROR (dokování už selhalo, neopakovat donekonečna) |
| Ostatní (`NAVIGATION_ABORTED`, `BLOCKED_FAILED` apod.) | → pokus o návrat do doku. Pokud i dokování selže, vrátí se sem s `DOCKING_*` → TERMINAL_ERROR. |

### TERMINAL_ERROR (karanténa)

- Robot **zůstane na místě**, nůž je bezpečně vypnutý
- **Pípá každých 30 s** (melodie)
- Čeká na signál `/mower_smach/reset` (`Bool: True`)
- Po resetu → zpět na `WAIT_FOR_PROGRAM`

---

## 10. Safe Blade Shutdown

Bezpečné vypnutí nože se volá na mnoha místech. Sekvence:

1. Vypne motor (`set_motor_on = False`)
2. Přečte `max_height` z FW (fallback 80)
3. Nastaví výšku na `max_height`
4. Čeká na stav `READY` (nůž dojel nahoru)

Je idempotentní — opakované volání je bezpečné.

---

## Souhrn topiců

| Topic | Směr | Účel |
|---|---|---|
| `/web_plan/program_active` | ← vstup | Nový sekací program |
| `/web_plan/program_active_unfinished` | ← vstup | Obnovení po pádu |
| `/mower_smach/stop` | ← vstup | Nouzové zastavení |
| `/mower_smach/reset` | ← vstup | Reset z karantény |
| `/weather_alert/rain_alert` | ← vstup | Data o dešti |
| `/pm/power_status` | ← vstup | Stav baterie |
| `/mower/status` | ← vstup | Stav sekačky (RPM, teplota, BLOCKED) |
| `/nav_tf/odom_status` | ← vstup | GPS lokalizace (obě GPS) |
| `/dock_smach/dock_status` | ← vstup | Stav doku |
| `/dock_smach/start_docking` | → výstup | Příkaz k dokování/undockování |
| `/navi_manager/load_map` | → výstup | Nahrání outdoor mapy |
| `/nextion/log_info` | → výstup | Log zprávy na displej |
