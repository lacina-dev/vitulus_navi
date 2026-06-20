# Mower SMACH — analýza chování u překážek a TODO pro přepracování

> Stav: **návrh, zatím nic neměněno.** Tento dokument popisuje, proč robot
> selhává na překážkách a končí v `TERMINAL_ERROR`, a navrhuje změny seřazené
> podle priority. Slouží jako zadání pro pozdější zpracování (doladíme společně).
>
> Datum: 2026-05-30

---

## STAV IMPLEMENTACE (2026-05-30)

**Implementováno** (záloha původního stavu v `~/BACKUP/vitulus_navi_<timestamp>/`):

- ✅ **P0-1** — globální costmapa nově vidí živé překážky přes
  **`ObstacleLayer`** (zdroje `/rplidar/scan_5m` + `/obstacles_cloud`),
  `update/publish_frequency` 0,5 → 1,0 Hz. (`config/global_costmap_params.yaml`)
  - ⚠️ **POZOR – první pokus byl chybný:** zrcadlení rolling lokálních map jako
    `StaticLayer` do *statické* globální costmapy ji zmenšilo na 5×5 m a smazalo
    zdi → globální plánovač jel přes zeď. `StaticLayer` v nerolling costmapě
    resizuje master grid. Opraveno na `ObstacleLayer` (jen překrývá, neresizuje,
    sám raytrace-čistí). **Změna se projeví až po restartu `move_base_flex`.**
- ✅ **P0-2** — nový stav `DetourAroundObstacle` (objezd překážky + návrat na
  linku) v `states.py`, zapojený v `mission.py` (`NAV_RECOVERY → DETOUR →
  WINDOW_PLANNER_PATH`).
- ✅ **P0-3** — `NAVIGATION_ABORTED` odebráno z `NO_DOCK_REASONS`; robot se při
  navigačním selhání nejdřív pokusí o návrat do doku. (`top_level.py`)
- ✅ **P0-4** — `clear_costmap` recovery cílí na reálné vrstvy
  (`lidar/cloud_costmap_layer`) místo neexistující `laser_layer`.
  (`config/recovery_behavior.yaml`)
- ✅ **P1-1** — `NAV_RECOVERY` už nezahazuje celou linku; vrací `detour_needed`.
- ✅ **P1-2** — eskalace přepracována: skip jen jedné linky a kritická chyba až
  po 5 po sobě neposekatelných linkách (`DETOUR_ESCALATE` v `mission.py`).
- ✅ **P2-1/P2-2** — sjednoceno čekání (~6 s) a zaktualizována `mower_smach.md`.

**Záměrně NEZMĚNĚNO (k doladění):**

- ⏸️ **P1-3 (proaktivní detekce)** — `CHECK_PLANNER_PATH`/`TrimAndRetry` zatím
  fungují postaru (při bypassu se vjede a TEB → abort → DETOUR to dořeší).
  Reaktivně to funguje, jen méně elegantně.

**Iterace 2 (2026-05-31) — debug logování + řešení „konce chunku v překážce":**

- ✅ **Detailní logování** napříč pipeline (grep-tagy `[EXE]`, `[WINDOW]`,
  `[DETOUR]`, `[NAV_REC]`): indexy oken, cíl chunku, dist_to_goal, ujetá
  vzdálenost, MBF outcome kódy slovně (`mbf_outcome_str` v `helpers.py`),
  časování, u DETOUR každý pokus o napojení (get_path/exe_path outcome+cost+t).
- ✅ **Detekce uváznutí (stall) v `ExecutePathWithFeedback`** — když robot
  `STALL_TIME` (5 s) nepostupuje a není u cíle, akce se zruší dřív, než vyprší
  TEB `oscillation_timeout` (10 s). Rozhodnutí podle ujeté vzdálenosti v chunku:
  - ujel ≥ `PROGRESS_EPS` (0,3 m) → `replan_needed` = přeplánuj okno z aktuální
    pozice (řeší „konec chunku skončil v překážce" — odřízne nedosažitelný
    konec),
  - ujel < 0,3 m (překážka hned před robotem) → `aborted` → NAV_RECOVERY → DETOUR.
  Tím odpadá to dlouhé oscilování („tanečky") u nedosažitelného cíle.
- ⏸️ K doladění po testu: konstanty `STALL_TIME`, `PROGRESS_EPS`,
  `REJOIN_DISTANCES_M`, příp. proaktivní ořez konce okna mimo překážku přes
  `check_path_cost` (zatím řešeno reaktivně přes stall→replan).
- ⏸️ **P1-4 (ladění TEB)** — parametry TEB **nesahány**, ať se nerozbije dnešní
  ladění. K ověření: `min_obstacle_dist`/inflation napříč costmapami.

**Nutno ověřit na robotu:** rebuild (`catkin build vitulus_navi`), že globální
costmapa se 3 StaticLayer korektně skládá (lokální to dělá, takže OK), a reálné
chování objížďky u překážky.

---

## 1. Co `mower_smach` (balíček `mower_unit_smach`) dělá dnes

Vstupní node: `nodes/mower_unit_smach`. Top-level stavový automat:

```
WAIT_FOR_PROGRAM → PRE_START_CHECK → MISSION_CONCURRENCE → RETURN_TO_DOCK
                                          │                      │
                                          ↓                      ↓
                                    CRITICAL_ERROR ←────────────┘
                                          ↓
                                    TERMINAL_ERROR (karanténa, pípá, čeká na reset)
```

Cíl podle zadání je splněn jen částečně: vyjede z doku, načte mapu/program,
zajistí lokalizaci (GPS/RTABMAP), poseká podle programu, vrátí se do doku, umí
resume (`last_result = on_path-<zone>-<path_idx>-<window>`), monitoruje počasí,
baterii a teplotu motoru souběžně přes `MISSION_CONCURRENCE`.

**Sekání samotné** (`mission.py` → `_build_process_path_sm`):

```
GET_PATH_DATA → GET_PATH_TO_BEGIN → CHECK_DISTANCE → EXE_PATH_TO_BEGIN
   → CHECK_PLANNER_PATH → WINDOW_PLANNER_PATH ⟷ EXE_PLANNER_PATH
```

- Sekací linka se segmentizuje po ~3 cm a okénkuje na chunky po 200 bodech
  (`states.py:WindowPlannerPath`).
- Každý chunk se posílá na `/move_base_flex/exe_path`
  (`states.py:ExecutePathWithFeedback`) — tj. **přesné sledování trasy lokálním
  controllerem (TEB)**, bod po bodu, kvůli pokrytí (coverage).
- Při zaseknutí nože → `BLOCKED_RECOVERY` (4 fáze, `recovery.py`).
- Při selhání navigace (TEB nedokáže projet) → `NAV_RECOVERY`.

---

## 2. Kořenová příčina problému s překážkami (nejdůležitější zjištění)

Když se na sekací lince objeví překážka (nově položený předmět, zvíře, hromada
trávy, „duch" v mapě), **žádná vrstva navigačního stacku ji neumí objet**:

### 2.1 Globální plánovač je slepý vůči živým překážkám
`config/global_costmap_params.yaml` má jen:
```
plugins: static_layer (/navi_manager/map) + inflation_layer
```
Globální costmapa tedy obsahuje **pouze statickou mapu** (zdi z uložené mapy),
nikoli živé překážky z lidaru/pointcloudu. Živé překážky jsou jen v *lokální*
costmapě (`local_costmap_params.yaml`: `lidar_costmap_layer`,
`cloud_costmap_layer`, publikované z balíčku `vitulus_local_costmap`).

➡️ **Důsledek:** `get_path` (GlobalPlanner) i `CHECK_PLANNER_PATH`
(`GLOBAL_COSTMAP`) nevidí reálnou překážku. Kdyby se i chtěl naplánovat objezd,
naplánuje trasu **rovnou přes překážku**, protože o ní neví.

### 2.2 Sekání jede „čistým sledováním trasy", ne plánováním
`EXE_PLANNER_PATH` posílá fixní chunk na `exe_path`. To je controller (TEB),
který má držet zadanou linku (`weight_viapoint: 480`, `global_plan_viapoint_sep:
0.1`, `global_plan_overwrite_orientation: true`). TEB se umí lokálně vyhnout jen
v rámci optimalizace (`min_obstacle_dist: 0.5`, `weight_obstacle: 200`), ale
**neumí globálně přeplánovat objezd a vrátit se na linku**. Pokud překážka sedí
na lince, TEB nenajde řešení → `exe_path` skončí `aborted` (nebo osciluje a
spadne na `controller_patience: 6 s` / `oscillation_timeout: 10 s`).

To je správné chování pro pokrytí (chceme, aby se držel linky), ale znamená to,
že **objezd musí zařídit stavový automat na vyšší úrovni**, ne TEB.

### 2.3 `NAV_RECOVERY` neumí objet — umí jen čekat, couvnout, zkusit znovu, nebo zahodit celou linku
`recovery.py:build_nav_recovery_sm`:
1. nůž off + čekání (konstanta `WAIT_SECONDS = 5`, doc tvrdí 10 s) — pomůže jen
   dynamické překážce, která sama odejde,
2. `clear_costmap` + retry **téhož** chunku (max 2×),
3. couvnutí 0,4 m + retry téhož chunku,
4. `skip_path` → `continue_path` = **zahození CELÉ zbývající linky** (ne jen
   úseku u překážky) + `consecutive_nav_failures += 1`,
5. po **3** takových přeskočeních → `critical_error` → `NAVIGATION_ABORTED`.

### 2.4 `NAVIGATION_ABORTED` jde rovnou do karantény (TERMINAL)
`top_level.py:CriticalErrorState.NO_DOCK_REASONS` obsahuje `NAVIGATION_ABORTED`.
Takže 3 zablokované linky → robot se **ani nepokusí vrátit do doku**, zůstane
stát v poli, pípá a čeká na reset. To je přesně „skončí s TERMINAL ERROR".

### 2.5 `clear_costmap` recovery maže neexistující vrstvu (pravděpodobně no-op)
`recovery_behavior.yaml`: `clear_costmap` i `clear_costmap_laser` mají
`layer_names: [laser_layer]`. Ale **žádná costmapa nemá plugin jménem
`laser_layer`** — lokální má `base_static_layer / cloud_costmap_layer /
lidar_costmap_layer / inflation_layer`, globální má `static_layer /
inflation_layer`. Blok `laser_layer:` v `local_costmap_params.yaml` je tedy
osiřelá konfigurace, kterou žádný plugin nepřebírá.

➡️ **Důsledek:** clearing v `NAV_RECOVERY` (`goal.behavior = 'clear_costmap'`)
i v `RECOVERY_ZONE`/`RECOVERY_BEGIN` velmi pravděpodobně **nemaže nic**. Navíc
živé překážky jsou `StaticLayer` (lidar/cloud_map) — ty se beztak neraytracují
samy jako `ObstacleLayer`, čistí je jen externí node `vitulus_local_costmap`.
*(Nutno ověřit na robotu, ale podle konfigurace je clearing neúčinný.)*

### Shrnutí řetězce selhání
překážka na lince → TEB neprojede → `aborted` → wait/clear(no-op)/backup/retry
téže linky (překážka pořád tam) → zahození celé linky → po 3× `NAVIGATION_ABORTED`
→ `NO_DOCK_REASON` → **TERMINAL_ERROR uprostřed pole**.

Chybí celý koncept „**objeď překážku a vrať se na trasu**".

---

## 3. Návrh změn — TODO seřazené podle priority

### P0 — bez tohoto se objíždění nikdy nerozjede

- [ ] **P0-1: Globální costmapa musí vidět živé překážky.**
  Přidat do `global_costmap_params.yaml` živé obstacle vrstvy (lidar/cloud),
  např. `lidar_costmap_layer` + `cloud_costmap_layer` jako u lokální costmapy
  (nebo společný `ObstacleLayer`/voxel). Bez toho `get_path` ani objezd
  fyzicky nemůže fungovat. *(Souvisí s ladením plánovače, které jsme dnes
  řešili.)*
  - Pozor na velikost globální costmapy a frekvenci (`update_frequency: 0.5`) —
    pro reaktivní objíždění bude potřeba zvýšit.

- [ ] **P0-2: Nový stav „DETOUR" (objezd a návrat na trasu).**
  Když `EXE_PLANNER_PATH` skončí `aborted` kvůli překážce, místo zahození celé
  linky:
  1. Najít na aktuální lince **první volný bod ZA překážkou** (zjistit zablokovaný
     úsek přes `check_path_cost` po bodech / segmentech).
  2. `get_path` z aktuální TF pozice na ten downstream bod → globální objezd
     (využívá P0-1, jinak nefunguje).
  3. `exe_path` objezdu s **vypnutým nožem** (přejíždíme nesekanou plochu).
  4. Po dojezdu zapnout nůž a `WINDOW_PLANNER_PATH` od downstream bodu → pokračovat
     v sekání linky.
  Neposekaný zůstane jen malý úsek těsně u překážky (to je nevyhnutelné), ne celá
  linka. Robot se drží trasy „co nejvíce", jak zadání chce.
  - Místo napojení: `mission.py` přechod `EXE_PLANNER_PATH: 'aborted' →` (dnes
    `NAV_RECOVERY`).

- [ ] **P0-3: Nepadat do TERMINAL při navigačním selhání, pokud robot může jet.**
  `NAVIGATION_ABORTED` vyřadit z `NO_DOCK_REASONS` (`top_level.py:26`), nebo
  rozlišit „nemůžu se hnout z místa" vs. „jednu linku nešlo dojet". Jedna
  neposekatelná linka nesmí znamenat karanténu uprostřed pole — robot se má
  alespoň pokusit vrátit do doku.

- [ ] **P0-4: Opravit / ozdravit costmap clearing recovery.**
  V `recovery_behavior.yaml` nastavit `layer_names` na skutečně existující
  vrstvy (živé obstacle vrstvy), nebo zavést pořádný `ObstacleLayer` s
  raytrace clearingem pro dynamické překážky. Ověřit na robotu, že clear
  reálně něco dělá. Bez toho je „wait + clear + retry" jen čekání.

### P1 — chování objíždění a eskalace

- [ ] **P1-1: `skip_path` nesmí zahazovat celou linku.**
  Přeskakovat jen zablokovaný **úsek** (span bodů u překážky), ne zbytek linky.
  Po objezdu/přeskoku pokračovat ve windowingu od dalšího volného bodu.
  (`recovery.py:NavSkipCheck` + `WindowPlannerPath`.)

- [ ] **P1-2: Přepracovat eskalaci `consecutive_nav_failures`.**
  Dnes 3 přeskoky → critical. Po zavedení DETOUR by se mělo eskalovat jen tehdy,
  když selže i objezd (překážka neobjetelná / robot zaklíněný), ne při běžném
  objetí. Zvážit metriku „kolik % plochy nešlo posekat" místo prostého počtu.

- [ ] **P1-3: Proaktivní detekce blokace před vjetím.**
  `CHECK_PLANNER_PATH` (už voláno před každým chunkem) po P0-1 uvidí překážky →
  využít ho k vyvolání DETOUR **dřív**, než robot do překážky najede a TEB se
  zasekne. Dnes při neúspěchu jen ořezává konec (`TrimAndRetry`) a pak stejně
  jede (`bypass_check`) — to je špatně, „bypass" znamená vjet do známé překážky.

- [ ] **P1-4: Koordinace s laděním TEB (dnešní téma).**
  Pro coverage chceme TEB „lepkavý" na lince (vysoký `weight_viapoint`). Objíždění
  ať řeší stavový automat (P0-2), ne uvolňování TEB. Ověřit, že `min_obstacle_dist:
  0.5` + `inflation_dist: 0.6` nejsou tak velké, že TEB odmítá projíždět úzké
  koridory mezi sousedními swathy u překážek (zbytečné aborty). Zvážit menší
  `min_obstacle_dist` pro sekačku + konzistentní inflation napříč costmapami
  (globální `inflation_radius: 0.85` vs. common `0.5` vs. local — sjednotit).

### P2 — drobnosti a konzistence

- [ ] **P2-1:** `NavBladeOffWait.WAIT_SECONDS = 5`, ale `doc/mower_smach.md` a
  komentář mluví o 10 s. Sjednotit kód ↔ doc.
- [ ] **P2-2:** Po zavedení DETOUR aktualizovat `doc/mower_smach.md` (sekce 4.2 a
  5.2).
- [ ] **P2-3:** Zvážit ukládání „neposekaných úseků" do programu (reporting do
  webUI: kde byla překážka), ať obsluha ví, kam se vrátit.
- [ ] **P2-4:** `WaitForMowerStatus`/`WaitForTopic` používají opakovaně
  `wait_for_message` — při více monitorech to může soupeřit o zprávy; zvážit
  trvalé subscribery (jak už dělá `ExecutePathWithFeedback._mower_sub`).

---

## 4. Otevřené otázky k dořešení (společně)

1. **Objízdný koridor:** jak daleko za překážku napojit linku zpět? Pevná
   vzdálenost (např. „první volný bod + N cm"), nebo dynamicky podle velikosti
   zablokovaného úseku?
2. **Co s dynamickou vs. statickou překážkou?** Dynamickou (člověk, zvíře) má
   smysl chvíli počkat (dnes 5 s); statickou objet hned. Rozlišovat podle toho,
   jestli překážka po `clear` zmizí?
3. **Globální costmapa s živými překážkami** — nezpůsobí to problémy v jiných
   režimech (indoor/RTABMAP, dokování)? Nutno otestovat napříč režimy.
4. **Rozsah dnešního ladění TEB** — které parametry jsme dnes měnili a proč, ať
   to s P1-4 nerozbijeme. (Doplnit referenci na konkrétní změny.)
5. **`vitulus_local_costmap`** — jak přesně se plní a čistí `lidar_map`/`cloud_map`?
   To určuje, jestli a jak rychle „zmizí" odjetá dynamická překážka.

---

## 5. Dotčené soubory (orientační mapa pro implementaci)

| Oblast | Soubor |
|---|---|
| Tok sekání / přechody stavů | `src/mower_unit_smach/mission.py` (`_build_process_path_sm`) |
| Okénkování, exekuce, detekce BLOCKED | `src/mower_unit_smach/states.py` (`WindowPlannerPath`, `ExecutePathWithFeedback`, `TrimAndRetry`) |
| Recovery (BLOCKED / NAV) | `src/mower_unit_smach/recovery.py` |
| Routing chyb (TERMINAL vs. dock) | `src/mower_unit_smach/top_level.py` (`CriticalErrorState`) |
| Globální costmapa (P0-1) | `config/global_costmap_params.yaml` |
| Costmap clearing (P0-4) | `config/recovery_behavior.yaml`, `config/local_costmap_params.yaml` |
| TEB ladění (P1-4) | `config/base_teb_local_planner_params.yaml`, `config/controllers.yaml` |
| Globální plánovač | `config/global_planner_params.yaml` |
