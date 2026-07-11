# Póza/odometrie v2 — GPS arbitr, hladké přechody, trvalý datum

_2026-07-06, návrh po diskusi. Zadání uživatele + nálezy z kódu navi_transform.
Vrstva odom (EKF wheels+IMU+VO+licp twisty) se NEMĚNÍ — je validovaná a spojitá._

## Zadání (od uživatele)

1. GPS je hlavní arbitr pózy, dokud má kvalitní signál.
2. Bez GPS přebírá VO/lidar/wheel dead-reckoning; dok dává iniciální pózu.
3. Přechody no-GPS↔GPS musí být hladké, včetně startu v no-GPS.
4. Lokalizace v mapě (pokud je), nebo z doku (známá póza).
5. Nová zahrada + start v doku bez GPS: po náběhu kvalitní GPS zpětně
   georeferencovat začátek (dok) do GPS souřadnic.
6. **Rewind při ztrátě GPS:** kvalitativní metriky hlásí degradaci pozdě —
   poslední ~1–2 s „dobré" GPS už bývají zkreslené. Kotva pro dead-reckoning
   se proto bere z času T−rewind (≈2 s před pádem kvality) + odometrický
   přírůstek od té doby. (Odom je spojitá → jde o JEDEN přepočet map→odom
   z korespondence v bufferu, žádná reintegrace.)

## Klíčové nálezy v kódu (navi_transform)

- map→odom v SAT módu: `callback_map_coords` počítá map→odom = utm→odom −
  utm_mapa, kde utm_mapa = `/navi_manager/map_coords` (georeference POČÁTKU
  načtené mapy v UTM) a utm→odom je TF od navsatu (skutečné UTM, absolutní).
  → **Datum mapy už existuje** (map_coords per navi mapa); chybí pro
  mapping-v3 sites a pro scénář „mapa ještě není".
- Zero map_coords → map→odom := utm→odom (mapa ≡ UTM, milionové souřadnice)
  — nutno ošetřit capture-em (bod R5), ne pádem do UTM.
- GPS-acquisition hystereze existuje (`_gps_stable`, 5 s) — sdílí ji georef
  i loop3sec set_pose cesta.
- navsat lifecycle vlastní navi_transform (restart indoor→outdoor kvůli
  čerstvému utm→odom bez multipath biasu od doku).
- rtabmap korekce: /nav_tf/set_map_pose + ownership pivot rtabmap_confident
  — z pózové cesty postupně ven (gloc/EDT tracker), viz
  MAPPING_ARCHITECTURE_PLAN.

## Architektura

### Stavový arbitr (explicitní, s hysterezí)
- `GPS_FIXED`: carrSoln FIXED + hAcc/vAcc prahy + shoda dual-antenna headingu
  (kritéria = stejná jako insertion_gate mapping-v3) + **probation okno**:
  po splnění prahů ještě ~3 s křížová kontrola Δgps vs Δodom (< práh, např.
  0,1 m/okno) — až pak důvěra. Vstup do stavu: korekce SLEW, ne skok.
- `NO_GPS`: map→odom zmrazen, jede odom vrstva (VO/licp bridging už funguje).
  Volitelně lidar reloc korekce (gloc/EDT), též slew.
- `DOCKED`: póza := dok kotva (přesná, ve framu mapy/site).

### Hladkost
- **Slew korektor**: v GPS_FIXED se map→odom dotahuje k cíli rychlostí max
  ~5 cm/s a ~0,5°/s (parametry). Nad kidnap práh (~1 m) skok + událost
  (mapping si dá cooldown / Clear).
- **Rewind buffer (bod 6)**: kruhový buffer ~10 s párů (t, odom→base,
  gps póza, kvalita). GOOD→BAD: najdi poslední vzorek s kvalitou
  „prokazatelně dobrá" (přísnější práh, min. T−rewind_s) → map→odom :=
  gps(T_a) ⊖ odom(T_a). Korekce bývá malá (cm–dm) → aplikovat hned/rychlým
  slew.
- **Trvalá inovační brána**: klouzavé okno Δgps vs Δodom i během GPS_FIXED —
  tiše degradující GPS se pozná dřív než z kovariance; zkracuje zkažený
  ocásek, který rewind musí pokrýt.

### Datum / georeference (R5)
- Per site: `~/.vitulus/mapping_v3/<site>/datum.yaml` = UTM (E, N, alt, yaw)
  počátku site framu (+ lat/lon informativně, + verze).
- **Capture (nová site, start bez GPS):** frame drží dok/odometrie. Při
  prvním GPS_FIXED (po probation) spočti utm_site = utm→odom ∘ (map→odom)⁻¹
  a ulož. Map→odom se NEMĚNÍ → žádný skok; dok tím zpětně dostává GPS
  souřadnice (ulož do dock.yaml). Datum se váže k lokálnímu framu, ne
  naopak — už postavená data se nepřepočítávají.
- **Apply (existující site):** georeference se použije jako map_coords
  ekvivalent → GPS měření od začátku v site framu; boot v doku bez GPS =
  dok kotva; náběh GPS souhlasí na úrovni RTK šumu (+ slew). Z-datum
  z alt → výšky konzistentní mezi sezeními (řeší i open item mapping-v3).
- Kidnap/nejistota: gloc (později EDT tracker) nad site rastrem.

## Stav (2026-07-06 večer)

- 1a georef capture: HOTOVO (insertion_gate → datum.yaml + session soubory).
- 1b apply datum: HOTOVO (navi_transform + symlink z mapping_manageru;
  zero-georef degenerace ošetřena).
- GPS-first arbitr: HOTOVO dřív, než plánováno (obrácení pivotu z 5. 7.
  v navi_transform i dock_localization_seed + probation v pohybu ≥1 m).
- Heading: zdravé PÁSMO rozptylu 0.0010–0.0030 (uživatelova polní data:
  „příliš dobrý" rozptyl < 0.001 = degenerované ŠPATNÉ řešení; strop 0.0008
  předtím zdravé vzorky odmítal). GNSS forensics recorder přidán.
- Fáze 2 rewind: HOTOVO (buffer @10 Hz při korekcích; splice
  rtk(T−2s)+wheel-EKF delta při potvrzeném výpadku; wheel EKF bez set_pose
  = čisté delty).
- Fáze 3 slew: pozice/heading slew už existoval (drive-6); doplněn slew+
  kidnap pro georef map→odom (datum cesta běží 1 Hz).
- Fáze 5 EDT tracker: IMPLEMENTOVÁN (gloc_server: kontinuální scan-to-EDT
  Gauss-Newton, adaptivní tlumení + best-iterate proti oscilaci na hřebeni
  EDT, degenerační brána z 1. iterace eig_min/n≥0.05, rms/velikostní brány;
  publikuje /nav_tf/set_map_pose jen při gps_good==0, zatím
  ~track_defer_to_rtabmap=true — vypnutím rtabmap odchází z pózové cesty;
  ~map_topic připraven na mapping-v3 rastr). Syntetická validace OK;
  polní test pod střechou OTEVŘEN.
- OTEVŘENO: polní validace cyklů otevřeno↔střecha (vč. trackeru pod
  střechou — očekávání: drift ~0.9 m → cm-dm); fáze 4 (formální arbitr
  + stavová publikace); přepnutí trackeru na mapping-v3 rastr a vypnutí
  rtabmapu (track_defer_to_rtabmap=false).

## Fáze

- **1a (additivní, bez rizika): georef capture v mapping pipeline** —
  insertion_gate při první RTK-grade stabilitě session spočítá a uloží
  `datum.yaml` site (utm→odom ⊖ map→odom + alt). Nemění chování pózy,
  jen měří a ukládá. Každá session připíše `datum_session_*.yaml` pro
  pozdější kontrolu driftu/refit.
- **1b: apply datumu** — navi_transform: pokud je k dispozici site datum
  a není načtená navi mapa s map_coords, použít site datum jako georef
  (a scénář zero-map_coords nikdy nespadne do „mapa=UTM").
- **2: rewind buffer + probation + trvalá inovační brána** (navi_transform,
  chirurgicky; polní test: projet přechod garáž↔zahrada tam i zpět, měřit
  skok pózy v okamžicích přechodů — cíl < 5 cm).
- **3: slew korektor** místo set_pose snapů (polní test: mapping běží,
  čítač korekcí má zůstat ~0, mapa se nesmí posunout).
- **4: arbitr formálně** (stavový automat, status publikace pro UI/mapping).
- **5: rtabmap ven z pózové cesty** (EDT tracker dle MAPPING_ARCHITECTURE_PLAN).

## Rizika

- Drift dok→první FIX se zapeče do datumu site (validované VO/licp ~0,15 m
  medián → pro konzistenci zahrady OK; offline refit yaw z RTK trajektorií
  možný později).
- navi_transform = validovaný, křehký kód → zálohy, fáze, polní verifikace
  každé fáze zvlášť (workflow: recommend-first, surgical edits).
- Rewind kotva vyžaduje, aby buffer měl GPS pózy už PŘEPOČTENÉ do site
  framu (jinak přechod přes datum offsety).
