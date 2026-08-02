# Rešerše: navigace & mapování pro náš HW v ROS1 — co změnit a co ne
2026-08-02, deep-research (108 agentů, 12 ověřených zjištění, adversariální verifikace 3-0, 25 zdrojů)

HW kontext: NUC i5, 4WD + enkodéry, RPLidar 2D, D435, 2× ZED-F9P (RTK + moving-baseline
heading), BNO085; zahrada ~600 m², GPS-denied dvůr, noční provoz.

## Hlavní závěr

**Náš stack odpovídá současné komunitní best practice a v percepci je PŘED ní.**
Referenční projekt OpenMower (6,6k★, ROS1 Noetic) je RTK-first stejně jako my — a
ověřeno přímo v jeho kódu: **nemá žádnou senzorickou detekci překážek** (costmap
ObstacleLayer bez observation_sources, překážky = ručně kreslené no-go polygony,
kolize řeší až nárazník). Náš direct raster z D435+lidaru a EDT tracker jsou nad
rámec toho, co komunita má. Většina verdiktů je KEEP/TUNE, ne REPLACE.

## Verdikt po oblastech

### 1. Lokalizace (dual-EKF + navsat + arbitr) — **KEEP**
- Dual-EKF robot_localization + navsat_transform je stále referenční vzor: používá
  ho OpenMowerNext (ROS2 port) i oficiální Nav2 GPS dokumentace. Mainline OpenMower
  má vlastní lehký EKF (xbot_positioning) — pořád Kalman, ne factor graph.
- Factor-graph fúze (FGO) existuje prakticky jen na ROS2: MowgliNext (GTSAM iSAM2,
  jeden graf místo přepínání ownership — přímý protějšek našeho arbitru), ale je to
  first-beta (~29★), ne field-proven. Na ROS1 je `fuse` (Locus, ROS1 větev živá
  06/2026), ALE bez GNSS senzorového modelu — museli bychom ho napsat + build ze
  zdrojů. Kvantitativní přínos FGO (~55 % redukce chyby, Wen et al.) je z urban-canyon
  code-pseudorange dat — na cm-RTK zahradě marginální.
- → **Neměnit.** Náš arbitr + heading anchor v2 dělá totéž co MowgliNext graf, jen
  explicitně; přepis = riziko bez měřitelného zisku.

### 2. Mapování (direct raster) — **KEEP (+ malé TUNE)**
- Vlastní 2D log-odds raster ze segmentovaných mračen je rozumný pro náš výpočetní
  výkon. elevation_mapping_cupy (moderní GPU alternativa) je CUDA-only — na Intel
  NUC bez NVIDIA **nepoužitelné**, vyřazeno z úvah.
- Tráva vs překážka: literatura (Sathyamoorthy 2023 aj.) řeší lidar INTENZITOU
  (pevné >0.75R, tráva <0.5R), ale vyžaduje 3D lidar s kalibrovanou reflektancí
  (VLP-16); RPLidar dává jen quality byte v jedné rovině — bez nového senzoru
  nereprodukovatelné. Levná mitigace = ladit obstacles_detection (D435) a výškové
  prahy, což už děláme (klasifikátor v5, ground-contact gate).

### 3. GPS-denied relokalizace (EDT tracker, vypnutý relock) — **TUNE — nejcennější nález**
- slam_toolbox localization mode NENÍ drop-in: sám Macenski varuje před laděním a
  nutností kvalitní odometrie; issues dokumentují skoky a nespolehlivou inicializaci.
  Náš EDT přístup není horší než dokumentovaný stav ekosystému.
- **RTAB-Map (plně ROS1) má hotové mechanismy přesně proti našemu 4,68m teleportu:**
  1. `Rtabmap/LoopGPS=true` — hypotézy jen v GPS rádiusu (RGBD/LocalRadius),
  2. `RGBD/OptimizeMaxError` — odmítne closure, která zhorší konzistenci grafu,
  3. `Optimizer/Robust` (Vertigo switchable constraints; nekombinovat s (2)).
- → Dvě cesty: (a) reaktivovat rtabmap jako relokalizační vrstvu s těmito gaty,
  nebo (b) **doimplementovat tytéž principy do našeho EDT relocku**: GPS/DR-radius
  gate na hypotézy + absolutní quality gate + konzistenční sanity check (korroborace
  druhým zdrojem před aplikací). Doporučuji (b) — držíme jednodušší pipeline a
  přesně to už máme rozpracované (gates e0d8fc5).

### 4. Plánování (MBF + TEB + vlastní coverage) — **KEEP**
- Žádný ověřený claim proti TEB pro pomalou coverage jízdu; OpenMower používá vlastní
  slic3r-based planner (ROS1), fields2cover je věc ROS2 éry (opennav_coverage na něm
  staví). Fields2Cover je čistá C++ knihovna — teoreticky jde přilinkovat i k ROS1,
  ale ověřený integrační vzor neexistuje. → Vlastní coverage planner držet; fields2cover
  je kandidát AŽ při migraci na ROS2, ne důvod k ní.
- Otevřená otázka (nezodpovězeno rešerší): TEB vs pure-pursuit/FTC pro přesné sledování
  boustrophedon drah — případný levný A/B experiment na vlastní datech.

### 5. ROS1 Noetic EOL — **plánovat, nespěchat**
- Komunita je rozdělená: mainline OpenMower AKTIVNÍ na Noetic (v1.2.0, 05/2026);
  Mowgli linie plně přešla na ROS2 Kilted/Nav2 (first-beta). Migrace je směr, ne
  nouzový stav — motivací jsou přínosy (Nav2, opennav_docking, coverage server),
  ne akutní riziko.
- Pragmaticky pro jednoho robota za NAT: **kontejnerizovaný Noetic je obhajitelný
  na 2-3 roky**. Doporučený postup: (1) teď nic, (2) při příští větší přestavbě
  zvážit ROS2 Jazzy+ s Nav2 — náš site bundle (UTM GeoJSON) a direct raster jsou
  na ROS verzi nezávislé, přenesly by se.
- Otevřené (rešerše nedala tvrdá data): reálné CVE riziko rosmaster/TCPROS za NAT.

### 6. Levné HW doplňky — **bez ověřené odpovědi**
- Reflektorové sloupky pro noční relokalizaci na dvoře a AprilTag na doku: žádný
  ověřený komunitní vzor nalezen (RPLidar intenzita = jen quality byte). Náš vlastní
  plán reflektorů (memory 07/2026) zůstává neověřená, ale rozumná hypotéza — vyžadovala
  by vlastní experiment s RPLidar quality na retroreflektivní pásce.

## Prioritizovaný shortlist

**Stojí za to:**
1. **Relock v2 s RTAB-Map principy** (GPS/DR-radius gate + absolutní quality +
   korroborace) → znovu zapnout auto-relock bezpečně. Nejvyšší hodnota/úsilí.
2. Kontejnerizace Noetic stacku (izolace od EOL Ubuntu 20.04) — až bude klid.
3. A/B experiment TEB vs jednoduchý pure-pursuit na coverage dráhách (jen měření).
4. Experiment RPLidar quality byte na retroreflektivní pásce (dvůr, noc) — rozhodne
   o reflektorových sloupcích.

**Výslovně NEstojí za to (teď):**
- Přepis fúze na factor graph (fuse bez GNSS modelu; přínos na RTK zahradě marginální).
- slam_toolbox jako náhrada EDT trackeru (dokumentovaně křehčí než to, co máme).
- elevation_mapping_cupy (CUDA-only, nemáme NVIDIA).
- Převzetí OpenMower stacku (v percepci je za námi).
- 3D lidar kvůli intenzitní klasifikaci trávy (řešíme D435 segmentací).
- Urgentní migrace na ROS2 (plánovat ano, spěchat ne).

## Klíčové zdroje
OpenMower: github.com/ClemensElflein/OpenMower + openmower.de docs;
OpenMowerNext: github.com/jkaflik/OpenMowerNext; MowgliNext: mowgli.garden,
github.com/cedbossneo/mowglinext; fuse: github.com/locusrobotics/fuse;
slam_toolbox: github.com/SteveMacenski/slam_toolbox; RTAB-Map robust graph:
github.com/introlab/rtabmap/wiki/Robust-Graph-Optimization; FGO vs EKF: Wen et al.,
NAVIGATION 68(2); tráva/intenzita: arxiv.org/pdf/2309.07014.
