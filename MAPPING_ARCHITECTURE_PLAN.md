# Mapping v3 — architektura „mapování se známými pózami"

_2026-07-06. Výstup dvoukolové ověřované rešerše (2× multi-agent, vše proti
primárním zdrojům). Nahrazuje rtabmap-SLAM jako tvůrce mapy; lokalizační stack
(EKF fúze + RTK brány + dock kotva + gloc) zůstává beze změny jako autorita pózy._

## Zadání (od uživatele)
- Platforma pro VÍCE uživatelů a RŮZNÝCH zahrad (i hektarové), ne jednoúčelovka.
- Dnes ROS1 Noetic (tvrdá podmínka), plánovaná migrace na ROS2 → řešení s budoucností.
- Přesnost: ~5 cm u struktur; mapa věrná realitě.
- Terén NENÍ rovina → 3D základ, 2D okupance = překážky 0,10–0,60 m NAD LOKÁLNÍ ZEMÍ.
- Mapa editovatelná a sezónně aktualizovatelná.

## Klíčový princip
Lokalizace je vyřešená a validovaná (RTK venku, odometrie+dock uvnitř, gloc
jako globální reloc). Mapování se tím mění ze SLAM na **vkládání se známými
pózami** — deterministické, bez loop closures, bez putujících rámců; přesnost
mapy = přesnost pózy.

## Doporučená architektura (vítěz přebodované matice)
1. **PRIMÁRNÍ produkt: grid_map 2.5D per zahrada** — vrstvy: `ground_elevation`
   (DEM), `obstacle_band_evidence`, `confidence`, `last_seen`. Oficiální ROS1 i
   ROS2 větve, binární balíky.
2. **SEKUNDÁRNÍ 3D archiv důkazů: octomap_server** (`.ot` per zahrada; už
   nainstalován, benchmarknut na NUC). Role: free-space raytracing pro mazání
   duchů + percentilová výplň DEM v neprojetých místech. (Absolutní-z projekce
   octomapu se NEPOUŽÍVÁ — na svahu je špatně.)
3. **Vlastní terénní jádro (~1–2k řádek, ROS-agnostické — potřeba u KAŽDÉHO
   backendu, není se čemu vyhnout):**
   - gated insertion (RTK FIXED + kovariance + shoda dual-antenna headingu s EKF
     + inovační brána; log rozhodnutí per scan),
   - DEM fúze: trajektorie robota (base_link z − poloměr kola; cm přesnost,
     imunní vůči vegetaci, pokryje vše posekatelné) + 5. percentil obsazených z
     per sloupec octree pro neprojetá místa + inpainting/slope-consistency,
   - band classifier/projektor: obstacle(x,y) = evidence bodů se
     z − DEM(x,y) ∈ [0,10; 0,60] m → 2D rastr; regenerace na vyžádání.
4. **Per-site bundle (mapa záznamu):** `sites/<garden_id>/` = manifest.yaml
   (semver, RTK datum, CRS) + boundary.geojson (geofence) + elevation.tif +
   obstacle GeoTIFF + nav pgm/yaml + garden.ot + edits-mask (lidské edity,
   komponované přes každou regeneraci) + waypoints/zones.geojson + dock.yaml.
   Editace v QGIS / obrázkovém editoru = nezávislé na ROS, ideální pro víc lidí.
5. **Výběr mapy podle GPS geofence** (~100 řádek: fix → point-in-polygon →
   load bundle; vlastní — ověřeno, že OSS ekvivalent neexistuje). Selektor
   vlastní sekvenci přepnutí (stop kontejnerů, navsat re-datum!).
6. **rtabmap:** dočasně jen reloc fallback; po nasazení EDT trackeru (rozšíření
   gloc: Gauss-Newton scan-to-EDT, kovariance z Hessiánu, degenerační brána na
   živé ploty) se vypíná.

## ROS2 migrační cesta (nejbezpečnější ze všech variant)
- octomap: oficiální ROS2 (octomap_server 2.3.x binárně, .bt/.ot identické).
- grid_map: oficiální ROS2 větve. Vlastní uzly = ROS-agnostické jádro + tenké shimy.
- nav2_map_server servíruje stejné pgm/yaml; nav2_route bere GeoJSON zóny nativně.
- **Fáze 5 = rozhodovací bod:** volitelná výměna 3D archivu za
  **vdb_mapping_ros2** (OpenVDB 10 z apt na 24.04, live map-editing služby,
  .vdb ↔ Houdini/Blender/pyopenvdb) nebo Bonxai, pokud dozraje. Band projektor
  konzumuje generické sloupcové dotazy → výměna backendu nemigruje žádná data.

## OpenVDB — proč ne teď a kdy ano
Nezamítnuto — **designovaný vyzyvatel pro fázi 5**. Dnes na ROS1: source build
OpenVDB 9 na focal, wrapper API-driftlý od jádra, bez releases, jeden správce.
Po ROS2: všechny jeho silné stránky zlevní (apt balík, aktivní first-party
ROS2 wrapper). Benchmark harness připraven (scratchpad/vdbbench.cpp).

## Zamítnuto (ověřené důvody)
wavemap (nejlepší CPU integrátor, ale 18 měsíců mrtvo, bez ROS2 → bez budoucnosti),
Bonxai (ROS2-only, pre-1.0, paměťová regrese na řídkých mračnech), UFOMap (dohasíná),
elevation_mapping_cupy (CUDA+ROS2, robot-centric; relevantní až s GPU robotem),
ANYbotics elevation_mapping (discontinued), voxblox (mrtvý), nvblox (GPU+ROS2),
ohm (bez ROS wrapperu), AMCL (0,2m třída). GPU dnes NENÍ odůvodněné — vše
změřeno/ověřeno jako CPU-pohodlné i na hektar.

## Rizika (poctivě)
1. Šev u vrat garáže (RTK↔odometrie) se do mapy otiskne — dock kotva je nosný
   předpoklad; garáž mapovat jedním čistým průjezdem a šev změřit.
2. Ladění země-vs-tráva je terénní práce (2D lidar na svahu tiše seká zem).
3. Negativní překážky (schody, srázy) = DEM-diskontinuita, vlastní práce navíc.
4. Divergence editovaného rastru vs 3D archivu → edits-mask disciplína +
   validační skript v deploy cestě.
5. Hektarová čísla octomapu = extrapolace z měřeného 0,12 ha (~130 MB/ha @5 cm;
   3 cm jen na lokální záplaty).
6. Insertion single-thread → throttle lidar 2–5 Hz, D435 downsample — hlídat
   ve watchdogu/robot_health.

## Fáze
- **0 (dny, nulové riziko):** octomap_server + gated insertion vedle současného
  stacku; bagy z běžného sekání.
- **1 (~týden):** trajectory-DEM → grid_map elevace; validace DEM proti RTK na svahu.
- **2 (1–2 týdny):** band classifier + projektor → první ground-relative rastr;
  A/B proti rtabmap gridu; edits-mask; přepnutí nav/costmap/gloc.
- **3 (~týden):** bundle schema + geofence selektor; QGIS round-trip; test na
  druhé zahradě.
- **4 (sezónně):** visibility-decay, re-drive → regenerace → compare → edits →
  version bump.
- **5 (ROS2):** shimy, nav2; rozhodovací bod vdb_mapping_ros2/Bonxai.
