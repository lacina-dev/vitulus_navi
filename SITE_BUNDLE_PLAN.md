# Site bundle — sjednocení waypointů/cest/zón (návrh 2026-07-18)

_Cíl: JEDEN zdroj pravdy per zahrada v `~/.vitulus/mapping_v3/<site>/`,
geometrie kanonicky v UTM (přenositelná mezi mapami/datumy, QGIS-friendly).
Legacy pickle svět zůstává jako runtime cache — žádná změna topic API._

## Klíčová zjištění inventury (2026-07-18, plný report u orchestrátora)
- Waypointy+cesty: per-mapa `saves/maps_data/<name>.pkl` (MapData; pózy v MAP
  frame + UTM datum mapy). Jména `docked`/`dock`/`undock` jsou nosná
  (dock_localization_seed, dock_smach, navi_transform).
- Zóny: `planner_data.pkl` per-mapa (workspace + ~11 numpy vrstev, až 79 MB) +
  **globální** `running/planner_programs.pkl` — programy nesou vlastní snapshot
  zón VČETNĚ vygenerovaných drah (mower jede přesně tyto dráhy, MAP frame).
- **PROBLÉM: všech 10 programů má map_name='2025_06'**, aktivní mapa je
  GARDEN_2026 — programové dráhy jsou v cizím map frame. UTM kanonizace to řeší
  převodem přes datum zdrojové mapy.
- Datum trojmo: pickle (utm_x/y/z + quat), site `datum.yaml` (refined, n=235),
  session/final varianty. Pro GARDEN_2026 vs novamapa: E/N shodné, alt Δ0.17 m,
  yaw Δ~3 mrad — informativní warn při migraci.

## Rozhodnutí (orchestrátor)
1. **Kanonický formát: UTM metry** (EPSG:326<zone>, zone v manifestu).
   map↔UTM: `utm = R(yaw_datum)·p_map + (utm_e, utm_n)` — implementace MUSÍ
   odvodit přesnou konvenci z navi_man `set_map_utm_transform` a mít
   roundtrip test (map→utm→map < 1 mm).
2. **Bundle soubory** v `sites/<site>/`:
   - `manifest.yaml` — site, version, utm_zone, `navi_map` link
     ("GARDEN_2026***env*OUTDOOR"), created/updated, poznámky.
   - `waypoints.geojson` — Points [utm_e, utm_n]; props: name, yaw_rad, z.
   - `paths.geojson` — LineStrings; props: name, yaws[] (délka = # vrcholů).
   - `zones.geojson` — Polygons; props: name, cut_height, rpm, type,
     coverage_angle, paths_distance, simplify, border_paths, area, length.
   - `programs.yaml` — name, zone_names[], speed, rpm, cut_height,
     override_zone, last_result… (vygenerované dráhy se NEUKLÁDAJÍ — jsou
     artefakt; regeneruje planner z polygonů existující mašinerií).
3. **Datum autorita:** pro navi georef se dnes NIC nemění (pickle datum řídí
   `set_map_utm_transform`). Bundle konverze používají datum ZDROJOVÉ mapy
   (export legacy) resp. site `datum.yaml` (site svět). Rozdíl > 0.05 m /
   2 mrad ⇒ hlasitý warn, nikdy tichá volba.
4. **Dual-write, bundle-wins:** navi_man/planner při každém zápisu uloží
   pickle (kompat) I bundle export. Při loadu: pickle, pak pokud je bundle
   novější (mtime manifestu/souboru), přepíše se obsah z bundlu. Rollback =
   smazat bundle soubory.
5. **Topic API beze změny** — všechna `/navi_manager/*`, `/web_plan/*` témata
   zůstávají; mění se jen persistence pod nimi. Dock programy (pkl) beze změny
   (odkazují jen jmény waypointů).
6. **Lib umístění:** `vitulus_mapping/src/vitulus_mapping/bundle.py` (+
   `geo.py` afinní převody). navi_man/node_planner importují přes devel
   PYTHONPATH; ověřit importovatelnost, jinak sys.path fallback po vzoru
   navi_man.
7. **Vazba site↔navi mapa:** `manifest.navi_map`. Migrace default:
   novamapa ↔ GARDEN_2026***env*OUTDOOR (datum E/N shodné, .last_served),
   ale CLI arg povinně explicitní.

## Pracovní balíčky
- **WP-A (Opus):** bundle.py + geo.py + roundtrip testy + migrační CLI
  `tools/migrate_to_bundle` (dry-run default, --apply): waypointy/cesty
  z libovolného maps_data pkl, zóny z planner_data/programů, programy
  z planner_programs.pkl → cílový site; převod přes datum zdrojové mapy;
  filtr na `***env*` soubory; report všech konverzí + datum delt.
  ORCHESTRÁTOR REVIEW API + dry-run výstupu před WP-B/C.
- **WP-B (Opus):** navi_man — export při každém save_running_map_data,
  import-if-newer v load_running_map_data; manifest link autodetekce
  (site_datum symlink → site) + ~param override.
- **WP-C (Opus):** node_planner — zóny export/import (zones.geojson),
  programy export/import (programs.yaml, dráhy regenerovat existující
  cestou); running pickly zůstávají cache.
- **WP-D (po UI auditu):** editor v vitulus_ui nad site mapou (waypointy,
  zóny, cesty, edits-mask) přes stávající topicy.

## Rizika
- Přesnost převodu (yaw rotace!) — roundtrip test povinný, mm tolerance.
- Programy: regenerace drah musí dát ekvivalentní pokrytí (ne bit-identické);
  před večerním testem porovnat délku/area vs snapshot.
- Latched cache po restartech (zone_list/program_list) — po importu vždy
  republish.
- `map_name='new'` v picklech — klíčovat VÝHRADNĚ jmény souborů.

## Site-native flip (2026-07-18)

Přepínač `site_native` v `vitulus_navi/config/navi_manager.yaml` (DEFAULT
`false`). Je to cesta k odstavení legacy rtabmap navi map: mapping-v3 site
bundle se stává runtime autoritou mapy. **Když je `false`, VŠECHNY větve jsou
no-op a legacy chování je byte-for-byte beze změny.** Když je `true`, mění tři
věci:
1. **navi_man** boot map source je vynucen na `octomap` (servírovaný site
   rastr; reuse stávající octomap cesty vč. serve_site retry / bootstrap
   fallbacku).
2. **navi_man** map→odom **georef datum** bere z `datum.yaml` servírovaného
   site (přes `_georef_datum_values`) místo pickle data mapy — publikuje se na
   `/navi_manager/map_coords` (živý zdroj map→odom v `navi_transform`).
   Jednorázový WARN s deltou vůči pickle datu (pokračuje bez ohledu na deltu;
   dnes mm/mrad) + jeden loginfo „georef datum: site '<name>'“.
3. **navi_man** přeskočí kopii rtabmap `.db` při `load_map` (rtabmap mimo
   runtime); **node_planner** přestaví workspace rastr (`initial_map` + np
   vrstvy) ze servírovaného `/mapping_manager/site_map` místo legacy rastru
   (stejná MapData+assemble cesta jako `callback_map`). Zóny/programy dál
   z bundlu (dnešní práce).

### Předpoklady PŘED flipem
- Proběhl **plný pokrývací mapovací průjezd** zahrady → mapping_manager
  servíruje kompletní site rastr (`/mapping_manager/site_map` nonempty,
  s FREE prostorem — planner potřebuje volno pro generování drah zón).
- Bundle migrovaný (`waypoints/paths/zones.geojson`, `programs.yaml`,
  `manifest.navi_map` == aktivní navi mapa, `datum.yaml` existuje).
- `rtabmap_on_start: false` (site-native počítá s rtabmapem mimo runtime).
- Večerní validace: octomap zdroj servíruje rozumný rastr, gloc/tracker sedí.

### Flip = jediná změna
- V `navi_manager.yaml`: `site_native: false` → `true`. Restart navi_man
  (a node_planner) — edity se projeví až po restartu.

### Co ověřit PO flipu (živě)
- **Costmap zdravý** nad site rastrem: `/navi_manager/map` = site rastr
  (`map_source` == `octomap`/`bootstrap`, ne `planner`), global costmap static
  layer má mapu, MBF akce naběhly, footprint je.
- **Regenerace zón dává PLNÉ pokrytí**: po loadu planner přestavěl rastr ze
  site (log „workspace raster rebuilt from /mapping_manager/site_map“); zóny po
  bundle importu/regeneraci nedriftují >5 % area/length (guard warny) a dráhy
  pokrývají celou zónu (porovnat vůči snapshotu před flipem).
- **Georef delta malá**: log „georef datum: site … (delta vs pickle dE/dN/dYaw)“
  — dnes dE/dN≈0, dYaw≈3 mrad, dAlt≈0.175 m; žádný skok pózy po startu.
- **Docking approach funguje**: dock seed/`docked` waypoint sedí, appro k doku
  bez dislokace (georef změna je mm/mrad, ale ověřit reálně u doku).

### Rollback
- `site_native: false` + restart. Žádná migrace dat, žádné mazání — pickle svět
  je netknutý (rtabmap `.db` se při dalším loadu zase zkopíruje, georef zpět na
  pickle datum, planner rastr zpět z legacy pickle). Když se přeskočená `.db`
  kopie stane problémem po rollbacku, stačí jeden `load_map` z UI.
