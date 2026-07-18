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
