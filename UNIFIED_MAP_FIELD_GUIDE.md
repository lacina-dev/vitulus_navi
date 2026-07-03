# Jednotná mapa dock↔zahrada — terénní návod

_2026-07-03. Cíl: JEDNA rtabmap mapa pokrývající garáž (dock) i zahradu, ve které
robot startuje z docku bez GPS, vyjede ven, seká, a vrací se — bez švu mezi
indoor/outdoor. Kód je připraven (boot-in-dock, rtabmap_on_start, dock seed,
map→odom yaw korekce); tenhle návod je fyzická část, kterou musí odjezdit člověk._

## Kdy
Za dne (kamera = vizuální loop closures do mapy), klidné světlo, suchá tráva.
GPS venku k dispozici (není nutná pro mapu samotnou — DB je GPS-agnostická —
ale navsat si během jízdy ustaví map→utm georeferenci).

## Postup

1. **Start: robot v docku, systém nabootovaný.** Zkontroluj, že aktivní mapa je
   outdoor (env OUTDOOR — kvůli `self.indoor=False`, ať nová mapa vznikne jako
   outdoor s outdoor rtabmap launchem). Po dnešku bootuje s `aaaa***env*OUTDOOR`,
   takže OK.
2. **Nová mapa:** ve web UI „New map" (outdoor) — tj. publish na
   `/navi_manager/new_map`. Spustí outdoor rtabmap v MAPPING módu s čistou DB
   (`--delete_db_on_start`; stará běžící DB se předtím zálohuje jako BACKUP_LAST).
3. **Mapovací jízda (ručně, pomalu):**
   - Začni V DOCKU (nebo těsně před ním), **postůj 5–10 s** (kotevní uzly).
   - Pomalu vyjeď dveřmi ven — v průjezdu jeď zvlášť pomalu (úzký profil, ať
     se scan-match chytá).
   - Objeď zahradu reprezentativní smyčkou; u výrazných míst (roh domu, strom,
     plot) **postůj ~5 s**. Vracej se občas přes už zmapovaná místa (loop closures).
   - Vrať se dveřmi zpátky ke doku a **zadokuj** (couvnutí do docku) — loop
     closure přesně v místě startu je nejcennější.
4. **Ulož „docked" waypoint:** robot je zadokovaný (blesk u baterie svítí,
   `/dock_smach/dock_status==0`) a lokalizace sedí → publish **`docked`** na
   `/navi_manager/save_waypoint`. (Ukládá aktuální map→base_link — TOHLE je kotva
   pro noční/bez-GPS starty.)
   - Klidně ulož i navigační bod `dock` (příjezdový bod před dokem), pokud ho
     mise používají.
5. **Ulož mapu:** publish jméno (např. `UNIFIED_07_2026`) na
   `/navi_manager/save_map` — **v klidu, zadokovaný** (kopíruje se živá DB;
   při stání do ní rtabmap nezapisuje). Vznikne `UNIFIED_07_2026***env*OUTDOOR`.
6. **Přepni boot mapu:** v `vitulus_navi/config/navi_manager.yaml` nastav
   `load_map_on_start: "UNIFIED_07_2026***env*OUTDOOR"` a restartni navi_man
   (nebo celý robot). Od té chvíle: boot v docku → mapa se načte → rtabmap
   localization → dock seed → robot ví, kde je, bez GPS, ve dne v noci.

## Ověření (PASS kritéria)

- Telemetrie (`vitulus_claude` navigation.yaml): `rtab_prox` při jízdě po známých
  místech roste (desítky), `rtab_ref` roste při mapování, po návratu k doku
  aspoň 1 loop closure.
- Po kroku 6: reboot / restart navi_man v docku → do ~2 min
  `tf map→base_link` sedí na dock pozici, `/rtabmap/localization_pose` stabilní
  (mm šum), heading = yaw docked waypointu.
- Volitelně databaseViewer: jediná souvislá komponenta grafu (žádné ostrovy).
- Kidnap test (nepovinné, plán Fáze 2): odnes/odjeď robota jinam za dne →
  vizuální reloc; v noci se počítá jen start z docku (fundamentální limit lidaru).

## Pozor

- **Zóny sekání / plánovací data jsou per-mapa.** Nová mapa = nový rámec →
  zóny v planneru bude potřeba překreslit (save_map ukládá i planner_data —
  ale k NOVÉ mapě se váže nový/prázdný stav). Počítej s tím než přepneš produkci.
- Během mapování nesekat a nespouštět mise (CPU + čistota mapy).
- Mapu NEEDITUJ za tmy — noční jízdy jen v localization módu (nekazí DB).
  Rozšiřování mapy = „Edit map" za dne (localize-then-map už je zapojený).
- Kdyby po kroku 2 rtabmap spadl / mapa se nekreslila: `docker logs
  RTABMAP_MAPPING` a zkontroluj `rosparam get /rtabmap/rtabmap/Mem/IncrementalMemory`
  (musí být true při mapování).
