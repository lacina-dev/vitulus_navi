# Mrtvé EKF/navsat configy (přesunuto 2026-07-10, WP4 LOCALIZATION_V2_PLAN.md)

Ověřeno stejným postupem jako `../../launch/_dead_20260710/README.md`
(grep přes celý `catkin_ws/src`, systemd, startup skript). Všechny níže
byly načítané POUZE ze souborů, které jsou samy mrtvé (viz
`../../launch/_dead_20260710/`), nebo ze zakomentovaných řádků.

Živý EKF config je **`../ekf_base_outdoor.yaml`**, načítaný
`vitulus_ekf_outdoor.launch` (živý, viz `../../doc/LAUNCH_CHAIN.md`),
který si PYTHONEM roslaunchuje `navi_transform` (řádek ~277). Živý
indoor config je `../ekf_base_indoor.yaml` (viz poznámka o
`ekf_indoor.launch` v launch README — ten NENÍ mrtvý). Živý navsat
config je `../navsat_transform.yaml`, načítaný `vitulus_navsat.launch`
(živý, `navi_transform` řádek ~284).

## Přesunuté soubory a proč

- **ekf_base.yaml** — načítán jen mrtvým `ekf_base.launch`.
- **ekf_outdoor.yaml** — načítán jen mrtvým `ekf_outdoor_all.launch`.
- **ekf_global.yaml** — načítán jen zakomentovaným řádkem v mrtvém
  `ekf_outdoor.launch` (dvojitě zakomentovaný HTML-entity blok,
  `&lt;!--...--&gt;`).
- **navsat.yaml** — načítán jen zakomentovaným řádkem v mrtvém
  `ekf_outdoor.launch`. Nezaměňovat s ŽIVÝM `navsat_transform.yaml`
  (jiný soubor, jiný obsah, zůstává na místě).

## Jak obnovit

`git mv` (nebo `mv`) soubor zpět do `vitulus_navi/config/ekf/` a zároveň
obnovit odpovídající launch soubor z
`../../launch/_dead_20260710/` (config bez launch, co ho načte, k ničemu
není).
