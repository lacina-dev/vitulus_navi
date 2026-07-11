# Mrtvé launch soubory (přesunuto 2026-07-10, WP4 LOCALIZATION_V2_PLAN.md)

Tyto soubory byly ověřeny jako **osiřelé** (žádná živá reference) před
přesunem: `grep -rn` přes celý `catkin_ws/src` (vč. `other/rplidar_ros`,
`cbgl`, `dock_utils`, atd.), přes `/etc/systemd/system/vitulus.service`
a `vitulus/launch/vitulus.startup`. Jediné nalezené reference byly
zakomentované `<!-- <include .../> -->` řádky nebo odkazy uvnitř jiných
už mrtvých souborů.

Produkční launch řetěz (viz `../../doc/LAUNCH_CHAIN.md`) je:
`vitulus.service` → `vitulus.startup` → `nodes/main` →
`vitulus_start.launch` → `navi_man.launch` → `navi_man`/`navi_transform`
si PYTHONEM roslaunchují `vitulus_navi_outdoor.launch` /
`vitulus_navi_indoor.launch` a `vitulus_ekf_outdoor.launch`. Tyto tři
soubory v tomto řetězci nefigurují.

## Přesunuté soubory a proč

- **ekf_base.launch** — načítal `config/ekf/ekf_base.yaml`. Jediná
  reference: zakomentovaný include ve
  `vitulus_navi_outdoor.launch:8` (`<!-- ... -->`).
- **ekf_outdoor.launch** — načítal (mj.) `ekf_base_outdoor.yaml` (ten je
  ŽIVÝ, ale přes `vitulus_ekf_outdoor.launch`, ne přes tento soubor) a
  zakomentovaně `ekf_global.yaml`/`navsat.yaml`. Jediná reference:
  zakomentovaný include ve `vitulus_navi_outdoor.launch:10`.
- **ekf_outdoor_all.launch** — načítal `config/ekf/ekf_outdoor.yaml`.
  Jediná reference: zakomentovaný include ve
  `vitulus_navi_outdoor.launch:6`. (Pozn.: tento soubor NEBYL v původním
  seznamu kandidátů z LOCALIZATION_V2_PLAN.md, ale ověření ho odhalilo
  jako stejně mrtvý — přidán do úklidu.)

## Co NENÍ mrtvé (ponecháno na místě, pro pořádek)

- **ekf_indoor.launch** — na první pohled vypadá podobně (malý EKF
  launch v config/ekf stylu), ALE je aktivně includován
  (needcommentovaně) v `vitulus_navi_indoor.launch:6`, který se
  PYTHONEM roslaunchuje z `navi_man` (řádek ~797) pokaždé, když
  `self.indoor == True` (nastaví se podle sufixu jména mapy
  `***env*INDOOR`). ŽIJE za indoor módu — NEPŘESOUVAT bez ověření na
  živém indoor testu.

## Jak obnovit

`git mv` (nebo `mv`) soubor zpět do `vitulus_navi/launch/` a odkomentovat
příslušný `<include>` v `vitulus_navi_outdoor.launch`. Nic jiného se
měnit nemusí — cesty (`$(find vitulus_navi)/...`) jsou beze změny.
