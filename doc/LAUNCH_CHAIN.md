# Produkční launch řetěz (ověřeno 2026-07-10, WP4 LOCALIZATION_V2_PLAN.md)

_Vzniklo jako WP4 úklidové practice: zdokumentovat SKUTEČNÝ boot řetěz
proti kódu (file:line), aby se nikdo neztratil v `vitulus_bringup`
XML stromu, který se nikde nespouští. Viz `LOCALIZATION_V2_PLAN.md`
sekce „Ověřený současný stav" a G11. Soubory autoritativní pro tenhle
dokument: `nodes/main`, `nodes/navi_man`, `nodes/navi_transform`,
`/etc/systemd/system/vitulus.service`._

## Řetěz krok za krokem

```
systemd vitulus.service
  → ExecStart: bash --login -c '... vitulus/vitulus/launch/vitulus.startup'
    → python3 vitulus/vitulus/nodes/main            (main:106)
      → roslaunch vitulus/vitulus/launch/vitulus_start.launch   (main:41)
        ├── vitulus_ups, device_state_publisher, nextion_lcd, nmcli,
        │   vitulus_description (vitulus4wd), vitulus_imu, rplidar,
        │   d435, vitulus_base/base_control, pcl_voxelgrid,
        │   vitulus_gnss/vitulus_ardusimple_heading.launch,
        │   vitulus_gnss/vitulus_ardusimple_gnss.launch,
        │   vitulus_mower, vitulus_ds4, vitulus_planner,
        │   system_monitor, vitulus_ui, weather_alert, vitulus_dock,
        │   vitulus_rosbag, vitulus_safety   (viz vitulus_start.launch)
        │
        └── vitulus_navi/launch/navi_man.launch
              (vitulus_start.launch, include řádek — pkg vitulus_navi)
            ├── vitulus_local_costmap/launch/local_costmap.launch
            ├── node navi_man      (pkg vitulus_navi, type navi_man)
            ├── node navi_transform (pkg vitulus_navi, type navi_transform)
            ├── node gloc_server    (pkg vitulus_navi, type gloc_server)
            ├── node node_mower_smach (mower_unit_smach)
            ├── node node_smach_exe_path, node_exe_goal
            └── include vitulus_navi/launch/topic_tools.launch
```

Odtud dál **`navi_man`** a **`navi_transform`** Pythonem (přes
`roslaunch.parent.ROSLaunchParent` / `roslaunch.rlutil`) SPOUŠTÍ další
launch soubory a docker kontejnery za běhu — to je ta část řetězu, která
není vidět v žádném XML stromu a kterou tenhle dokument fixuje.

## `navi_man` (nodes/navi_man) — nav stack + rtabmap docker

- **Outdoor mód** (`self.indoor == False`, default): roslaunchuje
  `vitulus_navi/launch/vitulus_navi_outdoor.launch`
  (`nodes/navi_man:806-810`, volání `start_navi_launch`).
- **Indoor mód** (`self.indoor == True`): roslaunchuje
  `vitulus_navi/launch/vitulus_navi_indoor.launch`
  (`nodes/navi_man:793-797`).
- `self.indoor` se nastavuje za běhu podle sufixu jména načítané mapy
  (`***env*INDOOR` vs `***env*OUTDOOR`, viz `callback_load_map`,
  `nodes/navi_man:1053-1056` a `callback_load_map_rtabmap:1093-1096`) —
  tedy oba launche jsou reálně dosažitelné podle toho, jaká mapa je
  aktivně načtená.
- `vitulus_navi_outdoor.launch` (živý) includuje:
  `pointcloud_to_laserscan.launch`, `vitulus_navi_mbf.launch` (move_base
  flex), `vo_rgbd_odometry.launch`, `lidar_icp_odometry.launch`. EKF
  includy uvnitř tohoto souboru (`ekf_outdoor_all.launch`,
  `ekf_base.launch`, `ekf_outdoor.launch`) jsou **zakomentované** — EKF
  pro outdoor mód jede jinudy, viz `navi_transform` níže.
- `vitulus_navi_indoor.launch` (živý) includuje totéž
  (`pointcloud_to_laserscan`, `vitulus_navi_mbf`, `vo_rgbd_odometry`,
  `lidar_icp_odometry`) PLUS aktivně (needcommentovaně)
  `vitulus_navi/launch/ekf_indoor.launch` → `ekf_localization_indoor`
  node s `config/ekf/ekf_base_indoor.yaml`, `publish_tf: true`. Tohle je
  jediné místo, kde běží samostatný `robot_localization` uzel publikující
  TF nezávisle na `navi_transform` — POZOR při další práci na
  arbitrovi (viz G4 v LOCALIZATION_V2_PLAN.md, indoor kill-switch).
- **RTABMAP docker** (`DockerConnection`, `docker.from_env()` — NE
  roslaunch v ROS procesu, ale `docker exec roslaunch ...` uvnitř
  kontejneru): `nodes/navi_man:200-209` startuje trvale
  `RTABMAP_ODOM_SYNC` (`/rtabmap_docker/data_odom_sync.launch`) a
  `RTABMAP_OBSTACLES` (`/rtabmap_docker/obstacles_detection.launch`);
  `RTABMAP_MAPPING` kontejner se spouští/přepíná on-demand s
  `/rtabmap_docker/rtabmap_outdoor.launch` nebo `rtabmap_indoor.launch`
  podle `self.indoor` (`nodes/navi_man:274-278`, `815-831`).

## `navi_transform` (nodes/navi_transform) — EKF + navsat on demand

- **EKF**: `roslaunch_ekf()` spouští
  `vitulus_navi/launch/vitulus_ekf_outdoor.launch`
  (`nodes/navi_transform:277`). Ten soubor načítá
  **`config/ekf/ekf_base_outdoor.yaml`** (ŽIVÝ config — viz
  LOCALIZATION_V2_PLAN.md „Vrstva 1") a startuje dva uzly:
  `ekf_wheel_odometry` (čistá wheel+IMU reference, nikdy se nekoriguje)
  a `ekf_wheel_nav_odometry` (fúzuje wheel+VO+licp+heading; do něj
  chodí `set_pose` korekce z GPS-arbitru). TF odom→base vysílá
  `navi_transform` sám (30 Hz), NE `robot_localization` — outdoor EKF
  uzly TF nepublikují.
- **Navsat**: `roslaunch_navsat()` spouští
  `vitulus_navi/launch/vitulus_navsat.launch`
  (`nodes/navi_transform:284`), on-demand přes `init_navsat()`
  (`nodes/navi_transform:288+`, když chybí čerstvý fused fix/nav).
  Ten soubor načítá **`config/ekf/navsat_transform.yaml`** (ŽIVÝ
  config, jiný soubor než osiřelý `navsat.yaml`).

## Živé configy — shrnutí

| Vrstva | Launch (živý) | Config (živý) |
|---|---|---|
| Outdoor EKF (vrstva 1) | `vitulus_ekf_outdoor.launch` | `config/ekf/ekf_base_outdoor.yaml` |
| Indoor EKF | `ekf_indoor.launch` (přes `vitulus_navi_indoor.launch`) | `config/ekf/ekf_base_indoor.yaml` |
| Navsat transform | `vitulus_navsat.launch` | `config/ekf/navsat_transform.yaml` |

## Mrtvé stromy — varovný seznam

### `vitulus_navi/launch/_dead_20260710/` a `vitulus_navi/config/ekf/_dead_20260710/`

Ověřeno a přesunuto v rámci WP4 (viz README.md v obou `_dead_20260710`
adresářích pro detail per soubor): `ekf_base.launch`,
`ekf_outdoor.launch`, `ekf_outdoor_all.launch` a jimi načítané
`ekf_base.yaml`, `ekf_outdoor.yaml`, `ekf_global.yaml`, `navsat.yaml`.
Jediné reference byly zakomentované includy ve `vitulus_navi_outdoor.launch`.

**POZOR na past**: `ekf_indoor.launch` a `ekf_base_indoor.yaml` VYPADAJÍ
podobně (stejný adresář, stejný vzor jména) ale jsou ŽIVÉ — viz sekce
výše. Nepřesouvat.

### `vitulus_bringup` — celý outdoor/indoor launch strom je mrtvý

`vitulus_bringup/launch/vitulus_outdoor.launch` a
`vitulus_bringup/launch/vitulus_indoor.launch` (a jejich XML podstrom)
nejsou z produkčního řetězce dosažitelné vůbec — nikde v
`vitulus_start.launch`, `nodes/main`, systemd unitě ani
`vitulus.startup` se `vitulus_bringup` nezmiňuje. Jediné reference na
`vitulus_bringup` v celém workspace jsou z `vitulus_slam/launch/vitulus_slam.launch`
a `vitulus/launch/t265.launch` — což jsou samy o sobě samostatné,
neintegrované vedlejší stromy, ne produkční cesta.

Navíc tento mrtvý strom má i vlastní ROZBITÉ reference (jen pro
info, NEOPRAVOVÁNO — je to mrtvý kód, netýká se WP4 „chirurgické úpravy"
pravidla):

- `vitulus_outdoor.launch` includuje `vitulus_gnss/launch/vitulus_ardusimple.launch`
  a `vitulus_gnss/launch/vitulus_ardusimple_lite.launch` — **ani jeden
  soubor neexistuje** (skutečné soubory v `vitulus_gnss/launch/` jsou
  `vitulus_ardusimple_gnss.launch` a `vitulus_ardusimple_heading.launch`,
  jiná jména).
  Include by při pokusu o spuštění shodil roslaunch s "file does not exist".
- `vitulus_indoor.launch` includuje `vitulus_navi/launch/launch_navi.launch`
  — **neexistuje** vůbec (žádný soubor toho jména kdekoli v `vitulus_navi`).

Pokud by měl někdo v budoucnu důvod `vitulus_bringup` oživit, tyhle dvě
rozbité reference je potřeba nejdřív opravit (pravděpodobně přepsat na
current `vitulus_navi_outdoor.launch`/`vitulus_navi_indoor.launch` a
skutečné GNSS launch soubory) — než to půjde vůbec spustit.

## Metodika ověření (pro příští audit)

Kandidát je „mrtvý", pokud `grep -rn <filename>` přes CELÝ
`/home/vitulus/catkin_ws/src` (všechny balíčky, ne jen `vitulus/`),
`/etc/systemd/system/vitulus.service` a `vitulus/launch/vitulus.startup`
nenajde nic mimo zakomentované XML (`<!-- -->` nebo dvojitě
escapované `&lt;!-- --&gt;`) nebo odkazy uvnitř jiného už-mrtvého
souboru. `.bak_*` soubory se do „živosti" nepočítají (jsou to zálohy).
