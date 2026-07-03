# Vitulus — Unified Localization Plan (rtabmap day/night/no-GPS, seasonal)

_Created 2026-06-21 from a multi-agent rtabmap config audit + adversarial review.
Return point for next weekend. Rollback for the rtabmap config:
`cp rtabmap_outdoor.launch.orig rtabmap_outdoor.launch` in `src/rtabmap_docker/`._

## Goal
ONE rtabmap map that relocalizes **day and night**, **with or without GPS**, is
**accurate and fast** to relocalize in (kidnap / cold-start / return-to-map), and
is **editable + extendable over seasons** so localization stays equally good.
Sources: wheel odom, VO, lidar-ICP, RTK GPS, IMU. The dock is indoors (no GPS) so
**cold-start without GPS is the normal case**, and **at night the 360° lidar must
carry relocalization** (camera dead).

## Architecture (target)
- **Layer 1 (odom, always):** robot_localization EKF `ekf_wheel_nav_odometry`
  fuses wheel+IMU+VO+licp → continuous `odom→base_link` (built + validated). Never jumps.
- **Layer 2 (map):** rtabmap (camera+lidar, GPS-agnostic db) → `map→odom`, with
  relocalization (loop closure + lidar proximity). GPS georeference stays an OUTER
  `map→utm` layer (navsat_transform), so the db reusability does NOT depend on GPS.
- **Layer 2b (absolute anchors):** discrete absolute pose fixes fused into the map
  layer when available — **GPS/RTK** outdoors, **rtabmap relocalization**, and **the
  DOCK** (see below). The dock is the most reliable anchor: GPS-free, day/night,
  precise, and present at every mission start.

## Dock as a localization anchor (Layer 2b)
The dock is a **GPS-free, day/night absolute landmark** with a known pose in the map.
It directly solves the hardest case — **cold-start at the (indoor, no-GPS) dock** —
and gives a recurring drift correction every time the robot docks or even just *sees*
the dock. EXISTING machinery to reuse (do NOT reinvent — audit it in Phase 0):
- `vitulus_dock` **intensity dock detector** (lidar retro-reflectors): `/dock_detector/intensity/dock`,
  `/dock_detector/intensity/status`, `/dock_detector/dock` — detects the dock pose
  RELATIVE to base_link, from a distance, day/night (lidar). Reverse-in docking is
  mechanically repeatable → docked pose is precise.
- `dock_detector/dock_localize_map.py` (`AlignScanInMap`): scan-to-map ICP that
  already publishes `/odometry/robot_map_odom` and `/odometry/dock_odom`.
- `navi_man` `get_point_pose("dock")` / `/navi_manager/map_point_pose`: per-map stored
  point poses (the dock pose lives here today). `navi_transform` already has
  `callback_set_dock_pose` → set_pose to the dock point (manual/webui trigger today).

DESIGN (to add):
1. **Every used map stores the dock's pose** (a named landmark). Standardize this so
   the dock pose is always present and trustworthy per map.
2. **Detection → absolute fix:** when the intensity detector sees the dock, combine the
   relative dock pose with the stored map dock pose → an absolute robot pose in the map.
   Feed it as a **landmark/prior into rtabmap** (graph anchor; with a fixed global
   landmark id the dock anchors EVERY season's session to one physical point — perfect
   for Req 3) AND/OR a corrective `set_pose` into the EKF (Layer 1, immediate).
3. **Cold-start:** boot while docked → initialise pose = dock pose immediately (no GPS,
   no relocalization needed). This is the normal mission start.
4. **Recurring correction:** every dock event re-anchors the pose (kills accumulated
   GPS-denied drift), and the dock doubles as a stable seasonal anchor.
5. **Trust/uncertainty:** feed with a tight covariance (mechanical repeatability ~cm) so
   the dock fix dominates when present. Auto-trigger on the docked/detected edge (not the
   manual webui trigger used today).

## Relocalization mechanics — VERIFIED 2026-06-27 (rtabmap docs / Labbé), and two failures
A kidnap test (night, no GPS, lidar-rich) did NOT relocalize; and a day "continue
mapping" drew a NEW disconnected map. Root causes, cross-checked against primary sources:

**THE governing fact:** rtabmap's **global, prior-free place recognition is APPEARANCE
(visual bag-of-words) ONLY.** _"There is no global localization with lidar, only for
camera"_ (Labbé, rtabmap_ros#912). Lidar only does ICP **refinement after a hypothesis
exists**. Proximity detection (`RGBD/ProximityBySpace`, `ProximityGlobalScanMap`) is
**LOCAL** — it searches within `RGBD/LocalRadius` of the current **odom-predicted pose**;
`ProximityGlobalScanMap` only makes the ICP *reference* the whole-map cloud, it is still
anchored to the current pose guess. **Neither can recover a true kidnap** (wrong prior).

- **Failure A (night kidnap) = FUNDAMENTAL, not a misconfig.** No camera → no prior-free
  global mechanism; lidar proximity needs a roughly-correct prior it doesn't have. No
  rtabmap param fixes this. **My earlier `night:=true` (ICP-only, camera off) made it
  WORSE** — it removed the only global mechanism. CORRECTED: camera stays ON, Strategy=2.
- **Failure B (new disconnected map) = workflow/config, FIXABLE.** Every mapping session
  starts a new map ID and only **merges on a loop closure** (`Rtabmap/StartNewMapOnLoopClosure`
  default false; wiki "Kinect-mapping"). If you enable mapping **before** relocalizing
  and without the old map in WM, you get two islands. Fix = (a) `Mem/InitWMWithAllNodes=true`
  (DONE — whole map in WM so a loop closure can merge), AND (b) the **localize-then-map
  workflow**: relocalize in localization mode (or seed a pose) FIRST, THEN switch to
  mapping via the runtime service `rtabmap/set_mode_mapping`. Keep `RGBD/OptimizeFromGraphEnd=false`
  (anchor to oldest node → preserves the loaded map frame). Never `delete_db_on_start`.

**THE fix for night/no-GPS relocalization = supply an EXTERNAL coarse prior, then lidar
refines (camera-independent):** rtabmap accepts an initial pose via the **`/rtabmap/initialpose`
topic** (`geometry_msgs/PoseWithCovarianceStamped`, == RViz "2D Pose Estimate";
`setInitialPose()` works in LOCALIZATION mode and forces `map→odom`), the `initial_pose`
launch param, or the `global_pose` prior topic.
- **Primary (this is Layer 2b):** on undock / cold-start, publish the known **dock map
  pose → `/rtabmap/initialpose`**. Lidar proximity then refines it with no camera. Cheapest,
  most reliable night/no-GPS solution; no new package.
- **Secondary (true kidnap away from dock):** **AMCL** on rtabmap's exported 2D occupancy
  grid — its `global_localization` service does prior-free particle dispersion from the
  360° lidar; feed the converged pose to `/rtabmap/initialpose`. Run as a prior-FEEDER
  only (rtabmap stays the single `map→odom` owner). Caveat: AMCL is weak on open, feature-
  poor lawn. (Scan-Context is 3D-lidar only — N/A to our 2D scanner.)
- Keep **camera ON + localization mode** so daytime/lit visual loop closures still
  auto-correct, and night runs never modify (poison) the saved map.

## Config applied now (active `rtabmap_outdoor.launch` = hardened unified)
Real fixes (the lidar-reloc unlock + accuracy + db hygiene):

| Param | was | now | why |
|---|---|---|---|
| Icp/MaxCorrespondenceDistance | 0.03 | 0.1 | was < VoxelSize 0.05 (broken) |
| RGBD/LocalRadius | 0.20 | 5.0 | 0.20 throttled proximity (start 5, not 10, to limit aliasing) |
| RGBD/ProximityPathMaxNeighbors | 0 (off) | 10 | one-to-many reloc — biggest lidar unlock |
| RGBD/ProximityOdomGuess | unset | true | seed proximity ICP from EKF pose → anti wrong-row |
| Reg/Force3DoF | false | true | single-ring 2D lidar |
| Grid/Sensor | 1 | 2 | lidar+depth grid (night-capable) |
| Grid/RangeMax | 2.7 | 8.0 | costmap range |
| Grid/RayTracing | unset | true | re-clear stale seasonal obstacles |
| RGBD/NeighborLinkRefining | false | true | accuracy when loops scarce |
| Mem/NotLinkedNodesKept | true(def) | false | stop ~25 MB/min db bloat while idle |
| Icp/CorrespondenceRatio | 0.1(def) | 0.2 | reject low-overlap aliased loops |
| Icp/MaxTranslation | unset | 0.5 | anti-teleport |
| odom_tf_linear_variance | 1e-4 | 1e-3 | let loops correct drift |
| odom_tf_angular_variance | 1e-4 | 4e-3 | heading-dominated drift (angular > linear) |
| RGBD/OptimizeMaxError | 0.0 | 3.0 | finite gate vs aliased loops in symmetric field |
| Optimizer/Robust, Iterations | false,0 | true,20 | graph optimization ON (loop closures correct map) |

Mode arg: `localization:=true` (+ RGBD/ProximityGlobalScanMap, MaxOdomCacheSize=20,
DetectionRate=4). `Mem/InitWMWithAllNodes=true` in ALL modes (loads the whole map to WM
so loop closures can merge — fixes mapping-restart drawing a new map). **The `night`
arg was REMOVED 2026-06-27**: turning the camera off / ICP-only removes rtabmap's only
prior-free global relocalization (visual BoW) — camera stays ON, Reg/Strategy=2, always
(dark frames don't poison the map in localization mode). LocalRadius 5→8 (tolerate a
coarse seed prior).
Explicit **no-ops** (already rtabmap default; do NOT credit them with fixes):
RGBD/ForceOdom3DoF=true, Icp/PointToPlane=false, Optimizer/PriorsIgnored=true.
`Icp/Strategy` (libpointmatcher) left default — **verify the container build before enabling** (Phase 0).

## Phased plan (hardened)

**Phase 0 — Ground truth & pre-flight (~45 min, do FIRST).** Back up db + launch.
Verify libpointmatcher present in the rtabmap container (gates Icp/Strategy=1).
**Survey 3–4 physical marks (dock + corners) with RTK, record their UTM coords** —
this is the GPS-independent kidnap ground truth. Log baseline `map→utm` at the dock.
**Audit the existing dock-localization stack** so Layer 2b reuses it: read
`vitulus_dock/src/dock_detector/dock_localize_map.py` (AlignScanInMap, what
`/odometry/robot_map_odom` and `/odometry/dock_odom` actually are/where they're used),
inspect `/dock_detector/intensity/status` + `/dock_detector/dock` (rates, frames,
accuracy when docked vs detected-from-distance), and confirm how/where the dock pose
is stored per map (`navi_man get_point_pose("dock")`).

**Phase 1 — Fix lidar mapping path, build ONE daytime map.** Apply config (done).
Mapping mode, fresh db, drive a representative loop by day, **dwell ≥5 s at an
anchor** so loops form. **PASS =** `rtab_prox` climbs to dozens (was 2), `rtab_ref`
grows, ≥1 real loop on revisit (match ratio >0.2), **single connected component**
in databaseViewer, no z/roll/pitch wobble, costmap visible to ~8 m all-round, db
flat while docked, occupancy free-space re-clears (RayTracing). If CPU saturates at
2 Hz, drop ProximityPathMaxNeighbors to 5 before lowering LocalRadius.

**Phase 2 — Night + kidnap reloc, GPS-OFF, surveyed ground truth (make-or-break).**
`localization:=true night:=true`. Relocalize against the Phase-1 map **with GPS
disabled**, at the surveyed marks, at night. **PASS requires BOTH:** (a) reloc
converges within ~0.3 m of the **surveyed coordinate** (NOT live GPS), AND (b) a
co-timed `rtab_prox`/`rtab_loop` event — **no event = fail even if the pose looks
right** (guards the "0.14 m was just odom riding along" trap). `vo_lost=1`
throughout proves lidar carried it. ≥5 kidnaps from different marks, ≥80% success,
**zero false-locks** (a wrong-row snap = hard fail). On false-locks: confirm
ProximityOdomGuess, raise CorrespondenceRatio→0.3, lower MaxTranslation, add an
RTK-heading sanity gate. Sweep LocalRadius 5→10 only if cold-start fails.

**Phase 2b — Dock anchor integration (Layer 2b).** Wire the dock as an absolute
localization anchor. Steps: (a) ensure every map reliably stores the dock pose;
(b) on the docked/detected edge, turn the intensity detector's relative dock pose +
the stored map dock pose into an absolute fix and **publish it to `/rtabmap/initialpose`**
(localization mode; forces `map→odom`) — lidar proximity then refines it without the
camera (verified mechanism, see section above). Optionally also a fixed-global-id rtabmap
landmark to anchor every seasonal session to the one physical dock; (c)
**cold-start:** on boot while docked, initialise pose = dock pose (no GPS / no reloc).
**PASS =** powering on at the dock yields the correct map pose immediately (GPS off,
night); after a long GPS-denied loop, docking snaps the pose back to the surveyed dock
coordinate; the dock landmark appears as a graph constraint in databaseViewer and is
shared across two sessions. Reuse `dock_localize_map.py` / `/odometry/robot_map_odom`
rather than rebuilding scan-to-map. Guard: a SPURIOUS dock detection must not teleport
the pose — gate on intensity-detector confidence + agreement with current estimate.

**Phase 3 — GPS-denial & map integrity.** Validate the **real** georef path:
`map→utm` stability across dock→outdoor→dropout; confirm NO reloc discontinuity on
GPS acquire (should be none by design — db is GPS-agnostic). Characterize **EKF
drift vs cumulative yaw** over ≥3 runs before locking `odom_tf_*`. A/B
`OptimizeMaxError` 0.0 vs 3.0 in a symmetric area at night; production db keeps the
setting that rejected the aliased loop (bias 3.0).

**Phase 4 — Seasonal editability, corruption-safe.** **Never append live to the
master.** Each session → its own dated db; merge **offline** via `rtabmap-reprocess`
on a copy; require **≥2 independent cross-session loops in different physical
locations** before accepting (one loop in a symmetric field is untrustworthy);
inspect every inter-session link in databaseViewer; `ReduceGraph` offline-only;
re-run the Phase-2 kidnap suite on the merged copy before promoting; keep pre-merge
master as rollback; pin `Db/TargetVersion` to the container's rtabmap version.

## Key failure modes to watch (from the adversarial review)
- **F1 Night camera** → use `night:=true` (ICP-only, camera off); don't run VisIcp on a dead camera.
- **F2 Lidar aliasing** in open/symmetric/hedge-row fields → ProximityOdomGuess + finite OptimizeMaxError + MaxTranslation + RTK-heading gate.
- **F5 Georef datum drift** over seasons → pin a fixed `datum` in navsat_transform; validate the dock's UTM each startup; keep db GPS-agnostic.
- **F6 Kidnap success unfalsifiable** → require a discrete reloc event + wrong initial pose + surveyed (not GPS) ground truth.
- **F7 Static odom variance** underweights heading drift → angular > linear; characterize drift vs yaw, don't trust one run.
- **F8 Multi-session merge** corrupts via one aliased inter-session loop → ≥2 loops, offline, inspect, kidnap-test before promote.

## Telemetry already in place (vitulus_claude navigation.yaml)
`rtab_loop`, `rtab_prox`, `rtab_ref` (relocalization events / map growth),
`reloc_x/y` (`/rtabmap/localization_pose` vs `gps_odom` for accuracy),
`vo_lost`/`licp_lost` (camera vs lidar health). Restart the telemetry logger to pick them up.

## Open decisions for next weekend
- Wire `navi_man` to launch the unified outdoor (point `self.outdoor` at it) vs the manual file-swap currently in place.
- libpointmatcher → enable `Icp/Strategy=1` if present.
- Final `LocalRadius` (5 vs 10) and `OptimizeMaxError` (0 vs 3) from Phase-2/3 data.
- **Dock anchor:** rtabmap landmark (fixed global id, anchors all seasons) vs EKF
  set_pose vs fuse `/odometry/robot_map_odom` as an absolute source — or a combination.
  And: standardize per-map dock-pose storage; auto-trigger on docked edge (replace the
  manual webui set_dock_pose); spurious-detection guard.
