# vitulus_navi
 VITULUS navigation manager.\
 ROS package\
 It's the backend for webUI and manage maps and navigation.
   
 
![WebUI](https://github.com/lacina-dev/vitulus_navi/blob/main/WebUI.png?raw=true)


## Geofence proposals (`src/vitulus_navi/geofence/`)

Robot-native, **propose-only** writer that turns a perimeter *clicked* in the
web-UI map editor into an **UNSIGNED** geofence proposal. It lives here (a
safety-adjacent nav package that ships with the robot) so the public web node
`vitulus_ui/nodes/webnode` (`:7779`) can serve `POST /api/geofence/propose`
without importing anything from the private agent (`vitulus_claude`).

- `fence.py` — verbatim copy of the agent's `geofence.py` (the `Fence`
  geometry/validation the test supervisor also uses; pure stdlib).
- `sitebundle.py` — verbatim copy of the agent's `sitebundle.py`.
- `propose.py` — propose-only port of the agent's `geofence_propose.py`:
  `build_proposal()` / `write_proposal()` convert `map` metres → UTM33, reject
  rings that cross unmapped cells, and write `geofence.proposed.geojson` next to
  the site bundle. Because `fence.py` is a verbatim copy, the ring digest and
  file format match the agent path bit-for-bit.

**Signing is intentionally NOT here.** An active `geofence.geojson` is produced
only by a human running the agent's `tools/geofence_propose sign` (or
`/geofence podepsat <digest>`). This subpackage has no sign function, so nothing
reachable from the web node can arm a fence (AGENT.md §11.1, §11.5). The
canonical geofence *loader* the supervisor consumes remains the agent's
`geofence.py`; the copies here are used only to pre-validate proposals.

## Dependencies note

The global costmap uses the memoryless obstacle layer registered as
`costmap_2d::NonPersistentVoxelLayer`. Despite the `costmap_2d::` prefix, the
plugin is provided by the external package
[`nonpersistent_voxel_layer`](https://github.com/SteveMacenski/nonpersistent_voxel_layer)
(the C++ class lives there as `nonpersistent_voxel_layer::NonPersistentVoxelLayer`;
the `costmap_2d::` name is only the pluginlib registration). Install it with:

```bash
sudo apt install ros-noetic-nonpersistent-voxel-layer
```

## Magnetometer heading watchdog (`navi_transform`, off by default)

An independent check of the navigation heading: `navi_transform` compares the
map heading of the robot (yaw of map → base_link) with the magnetometer heading
`/bno085/mag_heading` (`vitulus_imu`, same yaw convention as
`/gnss_heading/navheading`, so `d = wrap(mag_yaw + 90° − nav_yaw)`). **It only
reports**: four fields in `/nav_tf/loc_status` and one log line when a
disagreement starts / ends. It never changes `diff_yaw`, map → odom, the EKF or
anything a controller reads. Enable with `~mag_check_enable` (`true`) on the
`navi_transform` node in `launch/navi_man.launch`; with it off nothing is
subscribed and `loc_status` has no `mag_*` field (the payload is unchanged).

| `loc_status` field | |
|---|---|
| `mag_state` | state of the magnetometer heading: `ok` / `disturbed` / `uncalibrated` / `stale` (also when its status topic is silent > 3 s) |
| `mag_check` | `ok` = compared now; else why not: `dock` (dock status 0/1/2, dock hold, or dock status never received / older than `~dock_status_stale_s` — unknown is not compared), `unanchored` (heading has no absolute anchor yet, or indoor), `error` (a heading is not a finite number, or the check itself failed), `stale` (no heading message for 1 s, or the magnetometer status topic silent > 3 s), `not_ok` (magnetometer not `ok`), `settle` (less than `~mag_check_settle_s` after leaving the dock or after a heading step) |
| `mag_diff_deg` | `d` in degrees (magnetometer − navigation, + = magnetometer more to the left); `null` when not compared |
| `mag_disagree` | `true` when the comparisons of the last `~mag_disagree_window_s` show a lasting `|d|` > `~mag_disagree_deg`; how it starts and ends is below |

Parameters (`~`): `mag_check_enable` (false), `mag_disagree_deg` (25),
`mag_disagree_s` (5), `mag_disagree_window_s` (120), `mag_disagree_share` (0.8),
`mag_check_settle_s` (8), `mag_disagree_hold_s` (60), `mag_clear_ratio` (0.6),
`mag_clear_s` (3), `mag_step_deg` (8), `mag_step_window_s` (3),
`mag_max_age_s` (1), `mag_status_max_age_s` (3), `mag_heading_topic`,
`mag_status_topic`.

**When it reports.** While driving the magnetometer is `ok` only in stretches
(median ~1 s; 8–38 % of the time on the 2026-11-08 drives, 46 % while mowing on
2026-11-14), so the decision is taken over a sliding window of the valid
comparisons of the last 120 s. A comparison counts for the time since the
previous one (the first of a stretch for 0.1 s, any for at most 1 s; a clock
that goes backwards empties the window). `mag_disagree` rises on a comparison
over the limit when, going back from it, there is a span holding at least 5 s
of comparison time over the limit which is at least 80 % of all the comparison
time of that span. So:

- a lasting error is reported once 5 s of comparisons have been collected:
  after ~5 s with the magnetometer `ok` all the time, ~25 s at 20 % `ok`,
  ~50 s at 10 %, ~100 s at 5 %; agreement before the error began does not
  delay it;
- **not reported:** an error while the magnetometer gives less than 5 s of `ok`
  per 120 s (under ~4 %), or is not `ok` at all (near the dock, strong
  disturbance); an error under the limit (25°); anything in the dock, during
  `settle`, or with an unknown dock state;
- short excursions between agreeing comparisons are not reported (e.g. 1 s
  over the limit in every 10 s, or 4.5 s over between 3 s of agreement);
  excursions that make up ≥ 80 % of everything compared are — also when
  nothing but they was compared for 2 min.

**How a report ends.** The same rule mirrored: 3 s (`mag_clear_s`) of
comparisons with `|d|` < 0.6 × the limit (15°) making up ≥ 80 % of the span —
3 s after agreement returns, ~30 s at 10 % `ok`. Also at once on a new heading
era (`dock` / `unanchored` / `settle`), and 60 s after the last valid
comparison when comparisons stop altogether. A heading that stays 15–25° off
after a report keeps it raised on purpose: the two headings still do not
agree, and `mag_diff_deg` shows by how much.

**Heading step** (starts `settle`): the watchdog sees it itself, no hook in the
heading code — `diff_yaw + map_odom_yaw` moved ≥ 8° net within 3 s
(`set_heading`, GNSS hand-over slew at 5°/s, map-pose yaw correction), or the
heading got its first anchor. The running RTK re-anchors do not count (on the
drives `diff_yaw` moved up to 2° per sample and 5° net within 1–2 s); a slew
slower than ~2.7°/s is not seen. An offset that keeps turning faster than that
(it should not exist) would hold the watchdog in `settle` for as long as it
lasts.

**On recorded drives** (everything through the node's watchdog path):

| | 2026-11-08 22:00 | 2026-11-08 23:58 | 2026-11-14 mowing |
|---|---|---|---|
| navigation heading | node heading code replayed over the bag | same | real (`/odometry/ekf_wheel_nav_odom`, `/tf`, dock status) |
| magnetometer heading | today's `vitulus_imu` code over the bag | same, and the topic in the bag | the topic in the bag |
| `mag_check` `ok` / `not_ok` / `settle` (anchored, out of the dock) | 35–38 % / 59–61 % / ≤ 1 % | 8–17 % / 82–92 % / ≤ 1 % | 59 % / 40 % / 1 % (mowing: 46 % / 53 % / 1 %) |
| reports at 25° | 0 | 0 | 0 (also 0 at 20° and 15°) |
| largest `|d|` while compared | 8.9° | 22.3° (see below) | 15.8° while mowing, 10.2° otherwise (p99 11.2° / 6.8°) |
| 30° error put into the navigation heading: time to the report | median 5.1 s, max 8.7 s | median 10–53 s, max 135 s | median 8.5 s, p90 19 s, max 35 s (mowing: 11.3 / 20.4 / 35.2 s) |

On the 23:58 drive the replayed node heading was 18.5° off the RTK heading for
~30 s after its first anchor (the heading gates on `/gnss*/fix` are not in the
bag, so the live node may have refused that sample); the watchdog showed up to
22.3° there — no report at 25°, a report with a 20° or 15° limit, and nothing
else in the stack would have noticed. The same 30° error entering through
`diff_yaw` is a heading step, so it is reported 8 s later. Test:
`tools/test_t35_mag_check.py`.

## Traversability layer (local costmap, off by default)

The local traversability map of `vitulus_traversability` (`/traversability/local_costmap`, 6 × 6 m in `odom`,
0..99 graded, 100 lethal, -1 unknown) can be added to the **local** costmap of move_base_flex as one more
`costmap_2d::StaticLayer` (`traversability_layer`, before `inflation_layer`; `use_maximum: true`,
`trinary_costmap: false`, `track_unknown_space: false` — the same pattern as `cloud_costmap_layer`). Parameters:
`config/local_costmap_traversability.yaml`.

**Off by default.** `launch/vitulus_navi_mbf.launch` loads that file only when the robot sets

    export VITULUS_TRAVERSABILITY_NAV=true    # this layer in the local costmap

and the node runs: since T10 (stack) the stack starts `traversability_local` by itself
(`vitulus/launch/includes/traversability.launch`; `vitulus.startup` sets `VITULUS_TRAVERSABILITY_NODE=true` when
`vitulus_traversability` is in the workspace, `=false` in `~/.bashrc` turns it off). The older start from
vitulus_depth_obstacles (`VITULUS_TRAVERSABILITY=true`) counts too; with it set `vitulus.startup` switches the stack
start off (`VITULUS_TRAVERSABILITY_NODE=false`), so the node never runs twice (two `/traversability_local` with respawn
would shut each other down endlessly). The switch checks `VITULUS_TRAVERSABILITY_NODE` OR `VITULUS_TRAVERSABILITY`

in `~/.bashrc` (sourced by `vitulus.service`; navi_man starts the navigation launch in its own process, so the
variables must be in its environment). On = `1`, `true`, `yes`, `on` (any case, spaces ignored); anything else
(unset, empty, `false`, a typo), or the layer without the node = off, and the parameters are exactly as without the switch
(`roslaunch --dump-params` / `--nodes` of `vitulus_navi_outdoor.launch`, `vitulus_navi_indoor.launch` and
`vitulus_navi_mbf.launch` identical, `test/test_traversability_switch.py`).

* **Switch on**: add the line, restart the stack (`vitulus.service`) — a restart of navi_man or the navigation
  alone keeps the old environment.
* **Switch off**: remove (or set to `false`) `VITULUS_TRAVERSABILITY_NAV`, restart the stack. The node itself keeps
  running (it only publishes, e.g. for the UI).

Why inside this launch: `vitulus_navi_mbf.launch` starts with `<rosparam command="delete" ns="move_base_flex"/>`,
which runs before its parameters are loaded and wipes every `/move_base_flex/...` parameter set by any other launch
(so `vitulus_traversability traversability.launch nav_integration:=local` cannot configure the layer: started before
the navigation its parameters are deleted, started after it they come too late). The switch loads the layer inside
the `move_base_flex` node after `local_costmap_params.yaml` (the plugin list is replaced as a whole; the list in
`local_costmap_traversability.yaml` must stay `local_costmap_params.yaml` + `traversability_layer`, the test checks
it).

**Needs the node running**: a costmap_2d StaticLayer blocks the start of move_base_flex until the first map on its
topic arrives ("Requesting the map..." in the move_base_flex log). `traversability_local` therefore publishes an
all-unknown map (-1, neutral here: `track_unknown_space: false`, `use_maximum: true`) every 0.5 s from its start
until its first D435 cloud with TF, so a broken camera or missing TF does not hold the navigation. But when the node
does not run at all, move_base_flex waits for good. The node runs only when **all** of these hold:

* `VITULUS_TRAVERSABILITY_NODE=true` (the stack start) or `VITULUS_TRAVERSABILITY=true` (the start from
  vitulus_depth_obstacles; checked by the switch: without either the layer stays off),
* `obstacles_engine: depth_obstacles` in `config/navi_manager.yaml` — only then navi_man starts
  `vitulus_depth_obstacles/depth_obstacles.launch`, the source of the clouds (and of the node with
  `VITULUS_TRAVERSABILITY=true`); with `rtabmap` (the RTABMAP_OBSTACLES container) the stack-started node gets no
  clouds and publishes only its neutral grid, the layer adds nothing. **Not checked by the switch** (the launch cannot
  read navi_manager.yaml): never switch on with `obstacles_engine: rtabmap`,
* `vitulus_traversability` is built on the robot.

Checked locally (own roscore, fake TF and maps, `move_base_flex` of this launch): switched on, the local costmap
carries the traversability cost (lethal 100 → 100, graded 50 → 49, unknown → 0) and a stale `/move_base_flex/...`
parameter from before is gone; with the real `traversability_local` and no camera at all the local costmap is
published (neutral grid); with no traversability publisher at all it is not published (move_base_flex waits) and
comes up when a map arrives.
