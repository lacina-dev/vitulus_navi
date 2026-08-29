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
