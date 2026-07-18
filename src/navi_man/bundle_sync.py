#!/usr/bin/env python3
"""
bundle_sync.py -- PURE conversion helpers between the legacy navi_man MapData
world (ROS Pose objects, MAP frame) and the canonical site-bundle world
(vitulus_mapping.bundle dicts, UTM metres).  WP-B of the site-bundle unification.

PURE PYTHON: imports only ``math`` (indirectly via geo) and ``vitulus_mapping.geo``.
It MUST NOT import rospy / numpy / ROS message types, so it is unit-testable
offline.  It therefore never CONSTRUCTS ROS message objects -- it consumes and
produces plain data:

  * MapData -> bundle:   map_points_to_waypoints / map_paths_to_paths
        Inputs are MapPoint / MapPath-like objects (duck typed: .name, .pose,
        .poses; a pose exposes .position.{x,y,z} and .orientation.{x,y,z,w}).
        Outputs are bundle dicts ready for bundle.save_waypoints / save_paths.

  * bundle -> MapData:   waypoints_to_point_specs / paths_to_path_specs
        Inputs are bundle.load_waypoints / load_paths dicts.  Outputs are pure
        MAP-frame "specs" (dicts of x/y/z/yaw floats) -- the caller (navi_man,
        which HAS rospy) turns these into real MapPoint / MapPath / Pose objects.

Geometry is converted MAP<->UTM through the source map's datum using geo's
affine helpers (the exact convention derived + verified in geo.py).  Waypoint /
path yaws are stored in UTM in the bundle (portable across maps/datums); the z
component is offset by the datum altitude when the datum carries one, since the
vertical axis is not rotated by the pure-yaw datum.
"""

import os
import sys

try:
    from vitulus_mapping import geo
except Exception:  # pragma: no cover - devel PYTHONPATH fallback (per plan §6)
    sys.path.append(os.path.join(os.path.dirname(__file__),
                                 "..", "..", "..", "vitulus_mapping", "src"))
    from vitulus_mapping import geo

__all__ = [
    "map_points_to_waypoints", "map_paths_to_paths",
    "waypoints_to_point_specs", "paths_to_path_specs",
    "pose_to_map_xyzyaw", "datum_exceeds",
]


# --------------------------------------------------------------------------
# pose extraction (duck typed; works on geometry_msgs/Pose and test fakes)
# --------------------------------------------------------------------------
def pose_to_map_xyzyaw(pose):
    """(x, y, z, yaw_map) from a Pose-like object.  yaw is the Z-euler of the
    quaternion (this codebase zeroes orientation x/y -> pure yaw)."""
    p = pose.position
    o = pose.orientation
    yaw = geo.quat_to_yaw(float(o.x), float(o.y), float(o.z), float(o.w))
    return float(p.x), float(p.y), float(p.z), yaw


# --------------------------------------------------------------------------
# MapData -> bundle dicts  (MAP frame -> canonical UTM)
# --------------------------------------------------------------------------
def map_points_to_waypoints(points, datum):
    """points: iterable of MapPoint-like (.name, .pose).  datum: any schema
    accepted by geo (pickle MapData obj/dict, datum.yaml dict, canonical dict).
    Returns list of bundle waypoint dicts {name, e, n, yaw_rad, z}."""
    d = geo.normalize_datum(datum)
    has_alt = "alt" in d
    out = []
    for pt in points:
        x, y, z, yaw_map = pose_to_map_xyzyaw(pt.pose)
        e, n = geo.map_to_utm(x, y, d)
        wp = {
            "name": pt.name,
            "e": e,
            "n": n,
            "yaw_rad": geo.map_yaw_to_utm(yaw_map, d),
            "z": (d["alt"] + z) if has_alt else z,
        }
        out.append(wp)
    return out


def map_paths_to_paths(paths, datum):
    """paths: iterable of MapPath-like (.name, .poses).  Returns list of bundle
    path dicts {name, vertices:[(e,n),...], yaws:[...]} (UTM)."""
    d = geo.normalize_datum(datum)
    out = []
    for pa in paths:
        verts = []
        yaws = []
        for pose in pa.poses:
            x, y, _z, yaw_map = pose_to_map_xyzyaw(pose)
            e, n = geo.map_to_utm(x, y, d)
            verts.append((e, n))
            yaws.append(geo.map_yaw_to_utm(yaw_map, d))
        out.append({"name": pa.name, "vertices": verts, "yaws": yaws})
    return out


# --------------------------------------------------------------------------
# bundle dicts -> MAP-frame specs  (canonical UTM -> MAP frame)
# --------------------------------------------------------------------------
def waypoints_to_point_specs(waypoints, datum):
    """waypoints: bundle.load_waypoints() list.  Returns MAP-frame point specs
    [{name, x, y, z, yaw}] (yaw in MAP frame, radians)."""
    d = geo.normalize_datum(datum)
    has_alt = "alt" in d
    out = []
    for wp in waypoints:
        x, y = geo.utm_to_map(wp["e"], wp["n"], d)
        z = wp.get("z")
        if z is None:
            z = 0.0
        elif has_alt:
            z = z - d["alt"]
        out.append({
            "name": wp.get("name"),
            "x": x,
            "y": y,
            "z": float(z),
            "yaw": geo.utm_yaw_to_map(float(wp.get("yaw_rad", 0.0)), d),
        })
    return out


def paths_to_path_specs(paths, datum):
    """paths: bundle.load_paths() list.  Returns MAP-frame path specs
    [{name, poses:[{x, y, z, yaw}]}]."""
    d = geo.normalize_datum(datum)
    out = []
    for pa in paths:
        yaws = pa.get("yaws") or []
        poses = []
        for i, v in enumerate(pa["vertices"]):
            x, y = geo.utm_to_map(v[0], v[1], d)
            yaw_utm = yaws[i] if i < len(yaws) else 0.0
            poses.append({
                "x": x,
                "y": y,
                "z": 0.0,
                "yaw": geo.utm_yaw_to_map(float(yaw_utm), d),
            })
        out.append({"name": pa.get("name"), "poses": poses})
    return out


# --------------------------------------------------------------------------
# datum guard convenience (per plan §3 -- informational, never auto-switch)
# --------------------------------------------------------------------------
def datum_exceeds(a, b, pos_tol=0.05, yaw_tol=0.002):
    """Return (exceeded_bool, (dE, dN, dAlt, dYaw)) for datums a,b (any schema).
    exceeded == |dE|>pos_tol or |dN|>pos_tol or |dYaw|>yaw_tol."""
    de, dn, dalt, dyaw = geo.datum_delta(a, b)
    exceeded = abs(de) > pos_tol or abs(dn) > pos_tol or abs(dyaw) > yaw_tol
    return exceeded, (de, dn, dalt, dyaw)
