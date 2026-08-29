"""Read-only access to a mapping_v3 site bundle.

The bundle is the robot's own source of truth for where it is allowed to go:
`waypoints.geojson` and `paths.geojson` hold the places a human saved, in UTM
metres (EPSG:32633).  The test harness addresses those places **by name**, so
validating a scenario means resolving the name here and checking the resulting
geometry against the geofence — never accepting free coordinates.
"""
import json
import os

BUNDLE_ROOT = os.path.expanduser('~/.vitulus/mapping_v3')
GEOFENCE_NAME = 'geofence.geojson'


class SiteError(ValueError):
    pass


def bundle_dir(site):
    path = os.path.join(BUNDLE_ROOT, site)
    if not os.path.isdir(path):
        raise SiteError('site bundle does not exist: %s' % path)
    return path


def geofence_path(site):
    return os.path.join(BUNDLE_ROOT, site, GEOFENCE_NAME)


def list_sites():
    try:
        return sorted(name for name in os.listdir(BUNDLE_ROOT)
                      if os.path.isdir(os.path.join(BUNDLE_ROOT, name)))
    except OSError:
        return []


def load_waypoints(site):
    """{name: (easting, northing)} for every saved Point in the bundle."""
    features = _features(site, 'waypoints.geojson')
    waypoints = {}
    for feature in features:
        geometry = feature.get('geometry') or {}
        name = (feature.get('properties') or {}).get('name')
        if geometry.get('type') != 'Point' or not name:
            continue
        coordinates = geometry.get('coordinates') or []
        if len(coordinates) >= 2:
            waypoints[str(name)] = (float(coordinates[0]), float(coordinates[1]))
    return waypoints


def load_paths(site):
    """{name: [(easting, northing), ...]} for every saved LineString."""
    features = _features(site, 'paths.geojson')
    paths = {}
    for feature in features:
        geometry = feature.get('geometry') or {}
        name = (feature.get('properties') or {}).get('name')
        if geometry.get('type') != 'LineString' or not name:
            continue
        points = [(float(point[0]), float(point[1]))
                  for point in geometry.get('coordinates') or [] if len(point) >= 2]
        if points:
            paths[str(name)] = points
    return paths


def load_datum(site):
    """The bundle datum (utm_e / utm_n / yaw_rad), or None when absent."""
    import yaml
    path = os.path.join(bundle_dir(site), 'datum.yaml')
    if not os.path.isfile(path):
        return None
    with open(path, 'r') as stream:
        return yaml.safe_load(stream)


def _features(site, filename):
    path = os.path.join(bundle_dir(site), filename)
    if not os.path.isfile(path):
        raise SiteError('site %s has no %s' % (site, filename))
    try:
        with open(path, 'r') as stream:
            data = json.load(stream)
    except ValueError as exc:
        raise SiteError('%s is not valid JSON: %s' % (path, exc))
    features = data.get('features')
    if not isinstance(features, list):
        raise SiteError('%s has no feature list' % path)
    return features
