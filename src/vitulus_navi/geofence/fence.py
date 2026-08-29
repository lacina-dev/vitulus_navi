"""UTM geofence: loading, containment and margin classification.

The fence is a single closed ring stored as GeoJSON in the site bundle
(`~/.vitulus/mapping_v3/<site>/geofence.geojson`), in the same CRS as the
bundle's `waypoints.geojson` / `paths.geojson`: **UTM metres, EPSG:32633**.

Everything here is pure geometry with no ROS or third-party dependency, so it
can be unit-tested and used both by the supervisor (live checks at 10 Hz) and
by the runner (validating waypoints and paths before the robot ever moves).

Sign convention: `clearance()` is **positive inside** the ring and negative
outside, measured in metres to the nearest boundary segment.
"""
import hashlib
import json
import math

CRS_NAME = 'urn:ogc:def:crs:EPSG::32633'

# A hand-driven perimeter smaller than this is almost certainly a mistake
# (a stationary recording, or a drive that never left the dock).
MIN_AREA_M2 = 4.0
MIN_VERTICES = 3


class GeofenceError(ValueError):
    pass


class Fence(object):
    """A closed UTM ring plus the margin logic used to stop the robot."""

    def __init__(self, ring, site=None, source=None, note=None):
        ring = [(float(e), float(n)) for e, n in ring]
        # Accept both explicitly closed and open rings; store open internally.
        if len(ring) >= 2 and _same_point(ring[0], ring[-1]):
            ring = ring[:-1]
        if len(ring) < MIN_VERTICES:
            raise GeofenceError('geofence needs at least %d distinct vertices, got %d'
                                % (MIN_VERTICES, len(ring)))
        self.ring = ring
        self.site = site
        self.source = source
        self.note = note
        self.area = abs(_signed_area(ring))
        if self.area < MIN_AREA_M2:
            raise GeofenceError('geofence area %.2f m2 is below the %.1f m2 sanity floor'
                                % (self.area, MIN_AREA_M2))
        crossing = _self_intersection(ring)
        if crossing:
            raise GeofenceError('geofence ring intersects itself between segments %d and %d'
                                % crossing)
        easts = [p[0] for p in ring]
        norths = [p[1] for p in ring]
        self.bounds = (min(easts), min(norths), max(easts), max(norths))

    # -- identity ---------------------------------------------------------
    @property
    def digest(self):
        """Short stable hash of the ring; lets the runner prove the supervisor
        loaded the *same* fence rather than merely *a* fence."""
        blob = ';'.join('%.3f,%.3f' % point for point in self.ring)
        return hashlib.sha256(blob.encode('utf-8')).hexdigest()[:12]

    def summary(self):
        west, south, east, north = self.bounds
        return ('%s vertices, %.0f m2, %.0f x %.0f m, digest %s'
                % (len(self.ring), self.area, east - west, north - south, self.digest))

    # -- geometry ---------------------------------------------------------
    def contains(self, east, north):
        """Ray-casting point-in-polygon test."""
        inside = False
        count = len(self.ring)
        for index in range(count):
            ax, ay = self.ring[index]
            bx, by = self.ring[(index + 1) % count]
            if (ay > north) != (by > north):
                # Horizontal ray to +east; find the crossing easting.
                crossing = ax + (north - ay) * (bx - ax) / (by - ay)
                if crossing > east:
                    inside = not inside
        return inside

    def distance_to_edge(self, east, north):
        """Unsigned metres to the nearest boundary segment."""
        count = len(self.ring)
        return min(_point_segment_distance((east, north), self.ring[index],
                                           self.ring[(index + 1) % count])
                   for index in range(count))

    def clearance(self, east, north):
        """Metres to the boundary; positive inside, negative outside."""
        distance = self.distance_to_edge(east, north)
        return distance if self.contains(east, north) else -distance

    def classify(self, east, north, warn_m, hard_m):
        """('OK'|'WARN'|'HARD'|'OUTSIDE', clearance_m) for a robot position."""
        clear = self.clearance(east, north)
        if clear <= 0.0:
            return 'OUTSIDE', clear
        if clear <= hard_m:
            return 'HARD', clear
        if clear <= warn_m:
            return 'WARN', clear
        return 'OK', clear

    def segment_clearance(self, start, end):
        """Worst clearance along a straight segment, in metres.

        A positive result proves the whole segment stays inside the ring: both
        endpoints are inside and the segment crosses no edge, so the exact
        minimum is the segment-to-boundary distance.

        A segment that leaves the ring is always strictly negative, including
        the case where both endpoints are inside but the line cuts across a
        concave notch — there the distance to the boundary is zero, so the
        depth is measured by sampling instead of being reported as 0.0.
        """
        count = len(self.ring)
        edges = [(self.ring[index], self.ring[(index + 1) % count]) for index in range(count)]
        distance = min(_segment_segment_distance(start, end, a, b) for a, b in edges)
        crosses = any(_segments_cross(start, end, a, b) for a, b in edges)
        if self.contains(*start) and self.contains(*end) and not crosses:
            return distance
        return min(self._worst_sampled(start, end), -1e-6)

    def _worst_sampled(self, start, end, step_m=0.5, max_samples=400):
        """Most negative clearance found along a segment known to leave the ring."""
        length = math.hypot(end[0] - start[0], end[1] - start[1])
        count = max(2, min(max_samples, int(length / step_m) + 1))
        worst = 0.0
        for index in range(count + 1):
            t = index / float(count)
            point = (start[0] + t * (end[0] - start[0]), start[1] + t * (end[1] - start[1]))
            worst = min(worst, self.clearance(*point))
        return worst

    def polyline_clearance(self, points):
        """Worst clearance along a polyline; negative if it leaves the ring."""
        if not points:
            raise GeofenceError('polyline has no points')
        if len(points) == 1:
            return self.clearance(*points[0])
        return min(self.segment_clearance(points[index], points[index + 1])
                   for index in range(len(points) - 1))


# ---- loading / saving -----------------------------------------------------
def load_geofence(path, site=None):
    """Read a geofence GeoJSON file and return a validated `Fence`."""
    try:
        with open(path, 'r') as stream:
            data = json.load(stream)
    except (IOError, OSError) as exc:
        raise GeofenceError('cannot read geofence %s: %s' % (path, exc))
    except ValueError as exc:
        raise GeofenceError('geofence %s is not valid JSON: %s' % (path, exc))

    crs = _crs_name(data)
    if crs and crs != CRS_NAME:
        raise GeofenceError('geofence CRS is %s, expected %s (UTM metres)' % (crs, CRS_NAME))

    ring = _first_polygon_ring(data)
    fence = Fence(ring, site=site, source=path, note=data.get('note'))
    return fence


def dump_geofence(fence, site=None):
    """GeoJSON document for a `Fence`, matching the site bundle conventions."""
    ring = list(fence.ring) + [fence.ring[0]]
    return {
        'type': 'FeatureCollection',
        'note': ('Coordinates are UTM metres (easting, northing), NOT lon/lat. '
                 'Closed ring = the area the robot may drive inside during tests.'),
        'crs': {'type': 'name', 'properties': {'name': CRS_NAME}},
        'features': [{
            'type': 'Feature',
            'geometry': {'type': 'Polygon', 'coordinates': [[[e, n] for e, n in ring]]},
            'properties': {'name': 'geofence', 'site': site or fence.site,
                           'area_m2': round(fence.area, 2),
                           'digest': fence.digest},
        }],
    }


def _crs_name(data):
    crs = data.get('crs')
    if isinstance(crs, dict):
        return (crs.get('properties') or {}).get('name')
    return None


def _first_polygon_ring(data):
    features = data.get('features')
    if isinstance(features, list):
        for feature in features:
            geometry = (feature or {}).get('geometry') or {}
            ring = _ring_from_geometry(geometry)
            if ring:
                return ring
        raise GeofenceError('geofence file has no Polygon or LineString feature')
    ring = _ring_from_geometry(data.get('geometry') or data)
    if ring:
        return ring
    raise GeofenceError('geofence file contains no usable geometry')


def _ring_from_geometry(geometry):
    kind = geometry.get('type')
    coordinates = geometry.get('coordinates')
    if kind == 'Polygon' and coordinates:
        return [(point[0], point[1]) for point in coordinates[0]]
    if kind == 'LineString' and coordinates:
        return [(point[0], point[1]) for point in coordinates]
    return None


# ---- primitive geometry ---------------------------------------------------
def _same_point(a, b, tolerance=1e-6):
    return abs(a[0] - b[0]) < tolerance and abs(a[1] - b[1]) < tolerance


def _signed_area(ring):
    total = 0.0
    count = len(ring)
    for index in range(count):
        ax, ay = ring[index]
        bx, by = ring[(index + 1) % count]
        total += ax * by - bx * ay
    return total / 2.0


def _point_segment_distance(point, start, end):
    px, py = point
    ax, ay = start
    bx, by = end
    dx, dy = bx - ax, by - ay
    length = dx * dx + dy * dy
    if length <= 0.0:
        return math.hypot(px - ax, py - ay)
    t = ((px - ax) * dx + (py - ay) * dy) / length
    t = max(0.0, min(1.0, t))
    return math.hypot(px - (ax + t * dx), py - (ay + t * dy))


def _segments_cross(a, b, c, d):
    def orient(p, q, r):
        value = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if abs(value) < 1e-12:
            return 0
        return 1 if value > 0 else 2

    o1, o2, o3, o4 = orient(a, b, c), orient(a, b, d), orient(c, d, a), orient(c, d, b)
    if o1 != o2 and o3 != o4:
        return True
    return False


def _segment_segment_distance(a, b, c, d):
    if _segments_cross(a, b, c, d):
        return 0.0
    return min(_point_segment_distance(a, c, d), _point_segment_distance(b, c, d),
               _point_segment_distance(c, a, b), _point_segment_distance(d, a, b))


def _self_intersection(ring):
    """(i, j) of the first pair of non-adjacent segments that cross, else None."""
    count = len(ring)
    for i in range(count):
        a, b = ring[i], ring[(i + 1) % count]
        for j in range(i + 1, count):
            if j == i or (j + 1) % count == i or (i + 1) % count == j:
                continue
            c, d = ring[j], ring[(j + 1) % count]
            if _segments_cross(a, b, c, d):
                return (i, j)
    return None
