"""Does a stored path cross a map rule?  (docs/MAP_OBJECTS.md section 5)

Pure python, no ROS and no shapely: navi_man imports this and must start
whatever happens here.  Used by navi_man before a path is sent and by
smach_exe_path as the backstop.

A body half as wide as the robot follows the path: a pose counts as crossing
when it is inside an area, or nearer to it or to a line than HALF_WIDTH.
"""
import json
import math

HALF_WIDTH = 0.256     # vitulus_navi/config/costmap_common_params.yaml footprint
STEP = 0.10            # m; stored paths can have poses metres apart


def parse_rules(raw):
    """/map_objects/rules JSON -> [(name, points)] of the nogo areas."""
    rules = json.loads(raw)['rules']
    return [(str(r.get('name', '')), [(float(x), float(y)) for x, y in r['points']])
            for r in rules if r.get('type') == 'nogo' and len(r.get('points') or []) >= 3]


class RuleSet(object):
    """The nogo areas that apply.  A bad message or site null (mapping_manager away
    for a moment) keeps the last good ones, another site's rules are in another map."""

    def __init__(self):
        self.site = None
        self.site_nogos = []
        self.served = None           # /mapping_manager/status, None = not known

    def on_rules(self, raw):
        try:
            site = json.loads(raw).get('site')
            nogos = parse_rules(raw)
        except (ValueError, TypeError, KeyError, AttributeError):
            return False
        if site is not None:
            self.site, self.site_nogos = site, nogos
        return True

    def on_status(self, raw):
        try:
            serving = json.loads(raw).get('serving')
        except (ValueError, TypeError, AttributeError):
            return
        self.served = serving.get('site') if isinstance(serving, dict) else None

    @property
    def nogos(self):
        if self.served is not None and self.site != self.served:
            return []
        return self.site_nogos


def parse_edits(raw):
    """/map_objects/objects JSON -> (lethal, free): lethal = [(name, points, closed)]
    of obstacle and wall edits, free = [points] of the areas that clear them."""
    lethal, free = [], []
    for o in json.loads(raw).get('objects') or []:
        if o.get('family') != 'edit' or not o.get('active', True):
            continue
        pts = [(float(x), float(y)) for x, y in o.get('points') or []]
        if o.get('type') in ('obstacle', 'wall') and len(pts) >= 2:
            lethal.append((str(o.get('name', '')), pts, o.get('geometry') == 'area'))
        elif o.get('type') == 'free' and len(pts) >= 3:
            free.append(pts)
    return lethal, free


def resample(xy):
    """The polyline with no gap over STEP."""
    out = []
    for (x1, y1), (x2, y2) in zip(xy, xy[1:]):
        n = max(1, int(math.ceil(math.hypot(x2 - x1, y2 - y1) / STEP)))
        out.extend((x1 + (x2 - x1) * i / n, y1 + (y2 - y1) * i / n) for i in range(n))
    out.extend(xy[-1:])
    return out


def inside(pt, poly):
    x, y = pt
    hit = False
    for (x1, y1), (x2, y2) in zip(poly, poly[1:] + poly[:1]):
        if (y1 > y) != (y2 > y) and x < x1 + (y - y1) * (x2 - x1) / (y2 - y1):
            hit = not hit
    return hit


def _dist_segment(pt, a, b):
    ax, ay = a
    dx, dy = b[0] - ax, b[1] - ay
    length = dx * dx + dy * dy
    t = 0.0
    if length > 0.0:
        t = max(0.0, min(1.0, ((pt[0] - ax) * dx + (pt[1] - ay) * dy) / length))
    return math.hypot(pt[0] - ax - t * dx, pt[1] - ay - t * dy)


def _near(pts, geom, closed, half):
    edges = list(zip(geom, geom[1:] + geom[:1])) if closed else list(zip(geom, geom[1:]))
    for pt in pts:
        if closed and inside(pt, geom):
            return True
        if any(_dist_segment(pt, a, b) < half for a, b in edges):
            return True
    return False


def near_nogo(xy, nogos, margin):
    """Indexes of the poses [(x, y)] within `margin` of a nogo."""
    return [i for i, pt in enumerate(xy)
            if any(_near([pt], poly, True, margin) for _name, poly in nogos)]


def path_refusal(xy, nogos=(), lethal=(), free=(), half=HALF_WIDTH):
    """Why the path [(x, y)] must not run, None when it may."""
    if not xy or not (nogos or lethal):
        return None
    pts = resample(list(xy))
    for name, poly in nogos:
        if _near(pts, poly, True, half):
            return 'crosses nogo {}'.format(name)
    if lethal:
        open_pts = [p for p in pts if not any(inside(p, f) for f in free)]
        for name, geom, closed in lethal:
            if _near(open_pts, geom, closed, half):
                return 'crosses edit {}'.format(name)
    return None
