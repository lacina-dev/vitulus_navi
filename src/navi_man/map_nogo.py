"""The owner's nogo areas in /navi_manager/map whatever the map source (T36 Z1).

/navi_manager/map is the static layer of the global and the local costmap of
move_base_flex. The nogo areas of the editor get into it in ONE place only:
mapping_manager stamps them into /mapping_manager/site_map, which navi_man
serves while its map source is "octomap". With the source "planner" (set by
every start of the navi launch at run time: save / edit of a map, or by
/navi_manager/show_map) the grid is whatever node_planner shows on
/web_plan/map_edited: usually a derivative of the site raster, but refreshed
only after a debounce and never during a mission, and a layer view picked in
the editor (free polygons, a zone, ...) has no nogo at all. "rtabmap" and the
bootstrap map never had one.

Nogo.apply() puts the active nogo areas of /map_objects/rules (map frame, the
ones path_rules.RuleSet says apply) into any such grid as obstacles, the same
way mapping_manager does (vitulus_mapping editmask: floor to the cell that
contains the vertex, cv2.fillPoly, value 100). A grid whose cells are already
100 there comes back as the very same object, so nothing changes for it.

No ROS in here; cv2 is imported on first use (navi_man must start without it).
"""
import math

OBSTACLE = 100
FRAME = 'map'


def signature(nogos):
    """Comparable form of [(name, [(x, y)])]."""
    return tuple((name, tuple((float(x), float(y)) for x, y in pts)) for name, pts in nogos)


def _geometry(msg):
    i = msg.info
    p, q = i.origin.position, i.origin.orientation
    return (i.width, i.height, i.resolution, p.x, p.y, q.x, q.y, q.z, q.w)


def cells(nogos, info):
    """Flat indexes (iy * width + ix) of the cells the nogo areas cover."""
    import cv2
    import numpy as np
    w, h, res = int(info.width), int(info.height), float(info.resolution)
    ox, oy = info.origin.position.x, info.origin.position.y
    q = info.origin.orientation
    yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
    c, s = math.cos(yaw), math.sin(yaw)
    mask = np.zeros((h, w), dtype=np.uint8)          # image axes [iy, ix]
    for _name, pts in nogos:
        if len(pts) < 3:
            continue
        v = np.empty((len(pts), 2), dtype=np.float64)
        for k, (x, y) in enumerate(pts):
            dx, dy = x - ox, y - oy
            if yaw != 0.0:                            # into the grid's own axes
                dx, dy = c * dx + s * dy, -s * dx + c * dy
            v[k, 0] = dx / res
            v[k, 1] = dy / res
        cv2.fillPoly(mask, [np.floor(v).astype(np.int32).reshape(-1, 1, 2)], 1)
    return np.flatnonzero(mask).tolist()


class Nogo(object):
    """Remembers the cells of the last (nogo set, grid geometry) and the last
    stamped grid, so a grid that comes again costs a few thousand lookups."""

    def __init__(self):
        self._key = None
        self._cells = []
        self._src = None            # the last grid stamped ...
        self._out = None            # ... and what it became
        self.added = 0              # cells turned into obstacles in the last grid
        self.error = None           # why the last grid went out as it came

    def apply(self, msg, nogos):
        """msg with the nogo areas as obstacles. The same object when there is
        nothing to add; msg itself is never modified. Never raises: a grid that
        cannot be stamped goes out as it came (self.error says why)."""
        self.added, self.error = 0, None
        if msg is None or not nogos:
            return msg
        try:
            return self._apply(msg, nogos)
        except Exception as e:  # noqa: BLE001 - the map must go out
            self.error = repr(e)
            return msg

    def _apply(self, msg, nogos):
        info = msg.info
        n = int(info.width) * int(info.height)
        if n == 0:
            return msg                              # node_planner's "no map" grid
        if msg.header.frame_id != FRAME:
            self.error = 'frame {!r}'.format(msg.header.frame_id)
            return msg
        if not info.resolution > 0.0 or len(msg.data) != n:
            self.error = 'bad grid'
            return msg
        key = (signature(nogos), _geometry(msg))
        if key != self._key:
            self._cells = cells(nogos, info)
            self._key = key
            self._src = self._out = None
        if msg is self._src:
            self.added = self._added
            return self._out
        data = msg.data
        todo = [i for i in self._cells if data[i] != OBSTACLE]
        out = msg
        if todo:
            new = list(data)
            for i in todo:
                new[i] = OBSTACLE
            out = type(msg)()
            out.header = msg.header
            out.info = msg.info
            out.data = new
        self._src, self._out, self._added = msg, out, len(todo)
        self.added = len(todo)
        return out
