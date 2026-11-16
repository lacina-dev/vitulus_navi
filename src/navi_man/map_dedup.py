"""Publish /navi_manager/map only when it really changed (T39 C2). Pure, no ROS.

16. 11. 2026: node_planner sends /web_plan/map_edited every 2 s and navi_man
took every arrival as "the map changed", so the served map (1035x1931 = 2 mil.
cells, the very same grid with the very same stamp) went out again on every
slow tick. Each copy makes the global costmap of move_base_flex take the whole
grid over and inflate it again, the local costmap copy its static layer and
rosbridge serialize 2 MB of JSON for the UI.

/navi_manager/map is a latched topic: whoever connects later (a restarted
move_base_flex, a static layer re-subscribing after clear_costmaps, a new UI
client behind rosbridge) gets the last published grid from the latch, so a
repeat of an identical grid carries no information.

MapDedup remembers what was published last and answers "is this the same
grid?" - same frame, geometry and cells; the header stamp is not part of it.
"""


def signature(msg):
    """Frame + geometry of an OccupancyGrid (everything but stamp and cells)."""
    i = msg.info
    p, q = i.origin.position, i.origin.orientation
    return (msg.header.frame_id, i.width, i.height, i.resolution,
            p.x, p.y, p.z, q.x, q.y, q.z, q.w)


class MapDedup(object):

    def __init__(self):
        self._sig = None
        self._data = None
        self.sent = 0
        self.skipped = 0

    def is_same(self, msg):
        """True when msg carries the grid that was published last."""
        if self._sig is None or signature(msg) != self._sig:
            return False
        data = msg.data
        if data is self._data:
            return True
        if type(data) is not type(self._data) or len(data) != len(self._data):
            return False
        return data == self._data

    def note_published(self, msg):
        self._sig = signature(msg)
        self._data = msg.data
        self.sent += 1

    def note_skipped(self):
        self.skipped += 1

    def forget(self):
        """The next grid goes out whatever it is (explicit request / new source)."""
        self._sig = None
        self._data = None
