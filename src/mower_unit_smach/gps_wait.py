"""
Adaptive time budget for the "wait for RTK" gates (no ROS imports, unit-testable
offline: see tools/test_mission_undock_gate.py).

Used by the mission gate WAIT_FOR_GPS_FIX. dock_smach UNDOCK_WAIT_FOR_GPS in
vitulus_dock (src/dock_manager/gps_wait.py) keeps an identical copy - change
both together.

Why: after a cold start (hard reboot) the heading receiver reaches RTK only
~15-17 min after boot, ~10 min after the main receiver. A fixed 600 s wait
therefore failed on every cold start although the fix was on its way.

Rule (all times in seconds, plain floats):
  - base budget: the wait ends at start + base_s unless progress is seen;
  - progress = the main receiver is seen in RTK (first time in this wait), or
    the heading receiver newly switches to RTK (not yet stable): the deadline
    moves to now + extend_s;
  - the deadline never goes past gnss_start + max_s, where gnss_start is when
    GNSS data was first seen by the node (first NavSatFix behind
    /device_state_pub/icon_status, i.e. the receivers' start after a boot);
    unknown gnss_start = the wait's own start. The base budget always holds.
So on a robot that has been up for a long time the wait stays at base_s, while
right after a reboot it can wait until ~max_s after the receivers came up.
"""

REBOOT_HINT = "after a reboot usually ~15 min"


def fmt_mmss(seconds):
    seconds = max(0, int(seconds))
    return "%d:%02d" % (seconds // 60, seconds % 60)


class GpsWaitBudget(object):

    def __init__(self, start, base_s=600.0, extend_s=900.0, max_s=1500.0,
                 gnss_start=None):
        self.start = float(start)
        self.base_s = float(base_s)
        self.extend_s = float(extend_s)
        self.max_s = float(max_s)
        self.gnss_start = self.start if gnss_start is None else float(gnss_start)
        self.deadline = self.start + self.base_s
        self._main_rtk_seen = False
        self._nav_rtk = False

    def cap(self):
        """Latest possible deadline (the base budget always holds)."""
        return max(self.start + self.base_s, self.gnss_start + self.max_s)

    def update(self, now, gnss, gnss_nav):
        """Feed the current receiver states (icon_status 'RTK', '3DFIX', 'BAD',
        'DISABLED'). Returns True when the deadline was extended."""
        progress = False
        if gnss == "RTK" and not self._main_rtk_seen:
            self._main_rtk_seen = True
            progress = True
        nav_rtk = (gnss_nav == "RTK")
        if nav_rtk and not self._nav_rtk:
            progress = True
        self._nav_rtk = nav_rtk
        if progress:
            new_deadline = min(float(now) + self.extend_s, self.cap())
            if new_deadline > self.deadline:
                self.deadline = new_deadline
                return True
        return False

    def expired(self, now):
        return float(now) >= self.deadline

    def limit_s(self):
        """Current length of the wait (deadline - start)."""
        return self.deadline - self.start

    def text(self, now, gnss, gnss_nav):
        """Display line (ASCII: the Nextion writes iso8859-1)."""
        return "Waiting for RTK: main %s, heading %s - %s / %s, %s" % (
            gnss or "?", gnss_nav or "?", fmt_mmss(float(now) - self.start),
            fmt_mmss(self.limit_s()), REBOOT_HINT)
