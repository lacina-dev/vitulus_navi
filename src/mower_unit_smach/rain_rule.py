"""The rain rule of a mission start, without ROS.

One function, two callers: PreStartCheck decides with it and master_controller
(package vitulus_planner) asks it for the Autonomy page, so what the page says
and what the mission does cannot differ.

rain is a weather_alert/RainAlert (any object with its fields).  cfg:
    recent_min       int    rain this many minutes back still refuses (0 = only now)
    n_steps          int    nowcast steps looked at (1..3, 10 min each)
    reject_statuses  tuple  statuses that count as rain
"""

PAST_FIELDS = ('status_past60m', 'status_past50m', 'status_past40m', 'status_past30m',
               'status_past20m', 'status_past10m')
NOWCAST_FIELDS = ('status_nowcast10m', 'status_nowcast20m', 'status_nowcast30m')


def horizon_steps(horizon_min):
    """(horizon, steps): the message only carries 10/20/30-minute steps."""
    horizon = min(max(int(horizon_min), 10), 30)
    return horizon, max(1, horizon // 10)


def recent_fields(recent_min):
    """The history fields within recent_min: 60 = all six, 0 = none."""
    keep = max(0, min(int(recent_min), 60)) // 10
    return PAST_FIELDS[len(PAST_FIELDS) - keep:]


def nowcast_steps(rain, n_steps):
    return [getattr(rain, f, '') for f in NOWCAST_FIELDS[:n_steps]]


def start_verdict(rain, cfg):
    """None = dry enough to start, else the reason of rejected:<reason>."""
    if rain.rain_now == 1:
        return 'rain'
    if rain.status_now == 'RAIN' or any(
            getattr(rain, f, '') == 'RAIN' for f in recent_fields(cfg['recent_min'])):
        return 'recent_rain'
    if any(s in cfg['reject_statuses'] for s in nowcast_steps(rain, cfg['n_steps'])):
        return 'forecast'
    return None


def is_fresh(stamp_s, now_s, max_age_s):
    """A RainAlert older than max_age_s is no data: the node keeps publishing
    its last result when the radar source is unreachable. No stamp counts."""
    return stamp_s <= 0 or now_s - stamp_s <= max_age_s


def checks_active(mode, scheduled, ignore_once):
    """Do the rain checks hold for this run?  mode: always | calendar | ignore.
    "Ignore this time" is a person at a Run: a calendar run, with nobody watching,
    never takes it, whatever is left armed."""
    if mode == 'ignore':
        return False
    if ignore_once and not scheduled:
        return False
    return scheduled or mode != 'calendar'
