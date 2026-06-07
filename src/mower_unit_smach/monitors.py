"""
SMACH MonitorState callbacks for the MISSION_CONCURRENCE container.

Each callback returns True to keep monitoring, False to trigger preemption
of the Concurrence (which preempts the mission child SM).
"""
import rospy

from .helpers import get_weather_config, rain_forecast_steps


def weather_monitor_cb(userdata, msg):
    """Monitor /weather_alert/rain_alert (weather_alert/RainAlert).

    Pravidlo 2: Return to dock if the rain forecast over the configured horizon
    contains at least `monitor_min_steps` reject statuses. Statuses, horizon and
    a full bypass are configurable (see helpers.get_weather_config).
    """
    cfg = get_weather_config()
    if cfg['bypass']:
        return True
    nowcast = rain_forecast_steps(msg, cfg['n_steps'])
    rain_count = sum(1 for s in nowcast if s in cfg['reject_statuses'])
    if rain_count >= cfg['monitor_min_steps']:
        rospy.logwarn("[WEATHER_MONITOR] Rain (%s) in %d/%d forecast steps (<=%d min)",
                      "/".join(cfg['reject_statuses']), rain_count, cfg['n_steps'],
                      cfg['horizon_min'])
        return False
    return True


def battery_monitor_cb(userdata, msg):
    """Monitor /pm/power_status (vitulus_msgs/Power_status).

    Pravidlo 3: Trigger return-to-dock if battery_capacity <= return threshold.
    Threshold is configurable (~battery_return_pct); read per message so a live
    `rosparam set` takes effect without restarting the node.
    """
    return_pct = int(rospy.get_param('~battery_return_pct', 25))
    if msg.battery_capacity <= return_pct:
        rospy.logwarn("[BATTERY_MONITOR] Battery capacity=%d%% <= %d%%",
                      msg.battery_capacity, return_pct)
        return False
    return True


def mower_temp_monitor_cb(userdata, msg):
    """Monitor /mower/status (vitulus_msgs/Mower).

    Pravidlo 5: Trigger return-to-dock if motor state is TEMP.
    BLOCKED is handled locally inside ExecutePathWithFeedback.
    """
    if msg.status == 'TEMP':
        rospy.logwarn("[MOWER_TEMP_MONITOR] Motor overtemperature detected")
        return False
    return True


def stop_monitor_cb(userdata, msg):
    """Monitor /mower_smach/stop (std_msgs/Bool).

    Pravidlo 4: Trigger TERMINAL_ERROR on stop signal.
    """
    if msg.data:
        rospy.logwarn("[STOP_MONITOR] Stop signal received")
        return False
    return True
