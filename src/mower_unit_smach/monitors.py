"""
SMACH MonitorState callbacks for the MISSION_CONCURRENCE container.

Each callback returns True to keep monitoring, False to trigger preemption
of the Concurrence (which preempts the mission child SM).
"""
import rospy


def weather_monitor_cb(userdata, msg):
    """Monitor /weather_alert/rain_alert (weather_alert/RainAlert).

    Pravidlo 2: Trigger return-to-dock if RAIN predicted in 2+ nowcast steps.
    """
    nowcast = [msg.status_nowcast10m, msg.status_nowcast20m, msg.status_nowcast30m]
    rain_statuses = ('RAIN', 'ALERT', 'WARN')
    rain_count = sum(1 for s in nowcast if s in rain_statuses)
    if rain_count >= 2:
        rospy.logwarn("[WEATHER_MONITOR] Rain predicted in %d/3 nowcast steps", rain_count)
        return False
    return True


def battery_monitor_cb(userdata, msg):
    """Monitor /pm/power_status (vitulus_msgs/Power_status).

    Pravidlo 3: Trigger return-to-dock if battery_capacity <= 25%.
    """
    if msg.battery_capacity <= 25:
        rospy.logwarn("[BATTERY_MONITOR] Battery capacity=%d%% <= 25%%",
                      msg.battery_capacity)
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
