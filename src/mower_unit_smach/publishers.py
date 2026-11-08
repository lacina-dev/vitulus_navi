"""
Centralised ROS publishers for the mower SMACH node.

All publishers are created once at startup and shared across states,
eliminating per-execute publisher leaks.
"""
import json

import rospy
from std_msgs.msg import Bool, String, Int16
from nav_msgs.msg import Path
from vitulus_msgs.msg import PlannerProgram


class MowerPublishers:
    """Container for all ROS publishers used by mower_unit_smach."""

    def __init__(self):
        # --- SMACH status / UI ---
        self.smach_status = rospy.Publisher(
            '/mower_smach/status', String, latch=True, queue_size=1)
        self.stop_reason = rospy.Publisher(
            '/mower_smach/stop_reason', String, latch=True, queue_size=1)
        self.log_info = rospy.Publisher(
            '/nextion/log_info', String, queue_size=10, latch=True)
        self.active_program = rospy.Publisher(
            '/mower_smach/active_program', String, latch=True, queue_size=1)
        # How far a chain of programs got (chain.py), latched JSON.
        self.chain_state = rospy.Publisher(
            '/mower_smach/chain', String, latch=True, queue_size=1)
        # Program name once the mission is past every start check (it leaves
        # the dock now), "" at each mission start. That is what makes a slot
        # "run" for master_controller.
        self.mission_started = rospy.Publisher(
            '/mower_smach/mission_started', String, latch=True, queue_size=1)
        # True while the SM is anywhere but idle (WAIT_FOR_PROGRAM, STOPPED,
        # TERMINAL_ERROR): the supervisor and cmd_vel_mux send
        # /mower_smach/stop only then (18. 10. 2026).
        self.busy = rospy.Publisher(
            '/mower_smach/busy', Bool, latch=True, queue_size=1)
        self.pm_play_melody = rospy.Publisher(
            '/pm/play_melody', Int16, queue_size=10)
        self.show_map_layer = rospy.Publisher(
            '/web_plan/show_map_layer', String, latch=True, queue_size=1)

        # --- Navigation speed preset (same topic as webui speed buttons) ---
        self.navi_speed = rospy.Publisher(
            '/navi_manager/speed', String, latch=True, queue_size=1)

        # --- Docking ---
        self.dock_cancel = rospy.Publisher(
            '/dock_smach/stop', Bool, queue_size=1)

        # --- Program persistence ---
        # Two programs go out at once at a chain's boundary (NextProgramState).
        self.save_program = rospy.Publisher(
            '/web_plan/program_new', PlannerProgram, queue_size=2)

        # --- Mower hardware control ---
        # Blade and power are commands, not state: latched, a new serial_node gets the
        # last one replayed, the mission's True over somebody's False, and a True of
        # set_power resets a healthy MCU.  The mission waits for the board (status)
        # before its True, so a command never has to wait for a subscriber.
        self.mower_set_power = rospy.Publisher(
            '/mower/set_power', Bool, queue_size=1)
        self.mower_set_height = rospy.Publisher(
            '/mower/set_cut_height', Int16, latch=True, queue_size=1)
        self.mower_set_rpm = rospy.Publisher(
            '/mower/set_motor_rpm', Int16, latch=True, queue_size=1)
        self.mower_set_motor_on = rospy.Publisher(
            '/mower/set_motor_on', Bool, queue_size=1)
        # The zone mowed now, for map_rules_guard's zone fence: the blade cuts only
        # inside it.  Latched JSON {program, zone, frame, points}, {} between zones.
        self.active_zone = rospy.Publisher(
            '/mower_smach/active_zone', String, latch=True, queue_size=1)

        # --- Debug / visualisation ---
        self.current_path = rospy.Publisher(
            '/test_path', Path, latch=True, queue_size=1)

    def zone(self, program='', zone=None):
        """Say which zone is mowed now (a MapEditZone), None = none."""
        self.active_zone.publish(String(zone_payload(program, zone)))


def zone_payload(program='', zone=None):
    """/mower_smach/active_zone of a MapEditZone, '{}' when it has no outline."""
    if zone is None:
        return '{}'
    points = [[round(p.x, 3), round(p.y, 3)] for p in zone.polygon.polygon.points]
    if len(points) < 3:
        return '{}'
    return json.dumps({'program': program or '', 'zone': zone.name,
                       'frame': zone.polygon.header.frame_id or 'map', 'points': points})
