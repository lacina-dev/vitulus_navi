"""
Centralised ROS publishers for the mower SMACH node.

All publishers are created once at startup and shared across states,
eliminating per-execute publisher leaks.
"""
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
        self.save_program = rospy.Publisher(
            '/web_plan/program_new', PlannerProgram, queue_size=1)

        # --- Mower hardware control ---
        self.mower_set_power = rospy.Publisher(
            '/mower/set_power', Bool, latch=True, queue_size=1)
        self.mower_set_height = rospy.Publisher(
            '/mower/set_cut_height', Int16, latch=True, queue_size=1)
        self.mower_set_rpm = rospy.Publisher(
            '/mower/set_motor_rpm', Int16, latch=True, queue_size=1)
        self.mower_set_motor_on = rospy.Publisher(
            '/mower/set_motor_on', Bool, latch=True, queue_size=1)

        # --- Debug / visualisation ---
        self.current_path = rospy.Publisher(
            '/test_path', Path, latch=True, queue_size=1)
