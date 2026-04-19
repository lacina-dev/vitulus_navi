#!/usr/bin/env python
"""
Navigation-related states for the mower SMACH state machine.
"""
import rospy
import smach
import smach_ros
from std_msgs.msg import String
from mbf_msgs.msg import RecoveryAction
from mbf_msgs.srv import CheckPath, CheckPathRequest


class ActionGoalCallbacks:
    """Helper class for action goal and result callbacks."""
    
    @staticmethod
    def path_to_start_goal_cb(userdata, goal):
        """Prepare goal for get_path action to navigate to zone start."""
        # Ensure Move Base Flex is available
        client = actionlib.SimpleActionClient("/move_base_flex/move_base", mbf_msgs.MoveBaseAction)
        available = client.wait_for_server(rospy.Duration(5))
        if not available:
            rospy.logwarn("Move Base Flex is not immediately available")

    @staticmethod
    def path_to_start_result_cb(userdata, status, result):
        """Handle result from get_path action for zone start."""
        rospy.loginfo("Get path status: %s", status)
        rospy.loginfo("Get path result: %s", result.message)
        
        # Check for preemption (need to handle this in calling state machine)
        if result.outcome == 2:  # MBF Outcome: CANCELED
            return 'aborted'
        else:
            # Publish status information
            pub_log_info = rospy.Publisher('/nextion/log_info', String, queue_size=10, latch=True)
            pub_smach_status = rospy.Publisher('/mower_smach/status', String, latch=True, queue_size=1)
            pub_melody = rospy.Publisher('/pm/play_melody', Int16, queue_size=10)
            
            pub_log_info.publish(String("Go to zone begin"))
            pub_smach_status.publish(String("Go to zone begin"))
            pub_melody.publish(Int16(2))  # 2: double beep
            
            return 'succeeded'
    
    @staticmethod
    def path_to_path_start_goal_cb(userdata, goal):
        """Prepare goal for get_path action to navigate to path start."""
        # No specific goal preparation needed
        pass

    @staticmethod
    def path_to_path_start_result_cb(userdata, status, result):
        """Handle result from get_path action for path start."""
        rospy.loginfo("Get path status: %s", status)
        rospy.loginfo("Get path result message: %s", result.message)
        rospy.loginfo("Get path result outcome: %s", result.outcome)
        rospy.loginfo("Get path result cost: %s", result.cost)
        
        # Publish status information
        pub_log_info = rospy.Publisher('/nextion/log_info', String, queue_size=10, latch=True)
        pub_smach_status = rospy.Publisher('/mower_smach/status', String, latch=True, queue_size=1)
        
        pub_log_info.publish(String("{}: go to path {}/{}".format(
            userdata.zone_name, userdata.index_path+1, len(userdata.paths))))
        pub_smach_status.publish(String("{}: go to path {}/{}".format(
            userdata.zone_name, userdata.index_path+1, len(userdata.paths))))
        
        # Check for success and save path cost for distance check
        if result.outcome == 0:  # MBF Outcome: SUCCESS
            userdata.path_cost = result.cost
            return 'succeeded'
        else:
            return 'aborted'
    
    @staticmethod
    def exe_path_result_cb(userdata, status, result):
        """Handle result from exe_path action."""
        rospy.loginfo("Exe path status: %s", status)
        rospy.loginfo("Exe path result: %s", result.message)
        
        # Check for successful execution
        if result.outcome == 0:  # MBF Outcome: SUCCESS
            return 'succeeded'
        else:
            return 'aborted'
    
    @staticmethod
    def exe_path_planner_result_cb(userdata, status, result):
        """Handle result from exe_path action for planner path."""
        rospy.loginfo("Exe path status: %s", status)
        rospy.loginfo("Exe path result: %s", result.message)
        rospy.loginfo("Exe path outcome: %s", result.outcome)
        
        # Publish status information
        pub_log_info = rospy.Publisher('/nextion/log_info', String, queue_size=10, latch=True)
        pub_smach_status = rospy.Publisher('/mower_smach/status', String, latch=True, queue_size=1)
        
        pub_log_info.publish(String("{}: path {}/{} done".format(
            userdata.zone_name, userdata.index_path+1, len(userdata.paths))))
        pub_smach_status.publish(String("{}: path {}/{} done".format(
            userdata.zone_name, userdata.index_path+1, len(userdata.paths))))
        
        # Check for successful execution
        if result.outcome == 0:  # MBF Outcome: SUCCESS
            return 'succeeded'
        else:
            return 'aborted'
    
    @staticmethod
    def recovery_goal_cb(userdata, goal):
        """Prepare goal for recovery action."""
        pub_log_info = rospy.Publisher('/nextion/log_info', String, queue_size=10, latch=True)
        pub_log_info.publish(String("Recovery"))
        
        goal.behavior = 'clear_costmap'


class CheckPathState(smach.State):
    """
    Check if a path is valid and can be executed.
    
    This state uses the move_base_flex check_path service to validate a path.
    
    Inputs:
      - path: The path to check
    """
    def __init__(self):
        smach.State.__init__(self, 
                           outcomes=['succeeded', 'aborted', 'preempted'],
                           input_keys=['path', 'zone_name', 'index_path', 'paths'])
        
        # Publishers for status information
        self.pub_log_info = rospy.Publisher('/nextion/log_info', String, queue_size=10, latch=True)
        self.pub_smach_status = rospy.Publisher('/mower_smach/status', String, latch=True, queue_size=1)

    def execute(self, userdata):
        # Check for preemption
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
        
        # Publish status information
        self.pub_log_info.publish(String("{}: exe path {}/{}".format(
            userdata.zone_name, userdata.index_path+1, len(userdata.paths))))
        self.pub_smach_status.publish(String("{}: exe path {}/{}".format(
            userdata.zone_name, userdata.index_path+1, len(userdata.paths))))
        
        # Prepare service request
        try:
            rospy.wait_for_service('/move_base_flex/check_path_cost', timeout=5.0)
            check_path_srv = rospy.ServiceProxy('/move_base_flex/check_path_cost', CheckPath)
            
            request = CheckPathRequest()
            request.path = userdata.path
            request.safety_dist = 0.1
            request.lethal_cost_mult = 0
            request.inscrib_cost_mult = 0
            request.unknown_cost_mult = 0
            request.costmap = CheckPathRequest.GLOBAL_COSTMAP
            request.skip_poses = 0
            request.use_padded_fp = False
            request.path_cells_only = False
            
            # Call service
            response = check_path_srv(request)
            rospy.loginfo("Check path result: %s", response)
            
            return 'succeeded'
            
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return 'aborted'
        except rospy.ROSException as e:
            rospy.logerr("ROS exception: %s", e)
            return 'aborted'


def create_get_path_action_state(action_name, goal_slots, result_slots, 
                               goal_cb=None, result_cb=None, input_keys=None):
    """Factory function to create GetPath action states with consistent configuration."""
    if input_keys is None:
        input_keys = []
    
    return smach_ros.SimpleActionState(
        action_name,
        GetPathAction,
        goal_slots=goal_slots,
        result_slots=result_slots,
        goal_cb=goal_cb,
        result_cb=result_cb,
        input_keys=input_keys
    )


def create_exe_path_action_state(action_name, goal_slots, 
                               result_cb=None, input_keys=None):
    """Factory function to create ExePath action states with consistent configuration."""
    if input_keys is None:
        input_keys = []
    
    return smach_ros.SimpleActionState(
        action_name,
        ExePathAction,
        goal_slots=goal_slots,
        result_cb=result_cb,
        input_keys=input_keys
    )


def create_recovery_action_state(action_name, goal_cb):
    """Factory function to create Recovery action states with consistent configuration."""
    return smach_ros.SimpleActionState(
        action_name,
        RecoveryAction,
        goal_cb=goal_cb,
        input_keys=["error", "clear_costmap_flag"],
        output_keys=["error_status", 'clear_costmap_flag']
    )