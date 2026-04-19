#!/usr/bin/env python
"""
Mower control states for the SMACH state machine.
"""
import rospy
import smach
from std_msgs.msg import Bool, String, Int16


class MowerBaseState(smach.State):
    """Base class for mower control states to reduce code duplication."""
    def __init__(self, outcomes, input_keys=None, output_keys=None):
        if input_keys is None:
            input_keys = []
        if output_keys is None:
            output_keys = []
        
        smach.State.__init__(self, outcomes=outcomes, input_keys=input_keys, output_keys=output_keys)
        
        # Common publishers
        self.pub_log_info = rospy.Publisher('/nextion/log_info', String, queue_size=10, latch=True)
        self.pub_smach_status = rospy.Publisher('/mower_smach/status', String, latch=True, queue_size=1)
        self.pub_melody = rospy.Publisher('/pm/play_melody', Int16, queue_size=10)


class PowerOnMower(MowerBaseState):
    """
    Turn on the mower's power.
    
    This state sends the command to power on the mower.
    """
    def __init__(self):
        super(PowerOnMower, self).__init__(outcomes=['success', 'failure'])
        
    def execute(self, userdata):
        rospy.loginfo('[%s] Turning on mower - state POWER_ON_MOWER', rospy.get_caller_id())
        
        # Create publisher and message
        pub_mower_set_power = rospy.Publisher('/mower/set_power', Bool, latch=True, queue_size=1)
        msg = Bool()
        msg.data = True
        
        # Publish message
        pub_status = pub_mower_set_power.publish(msg)
        
        # Log status
        self.pub_log_info.publish(String("Turn on mower"))
        self.pub_smach_status.publish(String("Turn on mower"))
        self.pub_melody.publish(Int16(1))  # 1: beep
        
        # Wait for power to stabilize
        rospy.sleep(2.5)
        
        return 'success'


class PowerOffMower(MowerBaseState):
    """
    Turn off the mower's power.
    
    This state sends the command to power off the mower.
    """
    def __init__(self):
        super(PowerOffMower, self).__init__(outcomes=['success', 'failure'])
        
    def execute(self, userdata):
        rospy.loginfo('[%s] Setting mower off - state POWER_OFF_MOWER', rospy.get_caller_id())
        
        # Create publisher and message
        pub_mower_off = rospy.Publisher('/mower/set_power', Bool, latch=True, queue_size=1)
        msg = Bool()
        msg.data = False
        
        # Publish message
        pub_status = pub_mower_off.publish(msg)
        
        # Wait for power to stabilize
        rospy.sleep(1.5)
        
        return 'success'


class SetCutHeight(MowerBaseState):
    """
    Set the cutting height of the mower.
    
    This state sends the command to set the cutting height.
    
    Inputs:
      - zone_cut_height: The desired cutting height
    """
    def __init__(self):
        super(SetCutHeight, self).__init__(
            outcomes=['success', 'failure'],
            input_keys=['zone_cut_height']
        )
        
    def execute(self, userdata):
        rospy.loginfo('[%s] Setting cut height on mower - state SET_CUT_HEIGHT', rospy.get_caller_id())
        
        # Create publisher and message
        pub_mower_set_cut_height = rospy.Publisher('/mower/set_cut_height', Int16, latch=True, queue_size=1)
        msg = Int16()
        msg.data = userdata.zone_cut_height
        
        # Publish message
        pub_status = pub_mower_set_cut_height.publish(msg)
        
        # Log status
        self.pub_log_info.publish(String("Setting mower height"))
        self.pub_smach_status.publish(String("Setting height"))
        self.pub_melody.publish(Int16(1))  # 1: beep
        
        # Wait for height adjustment to complete
        rospy.sleep(1.5)
        
        return 'success'


class SetMotorRPM(MowerBaseState):
    """
    Set the RPM of the mower's motor.
    
    This state sends the command to set the motor RPM.
    
    Inputs:
      - zone_rpm: The desired motor RPM
    """
    def __init__(self):
        super(SetMotorRPM, self).__init__(
            outcomes=['success', 'failure'],
            input_keys=['zone_rpm']
        )
        
    def execute(self, userdata):
        rospy.loginfo('[%s] Setting motor rpm on mower - state SET_RPM', rospy.get_caller_id())
        
        # Create publisher and message
        pub_mower_set_motor_rpm = rospy.Publisher('/mower/set_motor_rpm', Int16, latch=True, queue_size=1)
        msg = Int16()
        msg.data = userdata.zone_rpm
        
        # Publish message
        pub_status = pub_mower_set_motor_rpm.publish(msg)
        
        # Log status
        self.pub_smach_status.publish(String("Set rpm"))
        
        return 'success'


class SetMotorOn(MowerBaseState):
    """
    Turn on the mower's motor.
    
    This state sends the command to turn on the mower's motor.
    """
    def __init__(self):
        super(SetMotorOn, self).__init__(outcomes=['success', 'failure'])
        
    def execute(self, userdata):
        rospy.loginfo('[%s] Setting mower motor on - state SET_MOTOR_ON', rospy.get_caller_id())
        
        # Create publisher and message
        pub_mower_set_motor_on = rospy.Publisher('/mower/set_motor_on', Bool, latch=True, queue_size=1)
        msg = Bool()
        msg.data = True
        
        # Publish message
        pub_status = pub_mower_set_motor_on.publish(msg)
        
        return 'success'


class SetMotorOffAndHome(MowerBaseState):
    """
    Turn off the mower motor and set home position.
    
    This state sends the commands to set home position and turn off the mower motor.
    """
    def __init__(self):
        super(SetMotorOffAndHome, self).__init__(outcomes=['success', 'failure'])
        
    def execute(self, userdata):
        rospy.loginfo('[%s] Setting mower motor off and home - state SET_MOTOR_OFF_AND_HOME', 
                     rospy.get_caller_id())
        
        # Show full map view
        show_map_planner_pub = rospy.Publisher("/web_plan/show_map_layer", String, latch=True, queue_size=1)
        show_map_planner_pub.publish(String("SMACH|MAP|FULL"))
        
        # Set home position
        pub_mower_set_home = rospy.Publisher('/mower/set_home', Bool, latch=True, queue_size=1)
        msg = Bool()
        msg.data = True
        pub_mower_set_home.publish(msg)
        rospy.sleep(1.5)
        
        # Turn off motor
        pub_mower_set_motor_off = rospy.Publisher('/mower/set_motor_on', Bool, latch=True, queue_size=1)
        msg = Bool()
        msg.data = False
        pub_status = pub_mower_set_motor_off.publish(msg)
        rospy.sleep(1.5)
        
        return 'success'