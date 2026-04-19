#!/usr/bin/env python
"""
Monitor-related states for the mower SMACH state machine.

These states monitor various aspects of the system, such as
mower status, map availability, and GPS fix.
"""
import rospy
import smach
import smach_ros
from std_msgs.msg import String, Int16
from vitulus_msgs.msg import Mower, Device_icon_status


class MonitorCallbacks:
    """Helper class for monitor state callbacks."""
    
    @staticmethod
    def mower_status_ready(userdata, msg):
        """Monitor callback for when mower status is READY."""
        if msg.status == 'READY':
            rospy.loginfo("Mower is ready")
            
            # Publish status
            pub_log_info = rospy.Publisher('/nextion/log_info', String, queue_size=10, latch=True)
            pub_smach_status = rospy.Publisher('/mower_smach/status', String, latch=True, queue_size=1)
            pub_melody = rospy.Publisher('/pm/play_melody', Int16, queue_size=10)
            
            pub_log_info.publish(String("Mower is ready"))
            pub_smach_status.publish(String("Mower ready"))
            pub_melody.publish(Int16(1))  # 1: beep
            
            rospy.sleep(0.5)
            return False
        else:
            return True
    
    @staticmethod
    def mower_status_off(userdata, msg):
        """Monitor callback for when mower status is OFF."""
        if msg.status == 'OFF':
            # Save program duration if it exists
            if hasattr(userdata, 'prg_start_time') and userdata.prg_start_time is not None:
                save_prg_pub = rospy.Publisher("/web_plan/program_new", PlannerProgram, queue_size=1)
                duration = (rospy.Time.now() - userdata.prg_start_time).to_sec() / 60
                rospy.loginfo("Program duration: %f minutes", duration)
                userdata.program.last_duration_minutes = int(round(duration))
                save_prg_pub.publish(userdata.program)
                userdata.prg_start_time = None

            # Publish status
            pub_active_prg = rospy.Publisher('/mower_smach/active_program', String, latch=True, queue_size=1)
            pub_smach_status = rospy.Publisher('/mower_smach/status', String, latch=True, queue_size=1)
            pub_log_info = rospy.Publisher('/nextion/log_info', String, queue_size=10, latch=True)
            
            pub_active_prg.publish(String(" "))
            pub_smach_status.publish(String("Ready"))
            pub_log_info.publish(String("Done"))
            
            return False
        else:
            return True
    
    @staticmethod
    def mower_cut_height_ready(userdata, msg):
        """Monitor callback for when cut height has been set."""
        if msg.status == 'READY':
            return False
        else:
            return True
    
    @staticmethod
    def mower_rpm_ready(userdata, msg):
        """Monitor callback for when RPM has been set."""
        zone_rpm = userdata.zone_rpm
        if zone_rpm - 100 < msg.moto_rpm < zone_rpm + 100:
            # Publish status
            pub_log_info = rospy.Publisher('/nextion/log_info', String, queue_size=10, latch=True)
            pub_smach_status = rospy.Publisher('/mower_smach/status', String, latch=True, queue_size=1)
            
            pub_log_info.publish(String("Mowing started..."))
            pub_smach_status.publish(String("Mowing started"))
            
            return False
        else:
            return True
    
    @staticmethod
    def map_loaded(userdata, msg):
        """Monitor callback for when map is loaded."""
        rospy.loginfo("Map status: %s", msg)
        
        # Show map in UI
        show_map_planner_pub = rospy.Publisher("/web_plan/show_map_layer", String, latch=True, queue_size=1)
        show_map_planner_pub.publish(String("SMACH|MAP|FULL"))
        
        if msg.data == userdata.program.map_name:
            # Publish status
            pub_log_info = rospy.Publisher('/nextion/log_info', String, queue_size=10, latch=True)
            pub_smach_status = rospy.Publisher('/mower_smach/status', String, latch=True, queue_size=1)
            pub_melody = rospy.Publisher('/pm/play_melody', Int16, queue_size=10)
            
            pub_log_info.publish(String("Map is ready"))
            pub_smach_status.publish(String("Map ready"))
            pub_melody.publish(Int16(1))  # 1: beep
            
            rospy.sleep(4)
            return False
        else:
            return True
    
    @staticmethod
    def rtabmap_ready(userdata, msg):
        """Monitor callback for when rtabmap is ready."""
        if msg.proximityDetectionId > 0:
            # Publish status
            pub_log_info = rospy.Publisher('/nextion/log_info', String, queue_size=10, latch=True)
            pub_smach_status = rospy.Publisher('/mower_smach/status', String, latch=True, queue_size=1)
            pub_melody = rospy.Publisher('/pm/play_melody', Int16, queue_size=10)
            
            pub_log_info.publish(String("Rtabmap is ready"))
            pub_melody.publish(Int16(1))  # 1: beep
            pub_smach_status.publish(String("Rtabmap ready"))
            
            rospy.sleep(0.5)
            return False
        else:
            return True
    
    @staticmethod
    def gps_fix_acquired(userdata, msg):
        """Monitor callback for when GPS fix is acquired."""
        rospy.loginfo("GNSS status: %s", msg)
        
        if msg.gnss == 'RTK':
            # Publish status
            pub_log_info = rospy.Publisher('/nextion/log_info', String, queue_size=10, latch=True)
            pub_smach_status = rospy.Publisher('/mower_smach/status', String, latch=True, queue_size=1)
            
            pub_log_info.publish(String("RTK fix acquired"))
            pub_smach_status.publish(String("RTK fix acquired"))
            
            rospy.sleep(0.5)
            return False
        else:
            return True
    
    @staticmethod
    def program_received(userdata, msg):
        """Monitor callback for when a program is received."""
        userdata.program = msg
        rospy.loginfo('[%s] Get new program to run - state WAIT_FOR_PROGRAM', rospy.get_caller_id())
        
        # Publish status
        pub_active_program = rospy.Publisher('/mower_smach/active_program', String, latch=True, queue_size=1)
        pub_log_info = rospy.Publisher('/nextion/log_info', String, queue_size=10, latch=True)
        pub_melody = rospy.Publisher('/pm/play_melody', Int16, queue_size=10)
        pub_smach_status = rospy.Publisher('/mower_smach/status', String, latch=True, queue_size=1)
        
        pub_active_program.publish(String(msg.name))
        pub_log_info.publish(String("Executing program: {}".format(msg.name)))
        pub_melody.publish(Int16(2))  # 2: double beep
        pub_smach_status.publish(String("Initiating"))
        
        rospy.sleep(1.0)
        userdata.prg_start_time = rospy.Time.now()
        
        return False


def create_monitor_state(topic, msg_type, callback_fn, input_keys=None, output_keys=None):
    """Factory function to create monitor states with consistent configuration."""
    if input_keys is None:
        input_keys = []
    if output_keys is None:
        output_keys = []
    
    return smach_ros.MonitorState(
        topic, 
        msg_type, 
        callback_fn,
        input_keys=input_keys,
        output_keys=output_keys
    )