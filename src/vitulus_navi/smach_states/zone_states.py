#!/usr/bin/env python
"""
Zone-related states for the mower SMACH state machine.
"""
import rospy
import smach
from std_msgs.msg import String


class GetZoneData(smach.State):
    """
    Extract zone-specific data from the program.
    
    This state prepares zone-specific settings (cut height, RPM, name, etc.)
    for the mower to operate on a particular zone.
    
    Inputs:
      - program: The full mowing program
      - index: Current zone index
      
    Outputs:
      - zone_cut_height: Height setting for the mower
      - zone_rpm: Speed setting for the mower blades
      - zone_name: Name of the zone
      - zone_start_pose: Starting position for the zone
      - paths: All paths within this zone
      - controller: The controller to use for this zone
    """
    def __init__(self):
        smach.State.__init__(self, 
                            outcomes=['available', 'failed', 'preempted'],
                            input_keys=['program', 'zone_cut_height', 'zone_rpm', 'index', 
                                       'zone_name', 'zone_start_pose', 'paths'],
                            output_keys=['zone_cut_height', 'zone_rpm', 'zone_name', 
                                        'zone_start_pose', 'paths', 'controller'])

    def execute(self, userdata):
        rospy.loginfo('[%s] Getting zone data - state GET_ZONE_DATA', rospy.get_caller_id())
        
        # Extract zone data from the program
        userdata.zone_cut_height = userdata.program.zone_list[userdata.index].cut_height
        userdata.zone_rpm = userdata.program.zone_list[userdata.index].rpm
        userdata.zone_name = userdata.program.zone_list[userdata.index].name
        userdata.zone_start_pose = userdata.program.zone_list[userdata.index].paths[0].poses[0]
        userdata.paths = userdata.program.zone_list[userdata.index].paths
        userdata.controller = 'base_local_planner/TrajectoryPlannerROS'
        
        rospy.loginfo('Zone: %s', userdata.zone_name)
        
        # Display the map
        show_map_planner_pub = rospy.Publisher("/web_plan/show_map_layer", String, latch=True, queue_size=1)
        show_map_planner_pub.publish(String("SMACH|MAP|FULL"))
        rospy.sleep(3)

        # Check for preemption
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
            
        return 'available'