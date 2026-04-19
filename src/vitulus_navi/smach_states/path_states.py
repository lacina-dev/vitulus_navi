#!/usr/bin/env python
"""
Path-related states for the mower SMACH state machine.
"""
import rospy
import smach
import math
import shapely
from shapely import geometry
from std_msgs.msg import String
from geometry_msgs.msg import Quaternion, PoseStamped
from nav_msgs.msg import Path
import PyKDL as kdl


class GetPathData(smach.State):
    """
    Extract path-specific data and prepare it for execution.
    
    This state prepares path data for navigation, including segmenting
    the path into smaller points for smoother navigation.
    
    Inputs:
      - paths: All paths in the current zone
      - index_path: Current path index
      
    Outputs:
      - path: Processed path for navigation
      - path_start_pose: Starting point of the path
      - final_pose: End point of the path
    """
    def __init__(self):
        smach.State.__init__(self, 
                           outcomes=['available', 'preempted'],
                           input_keys=['program', 'zone_cut_height', 'zone_rpm', 'index', 'zone_name', 
                                      'path_planner', 'zone_start_pose', 'path_plan', 'index_path', 
                                      'paths', 'path_start_pose', 'path', 'final_pose'],
                           output_keys=['zone_cut_height', 'zone_rpm', 'zone_name', 'zone_start_pose', 
                                       'path_planner', 'path_plan', 'paths', 'path_start_pose', 
                                       'path', 'final_pose'])

    def path_to_polygon(self, path):
        """Create shapely polygon from path poses."""
        polygon = geometry.LinearRing([[pose.pose.position.x, pose.pose.position.y] for pose in path.poses])
        polygon = shapely.remove_repeated_points(polygon)
        return polygon

    def path_to_multiline(self, path):
        """Create shapely multiline string from path poses."""
        multiline = geometry.MultiLineString(
            [[[pose.pose.position.x, pose.pose.position.y] for pose in path.poses]])
        return multiline

    def path_to_line(self, path):
        """Create shapely line string from path poses."""
        line = geometry.LineString(
            [[pose.pose.position.x, pose.pose.position.y] for pose in path.poses])
        return line

    def direction(self, line):
        """Calculate direction angle from line segment."""
        x = line[1][0] - line[0][0]
        y = line[1][1] - line[0][1]
        diagonal = math.sqrt(x ** 2 + y ** 2)
        
        # Calculate angle depending on quadrant
        if y < 0:
            if x >= 0:
                angle = math.asin(y / diagonal)
            else:
                angle = math.acos(x / diagonal) * -1
        else:
            if x >= 0:
                angle = math.asin(y / diagonal)
            else:
                angle = math.acos(x / diagonal)
                
        return angle

    def execute(self, userdata):
        rospy.loginfo('[%s] Getting path data - state GET_PATH_DATA', rospy.get_caller_id())
        
        # Get the current path
        path = userdata.paths[userdata.index_path]
        userdata.path_planner = path
        userdata.path_start_pose = path.poses[0]
        
        # Show the zone on the map
        show_map_planner_pub = rospy.Publisher("/web_plan/show_map_layer", String, latch=True, queue_size=1)
        show_map_planner_pub.publish(String("SMACH|ZONE|{}".format(userdata.zone_name)))
        
        # Create line from start to end points
        line = geometry.LineString([
            [path.poses[0].pose.position.x, path.poses[0].pose.position.y],
            [path.poses[-1].pose.position.x, path.poses[-1].pose.position.y]
        ])

        # Process path based on its type
        segmentize_distance = 0.03
        if line.length == 0:
            # Outline path (closed loop)
            rospy.loginfo("Processing outline path")
            start_offset = 0.18
            start_poses_offset = int(round(start_offset / segmentize_distance))
            
            # Create new path
            path_new = Path()
            path_new.header = path.header
            
            # Process the polygon outline
            polygon = self.path_to_polygon(path)
            for n in range(0, len(polygon.coords) - 1):
                segment = geometry.LineString([
                    [polygon.coords[n][0], polygon.coords[n][1]],
                    [polygon.coords[n + 1][0], polygon.coords[n + 1][1]]
                ])
                angle = self.direction(segment.coords)
                segment = segment.segmentize(segmentize_distance)
                
                # Create poses from segment points
                for point in segment.coords:
                    pose = PoseStamped()
                    pose.header.frame_id = "map"
                    pose.header.stamp = rospy.Time.now()
                    pose.pose.position.x = point[0]
                    pose.pose.position.y = point[1]
                    pose.pose.position.z = 0.0
                    pose.pose.orientation = Quaternion(
                        *(kdl.Rotation.RPY(0, 0, angle).GetQuaternion())
                    )
                    path_new.poses.append(pose)
                    
            # Apply offset to start/end poses
            path_new.poses = path_new.poses[start_poses_offset:-start_poses_offset]
            
            # Update path and poses
            path = path_new
            userdata.path_start_pose = path.poses[0]
            userdata.final_pose = path.poses[-1]
            
        else:
            # Coverage path (open path)
            rospy.loginfo("Processing coverage path")
            line = self.path_to_line(path)
            
            # Create new path
            path_new = Path()
            path_new.header.frame_id = "map"
            path_new.header.stamp = rospy.Time.now()

            # Segment the line and create poses
            line = line.segmentize(segmentize_distance)
            for pose_id in range(0, len(line.coords) - 1):
                angle = self.direction([
                    [line.coords[pose_id][0], line.coords[pose_id][1]],
                    [line.coords[pose_id + 1][0], line.coords[pose_id + 1][1]]
                ])
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.header.stamp = rospy.Time.now()
                pose.pose.position.x = line.coords[pose_id][0]
                pose.pose.position.y = line.coords[pose_id][1]
                pose.pose.position.z = 0.0
                pose.pose.orientation = Quaternion(
                    *(kdl.Rotation.RPY(0, 0, angle).GetQuaternion())
                )
                path_new.poses.append(pose)
                
            # Update path
            path = path_new

        # Publish current path for visualization
        pub_current_path = rospy.Publisher('/test_path', Path, latch=True, queue_size=1)
        pub_current_path.publish(path)
        userdata.path = path

        # Check for preemption
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
            
        return 'available'


class CheckDistance(smach.State):
    """
    Check if the robot is already at the target position or needs to navigate there.
    
    This state determines whether the robot needs to move to reach
    the starting point of a path or is already close enough.
    
    Inputs:
      - path_cost: The cost/distance to reach the target position
      
    Outcomes:
      - on_place: The robot is already at the target position (distance <= 0.2)
      - exe_path: The robot needs to move to reach the target position (distance > 0.2)
    """
    def __init__(self):
        smach.State.__init__(self, 
                           outcomes=['on_place', 'exe_path', 'preempted'],
                           input_keys=['path_cost'],
                           output_keys=['path_cost'])

    def execute(self, userdata):
        rospy.loginfo('[%s] Checking path distance - state CHECK_DISTANCE', rospy.get_caller_id())
        
        # Check for preemption
        if self.preempt_requested():
            self.service_preempt()
            return 'preempted'
            
        # Check if robot is close enough to target position
        if userdata.path_cost <= 0.2:
            return 'on_place'
        else:
            return 'exe_path'