#!/usr/bin/env python3

import rospy
import math
from std_msgs.msg import String
from nav_msgs.msg import Odometry, Path
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray
import uuid
import os
import sys, time
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from tf.transformations import quaternion_from_euler
from tf import TransformListener



class MapPoint:

    def __init__(self, name, pose, tf_frame):
        self.name = name
        self.pose = pose
        self.tf_frame = tf_frame
        self.marker_message = Marker()
        self.create_marker_msg()


    def create_marker_msg(self):
        # ARROW = 0
        # CUBE = 1
        # SPHERE = 2
        # CYLINDER = 3
        # LINE_STRIP = 4
        # LINE_LIST = 5
        # CUBE_LIST = 6
        # SPHERE_LIST = 7
        # POINTS = 8
        # TEXT_VIEW_FACING = 9
        # MESH_RESOURCE = 10
        # TRIANGLE_LIST = 11
        self.marker_message.header.frame_id = self.tf_frame
        self.marker_message.ns = self.name
        self.marker_message.type = self.marker_message.ARROW
        self.marker_message.action = self.marker_message.ADD
        self.marker_message.scale.x = 0.2
        self.marker_message.scale.y = 0.2
        self.marker_message.scale.z = 0.2
        self.marker_message.color.a = 1.0
        self.marker_message.color.r = 1.0
        self.marker_message.color.g = 1.0
        self.marker_message.color.b = 0.0
        self.marker_message.pose = self.pose









class MapPath:

    def __init__(self, name, tf_frame):
        self.name = name
        self.poses = []
        self.tf_frame = tf_frame
        self.message = Path()
        self.create_path_msg()


    def create_path_msg(self):
        self.message.header.frame_id = self.tf_frame
        self.message.poses = []
        for pose in self.poses:
            poseStamped = PoseStamped()
            poseStamped.header.frame_id = self.tf_frame
            poseStamped.header.stamp = rospy.Time.now()
            poseStamped.pose = pose
            self.message.poses.append(poseStamped)


    def add_pose(self, pose):
        self.poses.append(pose)
        self.create_path_msg()


class MapData:

    def __init__(self, name):
        self.map_name = name
        self.points = []
        self.paths = []
        self.utm_x = 0
        self.utm_y = 0
        self.utm_z = 0
        self.utm_orientation_z = 0
        self.utm_orientation_w = 0

    def is_utm(self):
        if (self.utm_x + self.utm_y + self.utm_z == 0):
            return False
        else:
            return True


    def add_map_point(self, point):
        self.points.append(point)

    def get_map_point(self, name):
        for point in self.points:
            if point.name == name:
                return point
        return False

    def get_map_point_list(self):
        result = ''
        for point in self.points:
            result += '{}|||'.format(point.name)
        return result

    def add_map_path(self, path):
        self.paths.append(path)

    def get_map_path(self, name):
        for path in self.paths:
            if path.name == name:
                return path
        return False

    def get_map_path_list(self):
        result = ''
        for path in self.paths:
            result += '{}|||'.format(path.name)
        return result

    def add_map_path_point(self, path_name, point):
        for path in self.paths:
            if path.name == path_name:
                path.add_pose(point)
