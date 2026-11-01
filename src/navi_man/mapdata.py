#!/usr/bin/env python3

import os
import sys

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import Marker

try:
    from vitulus_mapping import objnames
except Exception:  # pragma: no cover - devel PYTHONPATH fallback (as bundle_sync)
    sys.path.append(os.path.join(os.path.dirname(__file__),
                                 "..", "..", "..", "vitulus_mapping", "src"))
    from vitulus_mapping import objnames


class MapPoint:

    def __init__(self, name, pose, tf_frame):
        self.name = name
        self.pose = pose
        self.tf_frame = tf_frame
        self.marker_message = Marker()
        self.create_marker_msg()

    def create_marker_msg(self):
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
        # DOCK, UNDOCK, DOCKED answer in any case: a node not restarted yet still
        # asks for `docked`.
        name = objnames.canonical_name(name)
        for point in self.points:
            if point.name == name:
                return point
        return False

    def normalize_dock_names(self):
        """Dock points from an old pickle -> uppercase, one of each. True if
        something changed."""
        names = [point.name for point in self.points]
        kept = objnames.normalize_dock_names(names)
        if [name for _, name in kept] == names:
            return False
        points = []
        for i, name in kept:
            point = self.points[i]
            point.name = name
            point.marker_message.ns = name
            points.append(point)
        self.points = points
        return True

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
