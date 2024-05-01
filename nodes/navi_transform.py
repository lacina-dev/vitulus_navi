#!/usr/bin/env python
import copy
import time

import rospy
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseStamped, Pose, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from robot_localization.srv import SetPose, SetPoseRequest, SetPoseResponse
from sensor_msgs.msg import MagneticField, Imu
from sensor_msgs_ext.msg import magnetometer, accelerometer
from std_srvs.srv import Empty, EmptyResponse
from tf import TransformListener
from vitulus_msgs.msg import Imu_status
from std_msgs.msg import Bool, Float32MultiArray
import sys
import serial
import signal
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from sensor_msgs.msg import NavSatFix
from tf.transformations import euler_from_quaternion, quaternion_from_euler
import numpy as np
import math
from visualization_msgs.msg import Marker, MarkerArray
from collections import deque


def cart2pol(x, y):
    rho = np.sqrt(x ** 2 + y ** 2)
    phi = np.arctan2(y, x)
    return rho, phi


def pol2cart(rho, phi):
    x = rho * np.cos(phi)
    y = rho * np.sin(phi)
    return x, y


class NavTfNode:

    def __init__(self):
        self.base_odom_msg = Odometry()
        self.base_odom_msg.pose.pose.orientation.w = 1
        self.navheading_msg = Imu()
        self.navheading_msg.orientation.w = 1
        self.imu_msg = Imu()
        self.imu_msg.orientation.w = 1
        self.fused_msg = Imu()
        self.fused_msg.orientation.w = 1
        self.fix_usable = False
        self.fix_usable_history = [0, 0, 0]
        self.heading_fix_usable = False
        self.nav_last_fused_time = 0
        self.fix_last_fused_time = 0
        self.imu_delay = 0
        self.rtk_err = 0
        self.imu_yaw = 0
        self.nav_wheel_odom_offset_theta = 0
        self.navheading_yaw = 0
        self.fused_yaw = 0
        self.diff_yaw = 0
        self.set_pose_yaw = 0
        self.set_pose_pose = 0
        self.is_indoor = True
        self.marker_imu = Marker()
        self.marker_nav = Marker()
        self.marker_fuse = Marker()
        self.imu_queue_size = 10
        self.wheel_odom_msg = Odometry()
        self.rtk_odom_msg = Odometry()
        self.rtk_odom_msg.pose.pose.orientation.w = 1
        self.rtk_odom_heading_msg = Odometry()
        self.rtk_odom_heading_msg.pose.pose.orientation.w = 1
        self.filtered_wheel_odom_msg = Odometry()
        self.filtered_wheel_odom_msg.pose.pose.orientation.w = 1
        self.offset_wheel_odom_msg = Odometry()
        self.offset_wheel_odom_msg.pose.pose.orientation.w = 1
        self.odom_for_gps_msg = Odometry()
        self.odom_for_gps_msg.pose.pose.orientation.w = 1
        self.odom_fused_msg = Odometry()
        self.odom_fused_msg.pose.pose.orientation.w = 1
        self.navheading_wheel_odom_msg = Odometry()
        self.navheading_wheel_odom_msg.pose.pose.orientation.w = 1
        self.offset_navheading_wheel_odom_msg = Odometry()
        self.offset_navheading_wheel_odom_msg.pose.pose.orientation.w = 1
        self.offset_rtk_odom_msg = Odometry()
        self.offset_rtk_odom_msg.pose.pose.orientation.w = 1
        self.ekf_wheel_odom_msg = Odometry()
        self.ekf_wheel_odom_msg.pose.pose.orientation.w = 1
        self.ekf_wheel_nav_odom_msg = Odometry()
        self.ekf_wheel_nav_odom_msg.pose.pose.orientation.w = 1
        self.map_coords_msg = PoseStamped()
        self.map_odom_transform = TransformStamped()
        self.map_odom_transform.transform.rotation.w = 1
        self.fused_nav_time = int(str(rospy.Time.now())) / 1000000000
        self.fused_fix_time = int(str(rospy.Time.now())) / 1000000000
        self.init = True

        self.odometry_odom_pub = rospy.Publisher("/odometry/odom", Odometry, queue_size=100)
        self.odometry_for_gps_pub = rospy.Publisher("/odometry/for_gps", Odometry, queue_size=100)
        self.fused_nav_pub = rospy.Publisher("/gnss_heading/nav_fused", Imu, queue_size=100)
        self.base_link_imu_pub = rospy.Publisher("/bno085/imu_base_link", Imu, queue_size=100)
        self.marker_pub = rospy.Publisher("/gnss_heading/markers", MarkerArray, queue_size=1)
        self.yaws_pub = rospy.Publisher("/gnss_heading/yaws", Float32MultiArray, queue_size=1)
        self.wheel_odom_pub = rospy.Publisher("/nav_tf/odom_wheel", Odometry, queue_size=100)
        self.fused_odom_pub = rospy.Publisher("/nav_tf/odom_fused", Odometry, queue_size=100)
        self.nav_wheel_odom_pub = rospy.Publisher("/nav_tf/odom_wheel_nav", Odometry, queue_size=100)

        self.navheading_gnss_fix_sub = rospy.Subscriber("/gnss/fix", NavSatFix, self.callback_gnss_fix, queue_size=10)
        self.navheading_gnss_heading_fix_sub = rospy.Subscriber("/gnss_heading/fix", NavSatFix, self.callback_gnss_heading_fix, queue_size=10)
        self.navheading_rtk_sub = rospy.Subscriber("/gnss_heading/navheading", Imu, self.callback_navheading, queue_size=10)
        self.imu_sub = rospy.Subscriber("/bno085/imu", Imu, self.callback_imu, queue_size=200)
        self.wheel_odom_sub = rospy.Subscriber("/mobile_base_controller/odom", Odometry, self.callback_wheel_odom, queue_size=200)
        self.rtk_odom_sub = rospy.Subscriber("/odometry/gps", Odometry, self.callback_rtk_odom, queue_size=200)
        self.base_odom_sub = rospy.Subscriber("/odometry/odom", Odometry, self.callback_base_odom, queue_size=200)
        self.ekf_wheel_odom_sub = rospy.Subscriber("/odometry/ekf_wheel_odom", Odometry, self.callback_ekf_wheel_odom, queue_size=200)
        self.ekf_wheel_nav_odom_sub = rospy.Subscriber("/odometry/ekf_wheel_nav_odom", Odometry, self.callback_ekf_wheel_nav_odom, queue_size=200)
        self.map_coords_sub = rospy.Subscriber("/navi_manager/map_coords", PoseStamped, self.callback_map_coords)
        self.navi_indoor_sub = rospy.Subscriber("/navi_manager/is_indoor", Bool, self.callback_navi_indoor)
        self.set_pose_sub = rospy.Subscriber("/nav_tf/set_pose", PoseWithCovarianceStamped, self.callback_set_pose)

        self.tf2_Buffer = tf2_ros.Buffer()
        self.tf2_TransformListener = tf2_ros.TransformListener(self.tf2_Buffer)
        self.tf = TransformListener()
        self.tf2_StaticTransformBroadcaster = tf2_ros.StaticTransformBroadcaster()
        self.tf2_TransformBroadcaster = tf2_ros.TransformBroadcaster()

        self.reset_wheel_odom_srv = rospy.Service("nav_tf/reset_wheel_odom", Empty, self.srv_cb_reset_wheel_odom)
        self.set_nav_wheel_odom_srv = rospy.Service("nav_tf/set_nav_wheel_odom", Empty, self.srv_cb_set_nav_wheel_odom)
        self.set_pose_srv = rospy.Service("nav_tf/set_pose", SetPose, self.srv_cb_set_pose)

        rospy.loginfo("<{}> GNSSNode initiated.".format(rospy.get_caller_id()))

    def callback_set_pose(self, msg):
        print("Set pose")
        print(msg.pose.pose)
        self.set_nav_pose(msg.pose.pose)
        self.set_wheel_base_nav_pose(msg.pose.pose)


    def callback_navi_indoor(self, msg):
        self.is_indoor = msg.data

    def callback_map_coords(self, msg):
        if msg.pose != self.map_coords_msg.pose:
            print("Map changed")
            print(msg.pose)
            trans = None
            try:
                trans = self.tf2_Buffer.lookup_transform('utm', 'odom', rospy.Time(0))
            except:
                trans = None

            if trans is not None:
                print(trans)

                map_roll, map_pitch, map_yaw = euler_from_quaternion(
                    [msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w])
                odom_roll, odom_pitch, odom_yaw = euler_from_quaternion([trans.transform.rotation.x,
                                                                         trans.transform.rotation.y,
                                                                         trans.transform.rotation.z,
                                                                         trans.transform.rotation.w])
                yaw_offset = odom_yaw - map_yaw
                print("Map  Yaw offset: ", yaw_offset)
                quaternion = quaternion_from_euler(map_roll, map_pitch, yaw_offset)

                p_x = 0
                p_y = 0
                p_z = 0

                if (msg.pose.position.x + msg.pose.position.y + msg.pose.position.z) != 0:
                    p_x = trans.transform.translation.x - msg.pose.position.x
                    p_y = trans.transform.translation.y - msg.pose.position.y
                    p_z = trans.transform.translation.z - msg.pose.position.z

                map_odom_transform = TransformStamped()
                map_odom_transform.header.stamp = rospy.Time.now()
                map_odom_transform.header.frame_id = "map"
                map_odom_transform.child_frame_id = "odom"
                map_odom_transform.transform.translation.x = p_x
                map_odom_transform.transform.translation.y = p_y
                map_odom_transform.transform.translation.z = p_z
                map_odom_transform.transform.rotation.x = quaternion[0]
                map_odom_transform.transform.rotation.y = quaternion[1]
                map_odom_transform.transform.rotation.z = quaternion[2]
                map_odom_transform.transform.rotation.w = quaternion[3]
                print(map_odom_transform)

                self.map_odom_transform = map_odom_transform
        self.map_coords_msg = msg

    def callback_base_odom(self, msg):
        self.base_odom_msg = msg

    def callback_ekf_wheel_odom(self, msg):
        self.ekf_wheel_odom_msg = msg

    def callback_ekf_wheel_nav_odom(self, msg):
        self.ekf_wheel_nav_odom_msg = msg


    def callback_wheel_odom(self, msg):
        self.wheel_odom_msg = msg

    def callback_rtk_odom(self, msg):
        self.rtk_odom_msg = msg
        self.rtk_odom_heading_msg = copy.deepcopy(msg)
        self.rtk_odom_heading_msg.pose.pose.orientation = self.fused_msg.orientation



    def callback_gnss_fix(self, msg):
        usable = False
        if msg.position_covariance[8] <= 0.0001 \
                and msg.position_covariance[0] <= 0.000197 \
                and msg.position_covariance[4] <= 0.000197:
            if msg.status.status == 2:
                usable = True
                # self.set_nav_wheel_odom()
                # self.reset_wheel_odom()
                # self.set_wheel_base_pose()
        self.fix_usable = usable
        self.fix_usable_history.pop(0)
        self.fix_usable_history.append(usable)
        # print(self.fix_usable_history)
        # print("gnss_fix usable: ", usable)



    def callback_gnss_heading_fix(self, msg):
        usable = False
        if msg.position_covariance[8] <= 0.00065 \
                and msg.position_covariance[0] <= 0.000256 \
                and msg.position_covariance[4] <= 0.000256:
            if msg.status.status == 2:
                usable = True
        self.heading_fix_usable = usable
        # print("gnss_heading_fix usable: ", usable)

    def callback_navheading(self, msg):
        self.offset_navheading_imu(msg)
        self.navheading_msg = msg
        (roll, pitch, yaw) = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.navheading_yaw = yaw


    def callback_imu(self, msg):
        self.imu_msg = msg
        (roll, pitch, yaw) = euler_from_quaternion(
            [msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.imu_yaw = yaw
        self.publish_imu_baselink()
        self.publish_fused_navheading()

    def transform_odom_to_base_link(self):
        if not self.is_indoor:
            # msg = self.base_odom_msg
            # msg = self.filtered_wheel_odom_msg
            # msg = self.wheel_odom_msg
            # msg = self.rtk_odom_msg
            # msg = self.rtk_odom_heading_msg
            # msg = self.navheading_wheel_odom_msg
            # msg = self.ekf_wheel_odom_msg
            msg = self.ekf_wheel_nav_odom_msg
            # msg = self.odom_fused_msg
            static_transformStamped = TransformStamped()
            static_transformStamped.header.stamp = rospy.Time.now()
            static_transformStamped.header.frame_id = "odom"
            static_transformStamped.child_frame_id = "base_link"
            static_transformStamped.transform.translation.x = msg.pose.pose.position.x
            static_transformStamped.transform.translation.y = msg.pose.pose.position.y
            static_transformStamped.transform.translation.z = msg.pose.pose.position.z
            static_transformStamped.transform.rotation.x = msg.pose.pose.orientation.x
            static_transformStamped.transform.rotation.y = msg.pose.pose.orientation.y
            static_transformStamped.transform.rotation.z = msg.pose.pose.orientation.z
            static_transformStamped.transform.rotation.w = msg.pose.pose.orientation.w
            self.tf2_TransformBroadcaster.sendTransform(static_transformStamped)
            # print(msg.pose.pose)
            msg.header.frame_id = "odom"
            msg.child_frame_id = "base_link"
            self.odometry_odom_pub.publish(msg)

    def get_base_link_transform(self):
        trans = None
        try:
            trans = self.tf2_Buffer.lookup_transform('odom', 'base_link', rospy.Time(0))
        except:
            trans = None

        if trans is not None:
            odom_base_transform = TransformStamped()
            odom_base_transform.header.stamp = rospy.Time.now()
            odom_base_transform.header.frame_id = "odom"
            odom_base_transform.child_frame_id = "base_link"
            odom_base_transform.transform.translation.x = trans.transform.translation.x
            odom_base_transform.transform.translation.y = trans.transform.translation.y
            odom_base_transform.transform.translation.z = trans.transform.translation.z
            odom_base_transform.transform.rotation.x = trans.transform.rotation.x
            odom_base_transform.transform.rotation.y = trans.transform.rotation.y
            odom_base_transform.transform.rotation.z = trans.transform.rotation.z
            odom_base_transform.transform.rotation.w = trans.transform.rotation.w
            return odom_base_transform
        else:
            return None

    def transform_map_to_odom(self):
        if not self.is_indoor:
            odom_base_tf = self.get_base_link_transform()
            map_odom_tf = TransformStamped()
            map_odom_tf.header.stamp = rospy.Time.now()
            map_odom_tf.header.frame_id = "map"
            map_odom_tf.child_frame_id = "odom"
            map_odom_tf.transform.translation = self.map_odom_transform.transform.translation
            if odom_base_tf is not None:
                map_odom_tf.transform.translation.z = (odom_base_tf.transform.translation.z * -1)
            map_odom_tf.transform.rotation = self.map_odom_transform.transform.rotation

            self.tf2_TransformBroadcaster.sendTransform(map_odom_tf)


    def publish_odom_for_gps(self):
        self.odom_for_gps_msg = self.odom_for_gps_msg
        self.odom_for_gps_msg.header.stamp = rospy.Time.now()
        self.odom_for_gps_msg.pose.pose.orientation = self.fused_msg.orientation
        self.odom_for_gps_msg.header.frame_id = "odom"
        self.odom_for_gps_msg.child_frame_id = "base_link"
        self.odometry_for_gps_pub.publish(self.odom_for_gps_msg)

    def publish_odom_fused(self):
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        # msg.pose.pose.position.x = self.rtk_odom_heading_msg.pose.pose.position.x - self.filtered_wheel_odom_msg.pose.pose.position.x
        # msg.pose.pose.position.y = self.rtk_odom_heading_msg.pose.pose.position.y - self.filtered_wheel_odom_msg.pose.pose.position.y
        # msg.pose.pose.position.z = self.rtk_odom_heading_msg.pose.pose.position.z - self.filtered_wheel_odom_msg.pose.pose.position.z
        msg.pose.pose.position.x = self.rtk_odom_heading_msg.pose.pose.position.x
        msg.pose.pose.position.y = self.rtk_odom_heading_msg.pose.pose.position.y
        msg.pose.pose.position.z = self.rtk_odom_heading_msg.pose.pose.position.z
        # msg.pose.pose.orientation.x = self.rtk_odom_heading_msg.pose.pose.orientation.x - self.filtered_wheel_odom_msg.pose.pose.orientation.x
        # msg.pose.pose.orientation.y = self.rtk_odom_heading_msg.pose.pose.orientation.y - self.filtered_wheel_odom_msg.pose.pose.orientation.y
        # msg.pose.pose.orientation.z = self.rtk_odom_heading_msg.pose.pose.orientation.z - self.filtered_wheel_odom_msg.pose.pose.orientation.z
        # msg.pose.pose.orientation.w = self.rtk_odom_heading_msg.pose.pose.orientation.w - self.filtered_wheel_odom_msg.pose.pose.orientation.w
        # msg.pose.pose.orientation.x = self.rtk_odom_heading_msg.pose.pose.orientation.x
        # msg.pose.pose.orientation.y = self.rtk_odom_heading_msg.pose.pose.orientation.y
        # msg.pose.pose.orientation.z = self.rtk_odom_heading_msg.pose.pose.orientation.z
        # msg.pose.pose.orientation.w = self.rtk_odom_heading_msg.pose.pose.orientation.w
        msg.pose.pose.orientation.x = self.rtk_odom_heading_msg.pose.pose.orientation.x
        msg.pose.pose.orientation.y = self.rtk_odom_heading_msg.pose.pose.orientation.y
        msg.pose.pose.orientation.z = self.rtk_odom_heading_msg.pose.pose.orientation.z
        msg.pose.pose.orientation.w = self.rtk_odom_heading_msg.pose.pose.orientation.w

        self.odom_fused_msg = msg
        self.fused_odom_pub.publish(self.odom_fused_msg)

    def publish_relative_wheel_odom(self):
        """
        Publish the relative wheel odom. The wheel odom with the offset.
        :return:
        """
        self.filtered_wheel_odom_msg = Odometry()
        self.filtered_wheel_odom_msg.header.stamp = self.wheel_odom_msg.header.stamp
        self.filtered_wheel_odom_msg.header.frame_id = self.wheel_odom_msg.header.frame_id

        wheel_odom_roll, wheel_odom_pitch, wheel_odom_yaw = euler_from_quaternion(
            [self.wheel_odom_msg.pose.pose.orientation.x,
             self.wheel_odom_msg.pose.pose.orientation.y,
             self.wheel_odom_msg.pose.pose.orientation.z,
             self.wheel_odom_msg.pose.pose.orientation.w])
        offset_wheel_odom_roll, offset_wheel_odom_pitch, offset_wheel_odom_yaw = euler_from_quaternion(
            [self.offset_wheel_odom_msg.pose.pose.orientation.x,
             self.offset_wheel_odom_msg.pose.pose.orientation.y,
             self.offset_wheel_odom_msg.pose.pose.orientation.z,
             self.offset_wheel_odom_msg.pose.pose.orientation.w])

        n_off_distance, n_off_theta = cart2pol(
                        self.wheel_odom_msg.pose.pose.position.x - self.offset_wheel_odom_msg.pose.pose.position.x,
                        self.wheel_odom_msg.pose.pose.position.y - self.offset_wheel_odom_msg.pose.pose.position.y)

        n_x, n_y = pol2cart(n_off_distance,  n_off_theta - offset_wheel_odom_yaw)


        self.filtered_wheel_odom_msg.pose.pose.position.x = n_x
        self.filtered_wheel_odom_msg.pose.pose.position.y = n_y
        self.filtered_wheel_odom_msg.pose.pose.position.z = 0


        wheel_odom_yaw = wheel_odom_yaw - offset_wheel_odom_yaw
        quaternion = quaternion_from_euler(wheel_odom_roll, wheel_odom_pitch, wheel_odom_yaw)
        self.filtered_wheel_odom_msg.pose.pose.orientation.x = quaternion[0]
        self.filtered_wheel_odom_msg.pose.pose.orientation.y = quaternion[1]
        self.filtered_wheel_odom_msg.pose.pose.orientation.z = quaternion[2]
        self.filtered_wheel_odom_msg.pose.pose.orientation.w = quaternion[3]

        self.filtered_wheel_odom_msg.pose.covariance = self.wheel_odom_msg.pose.covariance
        self.filtered_wheel_odom_msg.twist.twist = self.wheel_odom_msg.twist.twist
        self.filtered_wheel_odom_msg.twist.covariance = self.wheel_odom_msg.twist.covariance
        self.wheel_odom_pub.publish(self.filtered_wheel_odom_msg)

    def publish_navheading_wheel_odom(self):
        """
        Publish the navheading wheel odom. The realtive wheel odom with the gnss navheading.
        :return:
        """
        self.navheading_wheel_odom_msg = Odometry()
        self.navheading_wheel_odom_msg.header.stamp = self.ekf_wheel_odom_msg.header.stamp
        self.navheading_wheel_odom_msg.header.frame_id = self.ekf_wheel_odom_msg.header.frame_id

        wheel_odom_roll, wheel_odom_pitch, wheel_odom_yaw = euler_from_quaternion(
            [self.ekf_wheel_odom_msg.pose.pose.orientation.x,
             self.ekf_wheel_odom_msg.pose.pose.orientation.y,
             self.ekf_wheel_odom_msg.pose.pose.orientation.z,
             self.ekf_wheel_odom_msg.pose.pose.orientation.w])
        offset_wheel_odom_roll, offset_wheel_odom_pitch, offset_wheel_odom_yaw = euler_from_quaternion(
            [self.offset_navheading_wheel_odom_msg.pose.pose.orientation.x,
             self.offset_navheading_wheel_odom_msg.pose.pose.orientation.y,
             self.offset_navheading_wheel_odom_msg.pose.pose.orientation.z,
             self.offset_navheading_wheel_odom_msg.pose.pose.orientation.w])
        offset_rtk_odom_roll, offset_rtk_odom_pitch, offset_rtk_odom_yaw = euler_from_quaternion(
            [self.offset_rtk_odom_msg.pose.pose.orientation.x,
             self.offset_rtk_odom_msg.pose.pose.orientation.y,
             self.offset_rtk_odom_msg.pose.pose.orientation.z,
             self.offset_rtk_odom_msg.pose.pose.orientation.w])

        n_off_distance, n_off_theta = cart2pol(
            self.ekf_wheel_odom_msg.pose.pose.position.x - self.offset_navheading_wheel_odom_msg.pose.pose.position.x,
            self.ekf_wheel_odom_msg.pose.pose.position.y - self.offset_navheading_wheel_odom_msg.pose.pose.position.y)
        n_off_rtk_distance, n_off_rtk_theta = cart2pol(
            self.rtk_odom_heading_msg.pose.pose.position.x - self.offset_rtk_odom_msg.pose.pose.position.x,
            self.rtk_odom_heading_msg.pose.pose.position.y - self.offset_rtk_odom_msg.pose.pose.position.y)

        n_x, n_y = pol2cart(n_off_distance, n_off_theta + offset_rtk_odom_yaw)
        rtk_x, rtk_y = pol2cart(n_off_distance, n_off_rtk_theta)

        # self.navheading_wheel_odom_msg.pose.pose.position.x = self.offset_rtk_odom_msg.pose.pose.position.x + n_x
        # self.navheading_wheel_odom_msg.pose.pose.position.y = self.offset_rtk_odom_msg.pose.pose.position.x + n_y
        self.navheading_wheel_odom_msg.pose.pose.position.x = self.offset_rtk_odom_msg.pose.pose.position.x + rtk_x
        self.navheading_wheel_odom_msg.pose.pose.position.y = self.offset_rtk_odom_msg.pose.pose.position.y + rtk_y
        self.navheading_wheel_odom_msg.pose.pose.position.z = self.offset_rtk_odom_msg.pose.pose.position.z

        wheel_odom_yaw = wheel_odom_yaw - offset_wheel_odom_yaw
        wheel_odom_yaw = wheel_odom_yaw + offset_rtk_odom_yaw
        quaternion = quaternion_from_euler(wheel_odom_roll, wheel_odom_pitch, wheel_odom_yaw)
        self.navheading_wheel_odom_msg.pose.pose.orientation.x = quaternion[0]
        self.navheading_wheel_odom_msg.pose.pose.orientation.y = quaternion[1]
        self.navheading_wheel_odom_msg.pose.pose.orientation.z = quaternion[2]
        self.navheading_wheel_odom_msg.pose.pose.orientation.w = quaternion[3]

        self.navheading_wheel_odom_msg.pose.covariance = self.ekf_wheel_odom_msg.pose.covariance
        self.navheading_wheel_odom_msg.twist.twist = self.ekf_wheel_odom_msg.twist.twist
        self.navheading_wheel_odom_msg.twist.covariance = self.ekf_wheel_odom_msg.twist.covariance
        self.nav_wheel_odom_pub.publish(self.navheading_wheel_odom_msg)


        # self.navheading_wheel_odom_msg = Odometry()
        # self.navheading_wheel_odom_msg.header.stamp = self.filtered_wheel_odom_msg.header.stamp
        # self.navheading_wheel_odom_msg.header.frame_id = self.filtered_wheel_odom_msg.header.frame_id
        #
        #
        # wheel_odom_roll, wheel_odom_pitch, wheel_odom_yaw = euler_from_quaternion(
        #     [self.filtered_wheel_odom_msg.pose.pose.orientation.x,
        #      self.filtered_wheel_odom_msg.pose.pose.orientation.y,
        #      self.filtered_wheel_odom_msg.pose.pose.orientation.z,
        #      self.filtered_wheel_odom_msg.pose.pose.orientation.w])
        # nav_odom_roll, nav_odom_pitch, nav_odom_yaw = euler_from_quaternion(
        #     [self.fused_msg.orientation.x,
        #      self.fused_msg.orientation.y,
        #      self.fused_msg.orientation.z,
        #      self.fused_msg.orientation.w])
        # wheel_odom_yaw_result = wheel_odom_yaw + nav_odom_yaw
        # quaternion = quaternion_from_euler(nav_odom_roll, nav_odom_pitch, wheel_odom_yaw_result)
        # self.navheading_wheel_odom_msg.pose.pose.orientation.x = quaternion[0]
        # self.navheading_wheel_odom_msg.pose.pose.orientation.y = quaternion[1]
        # self.navheading_wheel_odom_msg.pose.pose.orientation.z = quaternion[2]
        # self.navheading_wheel_odom_msg.pose.pose.orientation.w = quaternion[3]
        #
        #
        #
        #
        #
        #
        # n_distance, n_theta = cart2pol(self.filtered_wheel_odom_msg.pose.pose.position.x,
        #                                self.filtered_wheel_odom_msg.pose.pose.position.y)
        #
        # n_x, n_y = pol2cart(n_distance, self.nav_wheel_odom_offset_theta - wheel_odom_yaw_result)
        # # print("##################")
        # # print("n_distance: {:.2f} n_theta: {:.2f} imu_theta_offset: {:.2f} n_x: {:.2f}, n_y: {:.2f}".format(
        # #                                                                n_distance,
        # #                                                                 math.degrees(n_theta),
        # #                                                                math.degrees(wheel_odom_yaw_result),
        # #                                                                n_x,
        # #                                                                n_y))
        # # print("##################")
        #
        # self.navheading_wheel_odom_msg.pose.pose.position.x = n_x
        # self.navheading_wheel_odom_msg.pose.pose.position.y = n_y
        # self.navheading_wheel_odom_msg.pose.pose.position.z = self.filtered_wheel_odom_msg.pose.pose.position.z
        #
        #
        #
        # self.navheading_wheel_odom_msg.pose.covariance = self.filtered_wheel_odom_msg.pose.covariance
        # self.navheading_wheel_odom_msg.twist.twist = self.filtered_wheel_odom_msg.twist.twist
        # self.navheading_wheel_odom_msg.twist.covariance = self.filtered_wheel_odom_msg.twist.covariance
        # # print(self.filtered_wheel_odom_msg.pose.pose)
        # self.nav_wheel_odom_pub.publish(self.navheading_wheel_odom_msg)

    def srv_cb_reset_wheel_odom(self, srv):
        """
        Service callback to set the wheel odom offset. it zero the wheel odom.
        :param srv:
        :return:
        """
        self.reset_wheel_odom()
        return EmptyResponse()

    def srv_cb_set_nav_wheel_odom(self, srv):
        """
        Service callback to set the wheel odom to navheading offset. Make wheel odom same orientation as navheading.
        :param srv:
        :return:
        """
        self.set_nav_wheel_odom()
        return EmptyResponse()

    def srv_cb_set_pose(self, srv):
        """
        Service callback to set the initial pose.
        :param srv: robot_localization/SetPose
        :return: robot_localization/SetPoseResponse
        """
        self.set_nav_pose(srv)
        return SetPoseResponse()

    def reset_wheel_odom(self):
        self.offset_wheel_odom_msg = copy.deepcopy(self.wheel_odom_msg)

    def set_nav_pose(self, pose):
        """
        Set the initial pose of the robot.
        :param srv: geometry_msgs/Pose
        :return:
        """
        print("Set pose ")
        print(pose)
        self.set_pose_pose = pose
        self.set_pose_yaw = euler_from_quaternion([pose.orientation.x,
                                                   pose.orientation.y,
                                                   pose.orientation.z,
                                                   pose.orientation.w])[2]
        print("Set pose yaw: ", self.set_pose_yaw)

    def set_nav_wheel_odom(self):
        self.offset_navheading_wheel_odom_msg = copy.deepcopy(self.wheel_odom_msg)
        self.offset_rtk_odom_msg = copy.deepcopy(self.rtk_odom_heading_msg)

    def set_wheel_base_pose(self):
        rospy.wait_for_service('/ekf_wheel_odometry/set_pose')
        try:
            pose_srv = SetPoseRequest()
            pose_srv.pose.pose.pose.position.x = self.rtk_odom_msg.pose.pose.position.x
            pose_srv.pose.pose.pose.position.y = self.rtk_odom_msg.pose.pose.position.y
            pose_srv.pose.pose.pose.position.z = self.rtk_odom_msg.pose.pose.position.z
            # pose_srv.pose.pose.pose.orientation.x = self.fused_msg.orientation.x
            # pose_srv.pose.pose.pose.orientation.y = self.fused_msg.orientation.y
            # pose_srv.pose.pose.pose.orientation.z = self.fused_msg.orientation.z
            # pose_srv.pose.pose.pose.orientation.w = self.fused_msg.orientation.w
            pose_srv.pose.pose.pose.orientation.x = self.rtk_odom_msg.pose.pose.orientation.x
            pose_srv.pose.pose.pose.orientation.y = self.rtk_odom_msg.pose.pose.orientation.y
            pose_srv.pose.pose.pose.orientation.z = self.rtk_odom_msg.pose.pose.orientation.z
            pose_srv.pose.pose.pose.orientation.w = self.rtk_odom_msg.pose.pose.orientation.w
            pose_srv.pose.header.frame_id = ('odom')
            pose_srv.pose.header.stamp = rospy.Time.now()

            srv = rospy.ServiceProxy('/ekf_wheel_odometry/set_pose', SetPose)  # Connect to the service
            response = srv(pose_srv)
            print("Response: ", response)
        except rospy.ServiceException as e:
            rospy.logerr("Service /ekf_wheel_odometry/set_pose call failed: %s" % e)

    def set_wheel_base_nav_pose(self, pose=None):
        rospy.wait_for_service('/ekf_wheel_nav_odometry/set_pose')
        try:
            pose_srv = SetPoseRequest()
            if pose is None:
                pose_srv.pose.pose.pose.position.x = self.rtk_odom_msg.pose.pose.position.x
                pose_srv.pose.pose.pose.position.y = self.rtk_odom_msg.pose.pose.position.y
                pose_srv.pose.pose.pose.position.z = self.rtk_odom_msg.pose.pose.position.z
                pose_srv.pose.pose.pose.orientation.x = self.fused_msg.orientation.x
                pose_srv.pose.pose.pose.orientation.y = self.fused_msg.orientation.y
                pose_srv.pose.pose.pose.orientation.z = self.fused_msg.orientation.z
                pose_srv.pose.pose.pose.orientation.w = self.fused_msg.orientation.w
                pose_srv.pose.header.frame_id = ('odom')
            else:
                pose_srv.pose.pose.pose = pose
                pose_srv.pose.header.frame_id = ('map')

            pose_srv.pose.header.stamp = rospy.Time.now()

            srv = rospy.ServiceProxy('/ekf_wheel_nav_odometry/set_pose', SetPose)  # Connect to the service
            response = srv(pose_srv)
            # print("Response: ", response)

        except rospy.ServiceException as e:
            rospy.logerr("Service /ekf_wheel_nav_odometry/set_pose call failed: %s" % e)


    def offset_navheading_imu(self, msg):
        """
        Compute offset of the navheading with the imu heading. Do only if the rtk navheading is usable.
        :param msg: rtk navheading msg
        :return:
        """
        delay = abs(int(str((msg.header.stamp - self.imu_msg.header.stamp) / 1000000)))  # in ms
        self.imu_delay = delay
        rtk_error_z = msg.orientation_covariance[8]
        self.rtk_err = rtk_error_z
        if delay <= 6 and self.fix_usable and self.heading_fix_usable:

            # if rtk_error_z <= 0.00068 and rtk_error_z >= 0.00042:
            if rtk_error_z <= 0.00065 and rtk_error_z >= 0.00050:
                self.nav_last_fused_time = rospy.Time.now()
                (imu_roll, imu_pitch, imu1_yaw) = euler_from_quaternion([self.imu_msg.orientation.x,
                                                                        self.imu_msg.orientation.y,
                                                                        self.imu_msg.orientation.z,
                                                                        self.imu_msg.orientation.w])
                (rtk_roll, rtk_pitch, rtk_yaw) = euler_from_quaternion([msg.orientation.x,
                                                                        msg.orientation.y,
                                                                        msg.orientation.z,
                                                                        msg.orientation.w])
                if rtk_yaw < 0:
                    rtk_yaw += 2 * math.pi
                if imu1_yaw < 0:
                    imu1_yaw += 2 * math.pi
                self.diff_yaw = (rtk_yaw - imu1_yaw) + 1.5708  # 90 degrees
                self.init = False

    def print_info(self):

        # print("fused: {:.3f}s fix_usable: {:1} heading_usable: {:1} rtk_err: {:.5f} imu_delay: {:2} diff_yaw: {:.5f}"
        #       .format(fused_nav_time, self.fix_usable, self.heading_fix_usable, self.rtk_err, self.imu_delay, self.diff_yaw))
        print("fused fix: {:.3f}s fused nav: {:.3f}s fix_usable: {:1} heading_usable: {:1} rtk_err: {:.5f}"
              .format(self.fused_fix_time, self.fused_nav_time, self.fix_usable, self.heading_fix_usable, self.rtk_err))



    def publish_imu_baselink(self):
        """
        Publish imu in base_link frame. Obsolete
        :return:
        """
        imu = Imu()
        imu.header.stamp = rospy.Time.now()
        imu.header.frame_id = "base_link"
        imu.orientation = self.imu_msg.orientation
        imu.orientation_covariance = self.imu_msg.orientation_covariance
        imu.angular_velocity = self.imu_msg.angular_velocity
        imu.angular_velocity_covariance = self.imu_msg.angular_velocity_covariance
        imu.linear_acceleration = self.imu_msg.linear_acceleration
        imu.linear_acceleration_covariance = self.imu_msg.linear_acceleration_covariance
        self.base_link_imu_pub.publish(imu)

    def publish_fused_navheading(self):
        """
        Publish fused navheading based on the imu heading and the offset.
        :return:
        """
        (imu_roll, imu_pitch, imu_yaw) = euler_from_quaternion([self.imu_msg.orientation.x,
                                                                self.imu_msg.orientation.y,
                                                                self.imu_msg.orientation.z,
                                                                self.imu_msg.orientation.w])
        if imu_yaw < 0:
            imu_yaw += 2 * math.pi

        if self.init:
            imu_yaw = imu_yaw + self.set_pose_yaw
        else:
            imu_yaw = imu_yaw + self.diff_yaw
        quaternion = quaternion_from_euler(imu_roll, imu_pitch, imu_yaw)
        fused_nav = Imu()
        fused_nav.header.stamp = rospy.Time.now()
        fused_nav.header.frame_id = "base_link"
        fused_nav.orientation.x = quaternion[0]
        fused_nav.orientation.y = quaternion[1]
        fused_nav.orientation.z = quaternion[2]
        fused_nav.orientation.w = quaternion[3]
        fused_nav.orientation_covariance = self.imu_msg.orientation_covariance
        fused_nav.angular_velocity = self.imu_msg.angular_velocity
        fused_nav.angular_velocity_covariance = self.imu_msg.angular_velocity_covariance
        fused_nav.linear_acceleration = self.imu_msg.linear_acceleration
        fused_nav.linear_acceleration_covariance = self.imu_msg.linear_acceleration_covariance
        self.fused_nav_pub.publish(fused_nav)
        self.fused_msg = fused_nav
        (fused_roll, fused_pitch, fused_yaw) = euler_from_quaternion([self.fused_msg.orientation.x,
                                                                      self.fused_msg.orientation.y,
                                                                      self.fused_msg.orientation.z,
                                                                      self.fused_msg.orientation.w])
        self.fused_yaw = fused_yaw
    def publish_yaws(self):
        yaws = Float32MultiArray()
        yaw_navheading = round(self.navheading_yaw * 180.0 / math.pi, 2)
        yaw_fused = round(self.fused_yaw * 180.0 / math.pi, 2)
        yaw_imu = round(self.imu_yaw * 180.0 / math.pi, 2)
        yaws.data = [yaw_imu, yaw_navheading, yaw_fused]
        self.yaws_pub.publish(yaws)

    def update_navheading_marker(self):
        quat_check = (self.navheading_msg.orientation.x
                      + self.navheading_msg.orientation.y
                      + self.navheading_msg.orientation.z
                      + self.navheading_msg.orientation.w)
        if quat_check != 0.0:
            color = [1.0, 0.0, 1.0, 1.0]
            scale = [2.0, 0.5, 1.0]
            self.marker_nav = self.get_marker(self.navheading_msg.orientation.x,
                                              self.navheading_msg.orientation.y,
                                              self.navheading_msg.orientation.z,
                                              self.navheading_msg.orientation.w,
                                              color, scale, z_offset=0.0, id=10, ns="NAV_RTK")

    def update_fuse_marker(self):
        quat_check = (self.fused_msg.orientation.x
                      + self.fused_msg.orientation.y
                      + self.fused_msg.orientation.z
                      + self.fused_msg.orientation.w)
        if quat_check != 0.0:
            color = [1.0, 1.0, 1.0, 0.6]
            scale = [2.2, 1.2, 1.2]
            self.marker_fuse = self.get_marker(self.fused_msg.orientation.x,
                                              self.fused_msg.orientation.y,
                                              self.fused_msg.orientation.z,
                                              self.fused_msg.orientation.w,
                                              color, scale, z_offset=0.0, id=10, ns="NAV_FUSE")

    def update_imu_marker(self):
        quat_check = (self.imu_msg.orientation.x
                      + self.imu_msg.orientation.y
                      + self.imu_msg.orientation.z
                      + self.imu_msg.orientation.w)
        if quat_check != 0.0:
            color = [0.0, 1.0, 0.0, 1.0]
            scale = [2.0, 0.5, 1.0]
            self.marker_imu = self.get_marker(self.imu_msg.orientation.x,
                                              self.imu_msg.orientation.y,
                                              self.imu_msg.orientation.z,
                                              self.imu_msg.orientation.w,
                                              color, scale, z_offset=0.0, id=10, ns="NAV_IMU")



    def get_marker(self, x, y, z, w, color, scale, z_offset, id, ns):
        marker = Marker()
        marker.header.frame_id = "bno_imu_link"  # overide frame for alignment in view
        marker.header.stamp = rospy.Time.now()
        marker.ns = ns
        marker.type = Marker.ARROW
        # marker.type = Marker.CUBE
        marker.id = id
        marker.scale.x = scale[0]
        marker.scale.y = scale[1]
        marker.scale.z = scale[2]
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]
        marker.pose.position.x = 0
        marker.pose.position.y = 0
        marker.pose.position.z = -z_offset
        marker.pose.orientation.x = x
        marker.pose.orientation.y = y
        marker.pose.orientation.z = z
        marker.pose.orientation.w = w
        return marker

    def publish_markers(self):
        self.update_navheading_marker()
        self.update_fuse_marker()
        self.update_imu_marker()
        m_array = MarkerArray()
        m_array.markers = [self.marker_nav, self.marker_fuse, self.marker_imu]
        self.marker_pub.publish(m_array)

    def loop10hz(self, event=None):
        node_gnss.publish_yaws()

        # node_gnss.publish_markers()

    def loop3hz(self, event=None):
        node_gnss.publish_markers()


    def loop1hz(self, event=None):
        node_gnss.print_info()

    def loop3sec(self, event=None):
        # print(sum(self.fix_usable_history))
        if sum(self.fix_usable_history) == 3:
            self.fix_last_fused_time = rospy.Time.now()
            # self.set_nav_wheel_odom()
            self.set_wheel_base_nav_pose()
            # print("############################################ fuse_rtk_and_wheel_odom ############################################")

    def loop30hz(self, event=None):
        self.publish_relative_wheel_odom()
        self.publish_odom_for_gps()
        self.publish_navheading_wheel_odom()
        self.publish_odom_fused()
        self.transform_odom_to_base_link()
        self.transform_map_to_odom()
        self.fused_nav_time = (int(str(rospy.Time.now())) - int(str(self.nav_last_fused_time))) / 1000000000
        self.fused_fix_time = (int(str(rospy.Time.now())) - int(str(self.fix_last_fused_time))) / 1000000000


if __name__ == '__main__':
    try:
        rospy.init_node('GNSS_node')
        rospy.loginfo("<{}> Initialising GNSS_node...".format(rospy.get_caller_id()))
        node_gnss = NavTfNode()
        rate = 10
        r = rospy.Rate(rate)  # 10Hz

        def signal_handler(signum, frame):
            rospy.signal_shutdown("end")
            # sys.exit(0)
        signal.signal(signal.SIGINT, signal_handler)

        rospy.loginfo("<{}> Initialised.".format(rospy.get_caller_id()))
        rospy.Timer(rospy.Duration(1.0), node_gnss.loop1hz)
        rospy.Timer(rospy.Duration(1.0 / 3.0), node_gnss.loop3hz)
        rospy.Timer(rospy.Duration(1.0 / 30.0), node_gnss.loop30hz)
        rospy.Timer(rospy.Duration(1.0 / 10.0), node_gnss.loop10hz)
        rospy.Timer(rospy.Duration(1.0/10), node_gnss.loop3sec)

        rospy.spin()

    except rospy.ROSInterruptException:
        rospy.loginfo(rospy.get_caller_id() + "  GNSS_node error.")
