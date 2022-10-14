#!/usr/bin/env python3
import struct
from ctypes import *
import numpy as np

import rospy
import tf2_ros
import tf2_geometry_msgs
import tf
import geometry_msgs
import math

from std_msgs.msg import String
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from can_msgs.msg import Frame
import sys
import os

#Need in running in ROS
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append('../')
from scripts.check_sum import *
from dji_rs3pro_ros_controller.srv import *
from dji_rs3pro_ros_controller.msg import *

class CheckAngleDetectionDelay(object):
    def __init__(self):
        print("Starting CheckAngleDetectionDelay")
        self.imu_data = Imu()
        self.imu_angle = EularAngle()
        self.gimbal_angle = EularAngle()
        self.now_time = 0.0
        self.last_time = 0.0

    def imu_data_callback(self, msg):
        self.imu_data = msg
        self.imu_data.header.frame_id = "imu_link"

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.header = self.imu_data.header
        pose_stamped.pose.orientation = self.imu_data.orientation
        
        try:
            self.trans = self.tfBuffer.lookup_transform('imu_link', 'end_effector', rospy.Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.rate.sleep()

        pose_trans = tf2_geometry_msgs.do_transform_pose(pose_stamped, self.trans)
        #print(pose_trans)

        roll, pitch, yaw = self.euler_from_quaternion(pose_trans.pose.orientation.x, pose_trans.pose.orientation.y, pose_trans.pose.orientation.z, pose_trans.pose.orientation.w)
        self.imu_angle.roll = yaw + 180.0
        self.imu_angle.pitch = -1.0* roll
        self.imu_angle.yaw = pitch
        #print("imu_angle:", self.imu_angle)


    def euler_from_quaternion(self, x, y, z, w):
        """
        Convert a quaternion into euler angles (roll, pitch, yaw)
        roll is rotation around x in radians (counterclockwise)
        pitch is rotation around y in radians (counterclockwise)
        yaw is rotation around z in radians (counterclockwise)
        """
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)/math.pi*180.0
     
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)/math.pi*180.0
     
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)/math.pi*180.0
     
        return roll_x, pitch_y, yaw_z

    def gimbal_angle_callback(self, msg):
        self.gimbal_angle = msg
        self.gimbal_angle.roll = self.gimbal_angle.roll/math.pi*180.0
        self.gimbal_angle.pitch = self.gimbal_angle.pitch/math.pi*180.0
        self.gimbal_angle.yaw = self.gimbal_angle.yaw/math.pi*180.0

        self.last_time = self.now_time
        self.now_time = rospy.get_time()
        dt = self.now_time - self.last_time

        print("Delta Time: ",dt)
        print("gimbal_angle roll: {:4f}, pitch: {:4f}, yaw: {:4f}".format(self.gimbal_angle.roll, self.gimbal_angle.pitch, self.gimbal_angle.yaw))
        print("imu_angle roll   : {:4f}, pitch: {:4f}, yaw: {:4f}".format(self.imu_angle.roll, self.imu_angle.pitch, self.imu_angle.yaw))
        print("\n\n")

    def ros_init(self):
        rospy.init_node('check_angle_detection_delay', anonymous=True)
        self.rate = rospy.Rate(10)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.sub_imu_data = rospy.Subscriber("/imu/data", Imu, self.imu_data_callback)
        self.sub_gimbal_angle = rospy.Subscriber("gimbal_angle", EularAngle, self.gimbal_angle_callback)

    def ros_spin(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        rosnode = CheckAngleDetectionDelay()
        rosnode.ros_init()
        rosnode.ros_spin()

    except rospy.ROSInterruptException:
        pass
