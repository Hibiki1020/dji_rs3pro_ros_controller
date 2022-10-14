#!/usr/bin/env python3
import struct
from ctypes import *
import numpy as np
import math
import argparse
import yaml

import rospy
import tf

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Image
from can_msgs.msg import Frame
import tf2_ros
import tf2_geometry_msgs
import tf
import geometry_msgs
import sys
import os

#Need in running in ROS
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append('../')
from scripts.check_sum import *
from scripts.can_bus_controller import *
from dji_rs3pro_ros_controller.srv import *
from dji_rs3pro_ros_controller.msg import *

class GimbalController(GimbalBase):
    def __init__(self, 
        header, 
        enc, 
        res1,
        res2,
        res3, 
        seq, 
        send_id, 
        recv_id, 
        FRAME_LEN,
        can_recv_msg_buffer,
        can_recv_msg_len_buffer,
        can_recv_msg_buffer_len,
        total_byte_data,
        roll, pitch, yaw,
        CFG):

        super.__init__(self, 
        header, 
        enc, 
        res1,
        res2,
        res3, 
        seq, 
        send_id, 
        recv_id, 
        FRAME_LEN,
        can_recv_msg_buffer,
        can_recv_msg_len_buffer,
        can_recv_msg_buffer_len,
        total_byte_data,
        roll, pitch, yaw)

        self.header = header
        self.enc = enc
        self.res1 = res1
        self.res2 = res2
        self.res3 = res3
        self.seq = seq
        self.send_id = send_id
        self.recv_id = recv_id
        self.FRAME_LEN = FRAME_LEN
        self.can_recv_msg_buffer = can_recv_msg_buffer
        self.can_recv_msg_len_buffer = can_recv_msg_len_buffer
        self.can_recv_msg_buffer_len = can_recv_msg_buffer_len
        self.total_byte_data = total_byte_data
        
        self.roll = roll
        self.pitch = pitch
        self.yaw = yaw

        self.CFG = CFG
        self.record_data = bool(CFG["record_data"])
        self.record_data_path = str(CFG["record_data_path"])
        self.roll_max = float(CFG["roll_max"])
        self.roll_min = float(CFG["roll_min"])
        self.pitch_max = float(CFG["pitch_max"])
        self.pitch_min = float(CFG["pitch_min"])
        self.yaw_max = float(CFG["yaw_max"])
        self.yaw_min = float(CFG["yaw_min"])

        self.imu_angle = EularAngle()
        self.img_data = Image()

    def img_data_callback(self, msg):
        self.img_data = msg


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
        self.imu_angle.roll = yaw + math.pi
        self.imu_angle.pitch = -1.0* roll
        self.imu_angle.yaw = pitch
        #print("imu_angle:", self.imu_angle)


    def euler_from_quaternion(self, x, y, z, w):
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


    def ros_init_controller(self):
        rospy.init_node('gimbal_controller', anonymous=True)

    def set_param_in_controller(self):
        self.sub_imu_data = rospy.Subscriber("/imu/data", Imu, self.imu_data_callback)
        self.sub_img_data = rospy.Subscriber("/camera/image_raw", Image, self.img_data_callback)

    def controller_spin(self):
        while not rospy.is_shutdown():
            self.request_current_position()
            self.rate.sleep()




if __name__ == '__main__':
    parser = argparse.ArgumentParser("./control_gimbal_angle.py")
    parser.add_argument(
        '--train_cfg', '-c',
        type=str,
        required=True,
        help='Training configuration file'
    )

    FLAGS, unparsed = parser.parse_known_args()

    #Load yaml file
    try:
        print("Opening train config file %s", FLAGS.train_cfg)
        CFG = yaml.safe_load(open(FLAGS.train_cfg, 'r'))
    except Exception as e:
        print(e)
        print("Error opening train config file %s", FLAGS.train_cfg)
        quit()

    try:
        rosnode = GimbalController(CFG)
        rosnode.ros_init_controller()
        rosnode.set_param()
        rosnode.set_param_in_controller()
        rosnode.controller_spin()
    except rospy.ROSInterruptException:
        pass