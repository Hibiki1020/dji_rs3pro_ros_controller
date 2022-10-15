#!/usr/bin/env python3
import struct
from ctypes import *
import numpy as np
import math
import argparse
import yaml
import random

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
    def __init__(self, CFG):
        super().__init__()

        print(self.roll)

        # super.__init__(self, 
        # header, 
        # enc, 
        # res1,
        # res2,
        # res3, 
        # seq, 
        # send_id, 
        # recv_id, 
        # FRAME_LEN,
        # can_recv_msg_buffer,
        # can_recv_msg_len_buffer,
        # can_recv_msg_buffer_len,
        # total_byte_data,
        # roll, pitch, yaw)

        # self.header = header
        # self.enc = enc
        # self.res1 = res1
        # self.res2 = res2
        # self.res3 = res3
        # self.seq = seq
        # self.send_id = send_id
        # self.recv_id = recv_id
        # self.FRAME_LEN = FRAME_LEN
        # self.can_recv_msg_buffer = can_recv_msg_buffer
        # self.can_recv_msg_len_buffer = can_recv_msg_len_buffer
        # self.can_recv_msg_buffer_len = can_recv_msg_buffer_len
        # self.total_byte_data = total_byte_data
        
        # self.roll = roll
        # self.pitch = pitch
        # self.yaw = yaw

        self.CFG = CFG
        self.record_data = bool(CFG["record_data"])
        self.record_data_path = str(CFG["record_data_path"])
        self.print_status_checker = bool(CFG["print_status"])
        self.roll_max = float(CFG["roll_max"])
        self.roll_min = float(CFG["roll_min"])
        self.pitch_max = float(CFG["pitch_max"])
        self.pitch_min = float(CFG["pitch_min"])
        self.yaw_max = float(CFG["yaw_max"])
        self.yaw_min = float(CFG["yaw_min"])

        self.angular_velocity_threshold = float(CFG["angular_velocity_threshold"])
        self.target_angle_threshold = float(CFG["target_angle_threshold"])

        self.imu_angle = EularAngle()
        self.target_angle = EularAngle()
        self.img_data = Image()

        self.target_roll = 0.0
        self.target_pitch = 0.0
        self.target_yaw = 0.0

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
            #print(type(self.trans))
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
        self.rate = rospy.Rate(200)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.sub_imu_data = rospy.Subscriber("/imu/data", Imu, self.imu_data_callback)
        self.sub_img_data = rospy.Subscriber("/camera/image_raw", Image, self.img_data_callback)

    def print_status(self):
        print("Current position: yaw: {yaw}, pitch: {pitch}, roll: {roll}".format(yaw=self.yaw, pitch=self.pitch, roll=self.roll))

    def reach_target_angle(self):
        checker = False
        tmp_target_roll = self.target_roll/180.0*math.pi
        tmp_target_pitch = self.target_pitch/180.0*math.pi

        print(self.roll, self.pitch)
        print(tmp_target_roll, tmp_target_pitch)

        if abs(tmp_target_roll - self.roll) < self.target_angle_threshold and abs(tmp_target_pitch - self.pitch) < self.target_angle_threshold:
            checker = True
        else:
            checker = False

        return checker

    def request_target_position(self):
        print("Set New Target Angle")

        correct_target_angle = False

        while correct_target_angle==False:

            self.target_roll = random.uniform(self.roll_min, self.roll_max)
            self.target_pitch = random.uniform(self.pitch_min, self.pitch_max)
            self.target_yaw = random.uniform(self.yaw_min, self.yaw_max)

            tmp_current_roll = self.roll /math.pi * 180.0
            tmp_current_pitch = self.pitch /math.pi * 180.0
            tmp_current_yaw = self.yaw /math.pi * 180.0

            if(abs(self.target_roll - tmp_current_roll) > self.angular_velocity_threshold or abs(self.target_pitch - tmp_current_pitch) > self.angular_velocity_threshold or abs(self.target_yaw - tmp_current_yaw) > self.angular_velocity_threshold):
                correct_target_angle = True
                self.target_yaw = 0.0
                if self.print_status_checker:
                    print("Target angle set")
                    print("Target Angle: yaw: {yaw}, pitch: {pitch}, roll: {roll}".format(yaw=self.target_yaw, pitch=self.target_pitch, roll=self.target_roll))

    def set_attitude_control(self, in_yaw, in_roll, in_pitch):
        # yaw, roll, pitch in 0.1 steps (-1800,1800)
        # ctrl_byte always to 1
        # time_for_action to define speed in 0.1sec
        yaw = int(in_yaw*10.0)
        roll = int(in_roll*10.0)
        pitch = int(in_pitch*10.0)

        ctrl_byte = 0x01
        time_for_action = 0x14 # 2.0sec
        hex_data = struct.pack('<3h2B', yaw, roll, pitch,
                               ctrl_byte, time_for_action)

        pack_data = ['{:02X}'.format(struct.unpack('<1B', b'i')[
            0]) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        # print(cmd_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                                    cmd_id='00', data=cmd_data)

        self.send_cmd(cmd)
        return True

    def controller_spin(self):
        counter = 0
        while not rospy.is_shutdown():
            self.request_current_position()
            if self.print_status_checker:
                self.print_status()
            
            if self.reach_target_angle() or counter == 0:
                self.request_target_position()
                self.set_attitude_control(self.target_yaw, self.target_roll, self.target_pitch)
            
            counter += 1
            self.rate.sleep()




if __name__ == '__main__':
    parser = argparse.ArgumentParser("./control_gimbal_angle.py")
    parser.add_argument(
        '--train_cfg', '-c',
        type=str,
        required=False,
        default="../yaml/control_gimbal_angle.yaml",
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