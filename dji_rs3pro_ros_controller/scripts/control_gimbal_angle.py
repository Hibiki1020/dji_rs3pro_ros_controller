#!/usr/bin/env python3
import struct
from ctypes import *
import numpy as np
import math
import argparse
import yaml
import random
import ctypes as ct

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

        self.delta_time_min = int(CFG["delta_time_min"])
        self.delta_time_max = int(CFG["delta_time_max"])

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
            self.trans = self.tfBuffer.lookup_transform('imu_link', 'gimbal_base', rospy.Time())
            #print(type(self.trans))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            self.rate.sleep()

        pose_trans = tf2_geometry_msgs.do_transform_pose(pose_stamped, self.trans)
        #print(pose_trans)

        eular = tf.transformations.euler_from_quaternion((pose_trans.pose.orientation.x, pose_trans.pose.orientation.y, pose_trans.pose.orientation.z, pose_trans.pose.orientation.w))
        eular = tf.transformations.euler_from_quaternion((self.imu_data.orientation.x, self.imu_data.orientation.y, self.imu_data.orientation.z, self.imu_data.orientation.w))

        self.imu_angle.roll = eular[1] * -1.0
        self.imu_angle.pitch = (eular[0] + math.pi/2) * -1.0
        self.imu_angle.yaw = eular[2]

        self.pub_imu_correct_angle.publish(self.imu_angle)


    def ros_init_controller(self):
        rospy.init_node('gimbal_controller', anonymous=True)

    def set_param_in_controller(self):
        self.rate = rospy.Rate(50)
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        
        self.sub_imu_data = rospy.Subscriber("/imu/data", Imu, self.imu_data_callback)
        self.sub_img_data = rospy.Subscriber("/camera/image_raw", Image, self.img_data_callback)

        self.pub_imu_correct_angle = rospy.Publisher("/imu_correct_angle", EularAngle, queue_size=1)
        
        #self.service_set_angle = rospy.Service('send_joint_cmd', SendJointPos, self.send_joint_pos)
        #self.service_set_velocity = rospy.Service('send_joint_speed_cmd', SendJointSpeed, self.send_joint_speed_cmd)

    def print_status(self):
        print("Current position: roll:{:.3f}, pitch:{:.3f}, yaw:{:.3f}".format(self.roll/math.pi*180.0, self.pitch/math.pi*180.0, self.yaw/math.pi*180.0))
        deg_roll = self.roll/math.pi*180.0
        deg_pitch = self.pitch/math.pi*180.0
        deg_yaw = self.yaw/math.pi*180.0

        if deg_roll > self.roll_max or deg_roll < self.roll_min:
            rospy.logerr("Roll angle is out of range by handcarring")

        if deg_pitch > self.pitch_max or deg_pitch < self.pitch_min:
            rospy.logerr("Pitch angle is out of range by handcarring")

        if deg_yaw > self.yaw_max or deg_yaw < self.yaw_min:
            rospy.logerr("Yaw angle is out of range by handcarring")

    def reach_target_angle(self):
        checker = False
        tmp_target_roll = self.target_roll/180.0*math.pi
        tmp_target_pitch = self.target_pitch/180.0*math.pi

        target_angle_threshold_rad = self.target_angle_threshold/180.0*math.pi

        #print(self.roll, self.pitch)
        #print(tmp_target_roll, tmp_target_pitch)

        if abs(tmp_target_roll - self.roll) < target_angle_threshold_rad and abs(tmp_target_pitch - self.pitch) < target_angle_threshold_rad:
            print("ReachTargetAngle")
            #print(tmp_target_roll, tmp_target_pitch)
            #print(self.roll, self.pitch)
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

            self.target_delta = random.randint(self.delta_time_min, self.delta_time_max)

            tmp_current_roll = self.roll /math.pi * 180.0
            tmp_current_pitch = self.pitch /math.pi * 180.0
            tmp_current_yaw = self.yaw /math.pi * 180.0

            if(abs(self.target_roll - tmp_current_roll)/float(self.target_delta) > self.angular_velocity_threshold or abs(self.target_pitch - tmp_current_pitch)/float(self.target_delta) > self.angular_velocity_threshold or abs(self.target_yaw - tmp_current_yaw) > self.angular_velocity_threshold):
                correct_target_angle = True
                self.target_yaw = 0.0
                
                print("Target angle set")
                print("Target Angle: roll: {roll}, pitch: {pitch}, yaw: {yaw}".format(yaw=self.target_yaw, pitch=self.target_pitch, roll=self.target_roll))

    def set_attitude_control(self):
        # yaw, roll, pitch in 0.1 steps (-1800,1800)
        # ctrl_byte always to 1
        # time_for_action to define speed in 0.1sec
        yaw = int(self.target_yaw*10)
        roll = int(self.target_roll*10)
        pitch = int(self.target_pitch*10)

        print(yaw)

        success = False
        if -1800 <= yaw <= 1800 and -1800 <= roll <= 1800 and -1800 <= pitch <= 1800:
            success = self.setPosControl(yaw, roll, pitch)

        print("Send Command Control: ", success)


    def setPosControl(self, yaw, roll, pitch):
        ctrl_byte = 0x01
        #time_for_action = 0x14 # 2.0sec
        time_for_action = int(str(self.target_delta), base=16)
        hex_data = struct.pack('<3h2B', yaw, roll, pitch,
                               ctrl_byte, time_for_action)

        #pack_data = ['{:02X}'.format(struct.unpack('<1B', b'i')[0]) for i in hex_data]
        pack_data = ['{:02X}'.format(struct.unpack('<1B', i.to_bytes(1,'big'))[0]) for i in hex_data]
        cmd_data = ':'.join(pack_data)
        print(cmd_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                                    cmd_id='00', data=cmd_data)


        print(cmd)

        self.send_cmd(cmd)
        return True

    def send_joint_speed_cmd(self, req):
        # Angular speeds in 0.1 deg/sec
        yaw = req.yaw * 10
        pitch = req.pitch * 10
        roll = req.roll * 10

        success = False
        if -3600 <= yaw <= 3600 and -3600 <= roll <= 3600 and -3600 <= pitch <= 3600:
            success = self.setSpeedControl(yaw, roll, pitch)
        return SendJointSpeedResponse(success)

    def setSpeedControl(self, yaw, roll, pitch, ctrl_byte=0x80):
        hex_data = struct.pack('<3hB', yaw, roll, pitch, ctrl_byte)
        pack_data = ['{:02X}'.format(struct.unpack('<1B', i)[
            0]) for i in hex_data]
        cmd_data = ':'.join(pack_data)

        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                                    cmd_id='01', data=cmd_data)
        # print('cmd---data {}'.format(cmd))
        self.send_cmd(cmd)
        return True

    def controller_spin(self):
        counter = 0
        while not rospy.is_shutdown():
            if counter == 0:
                rospy.sleep(2.0)
            
            self.request_current_position()
            self.publish_current_position()
            self.rate.sleep() #ここでsleepしないと送信と受信を同時に行ってしまう

            if self.print_status_checker:
                self.print_status()
            
            if self.reach_target_angle() or counter == 0:
                print("Request New Target Angle")
                self.request_target_position()
                self.set_attitude_control()

            # print("gimbal_angle roll: {:4f}, pitch: {:4f}, yaw: {:4f}".format(self.pub_angle.roll, self.pub_angle.pitch, self.pub_angle.yaw))
            # print("imu_angle roll   : {:4f}, pitch: {:4f}, yaw: {:4f}".format(self.imu_angle.roll, self.imu_angle.pitch, self.imu_angle.yaw))
            # print("\n")
            
            counter += 1




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