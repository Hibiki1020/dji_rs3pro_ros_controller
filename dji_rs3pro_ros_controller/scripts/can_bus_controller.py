#!/usr/bin/env python
import struct
from ctypes import *
import numpy as np

import rospy
import tf

from std_msgs.msg import String
from sensor_msgs.msg import JointState
from can_msgs.msg import Frame
from dji_rs3pro_ros_controller.srv import *

from check_sum import *

class GimbalController(object):
    def __init__(self):
        self.header = 0xAA
        self.enc = 0x00
        self.res1 = 0x00
        self.res2 = 0x00
        self.res3 = 0x00
        self.seq = 0x0002

        self.send_id = int("223", 16)
        self.recv_id = int("222", 16)

        self.FRAME_LEN = 8

        self.can_recv_msg_buffer = []
        self.can_recv_msg_len_buffer = []
        self.can_recv_buffer_len = 10

        self.roll = 0.0
        self.pitch = 0.0
        self.yaw = 0.0

    def parse_position_response(self, data_frame):
        pos_data = data_frame[16:-4]
        yaw = int(
            '0x' + pos_data[1] + pos_data[0], base=16)
        roll = int(
            '0x' + pos_data[3] + pos_data[2], base=16)
        pitch = int(
            '0x' + pos_data[5] + pos_data[4], base=16)
        if yaw > 1800:
            yaw -= 65538
        if roll > 1800:
            roll -= 65538
        if pitch > 1800:
            pitch -= 65538

        self.yaw = yaw * 0.1 * np.pi / 180
        self.roll = roll * 0.1 * np.pi / 180
        self.pitch = pitch * 0.1 * np.pi / 180
        output = "Pitch: " + \
            str(self.pitch) + ", Yaw: " + \
            str(self.yaw) + ", Roll: " + str(self.roll)
        # print(output)
        self.br.sendTransform((0.0, 0.0, -1.0),
                              tf.transformations.quaternion_from_euler(
            0.0, 0.0, 0.0),
            rospy.Time.now(),
            "map",
            "gimbal_base")
        self.br.sendTransform((0.0, 0.0, 0.0),
                              tf.transformations.quaternion_from_euler(
                                  self.roll, self.pitch, self.yaw),
                              rospy.Time.now(),
                              "gimbal_base",
                              "end_effector")

    def parse_push_response(self, data_frame):
        # Function to display various gimbal parameters
        print("Catch push response")

    def can_callback(self, data):
        if data.id == self.recv_id:
            # print(len(data.data))
            # print(data)
            str_data = ['{:02X}'.format(struct.unpack('<1B', i)[
                                        0]) for i in data.data]
            # print(str_data)
            self.can_recv_msg_buffer.append(str_data)
            self.can_recv_msg_len_buffer.append(data.dlc)

            if len(self.can_recv_msg_buffer) > self.can_recv_buffer_len:
                # print("Pop")
                self.can_recv_msg_buffer.pop(0)
                self.can_recv_msg_len_buffer.pop(0)

            full_msg_frames = self.can_buffer_to_full_frame()

            # print(full_msg_frames)
            for hex_data in full_msg_frames:
                if self.validate_api_call(hex_data):
                    # ic(':'.join(hex_data[16:-4]))
                    request_data = ":".join(hex_data[12:14])
                    # print("Req: " + str(request_data))
                    if request_data == "0E:02":
                        # This is response data to a get position request
                        self.parse_position_response(hex_data)
                    if request_data == "0E:08":
                        self.parse_push_response(hex_data)

    def send_joint_pos(self, req):
        # print("Returning [%s + %s +%s ]" % (req.pitch, req.yaw, req.roll))

        yaw = 10 * req.yaw
        roll = 10 * req.roll
        pitch = 10 * req.pitch
        success = False
        if -1800 <= yaw <= 1800 and -1800 <= roll <= 1800 and -1800 <= pitch <= 1800:
            success = self.setPosControl(yaw, roll, pitch)
        return SendJointPosResponse(success)

    def send_joint_speed_cmd(self, req):
        # Angular speeds in 0.1 deg/sec
        yaw = req.yaw * 10
        pitch = req.pitch * 10
        roll = req.roll * 10

        success = False
        if -3600 <= yaw <= 3600 and -3600 <= roll <= 3600 and -3600 <= pitch <= 3600:
            success = self.setSpeedControl(yaw, roll, pitch)
        return SendJointSpeedResponse(success)

    def set_hyperparams(self):
        # Set the gimbal hyperparameters
        print("set_hyperparams")

    def ros_init(self):
        rospy.init_node('RS3Pro_Gimbal_Controller', anonymous=True)
        rate = rospy.Rate(10) # 10hz
        self.br = tf.TransformBroadcaster()

        self.sub_can_data = rospy.Subscriber('/gimbal_data', Frame, self.can_callback)
        self.pub_can_command = rospy.Publisher('/sent_messages', Frame, queue_size=10)

        self.service_set_angle = rospy.Service(
            'send_joint_cmd', SendJointPos, self.send_joint_pos)
        self.service_set_velocity = rospy.Service(
            'send_joint_speed_cmd', SendJointSpeed, self.send_joint_speed_cmd)

        self.set_hyperparams()



if __name__ == "__main__":
    try:
        rosnode = GimbalController()
        rosnode.ros_init()
    except rospy.ROSInterruptException:
        pass