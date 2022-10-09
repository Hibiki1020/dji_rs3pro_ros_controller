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

        return_code = int(0.0, base=8)
        limit_pitch_max = int(30.0, base=8)
        limit_pitch_min = int(0.0, base=8)
        limit_yaw_max = int(30.0, base=8)
        limit_yaw_min = int(0.0, base=8)
        limit_roll_max = int(30.0, base=8)
        limit_roll_min = int(0.0, base=8)

        # 1byte -> B 2byte -> H 4byte -> I
        hex_data = struct.pack('<7B', return_code, limit_pitch_max, limit_pitch_min, limit_yaw_max, limit_yaw_min, limit_roll_max, limit_roll_min)
        # To convert like 00 22 11
        pack_data = ['{:02X}'.format(struct.unpack('<1B', i)[0]) for i in hex_data]

        cmd_data = ':'.join(pack_data)
        # print(cmd_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                                    cmd_id='03', data=cmd_data)
        
        self.send_cmd(cmd)
        return True

    def assemble_can_msg(self, cmd_type, cmd_set, cmd_id, data):
        if data == "":
            can_frame_data = "{prefix}" + \
                ":{cmd_set}:{cmd_id}".format(
                    cmd_set=cmd_set, cmd_id=cmd_id)
        else:
            can_frame_data = "{prefix}" + ":{cmd_set}:{cmd_id}:{data}".format(
                cmd_set=cmd_set, cmd_id=cmd_id, data=data)

        cmd_length = len(can_frame_data.split(":")) + 15

        seqnum = self.seq_num()
        # ic(seqnum)
        can_frame_header = "{header:02x}".format(
            header=self.header)  # SOF byte
        can_frame_header += ":" + \
            ("%04x" % (cmd_length))[2:4]  # 1st length byte
        can_frame_header += ":" + \
            ("%04x" % (cmd_length))[0:2]  # 2nd length byte
        can_frame_header += ":" + \
            "{cmd_type}".format(cmd_type=cmd_type)  # Command Type
        can_frame_header += ":" + "{enc:02x}".format(enc=self.enc)  # Encoding
        can_frame_header += ":" + \
            "{res1:02x}".format(res1=self.res1)  # Reserved 1
        can_frame_header += ":" + \
            "{res2:02x}".format(res2=self.res2)  # Reserved 2
        can_frame_header += ":" + \
            "{res3:02x}".format(res3=self.res3)  # Reserved 3
        can_frame_header += ":" + seqnum    # Sequence number
        can_frame_header += ":" + calc_crc16(can_frame_header)

        # hex_seq = [eval("0x" + hex_num) for hex_num in can_frame_header.split(":")]

        whole_can_frame = can_frame_data.format(prefix=can_frame_header)
        whole_can_frame += ":" + calc_crc32(whole_can_frame)
        whole_can_frame = whole_can_frame.upper()
        #
        # print("Header: ", can_frame_header)
        # print("Total: ", whole_can_frame)
        return whole_can_frame

    def send_cmd(self, cmd):
        data = [int(i, 16) for i in cmd.split(":")]
        # print(data)
        self.send_data(self.send_id, data)

    def send_data(self, can_id, data):
        data_len = len(data)
        full_frame_num, left_len = divmod(data_len, self.FRAME_LEN)

        if left_len == 0:
            frame_num = full_frame_num
        else:
            frame_num = full_frame_num + 1

        # send_buf = (VCI_CAN_OBJ * (frame_num))()

        # msg = Frame()
        # msg.id = 0  # uint32
        # msg.is_rtr = False  # bool
        # msg.is_extended = False  # bool
        # msg.is_error = False  # bool
        # msg.dlc = 8  # uint8
        # msg.data = [0, 0, 0, 0, 0, 0, 0, 0]  # uint8[8]
        send_msg_buffer = []

        data_offset = 0
        for i in range(full_frame_num):
            msg = Frame()
            msg.id = can_id
            msg.is_rtr = False
            msg.is_extended = False
            msg.is_error = False
            msg.dlc = 8
            msg.data = [0, 0, 0, 0, 0, 0, 0, 0]

            for j in range(self.FRAME_LEN):
                msg.data[j] = data[data_offset + j]
            data_offset += self.FRAME_LEN
            send_msg_buffer.append(msg)

        # If there is data left over, the last frame isn't 8byte long

        if left_len > 0:
            msg = Frame()
            msg.id = can_id
            msg.is_rtr = False
            msg.is_extended = False
            msg.is_error = False
            msg.dlc = left_len
            msg.data = [0, 0, 0, 0, 0, 0, 0, 0]

            for j in range(left_len):
                msg.data[j] = data[data_offset + j]
            data_offset += self.FRAME_LEN
            send_msg_buffer.append(msg)

        # print(send_msg_buffer)
        for msg in send_msg_buffer:
            self.pub_can_command.publish(msg)

    def request_current_position(self):
        hex_data = [0x01]
        pack_data = ['{:02X}'.format(i)
                     for i in hex_data]
        cmd_data = ':'.join(pack_data)
        cmd = self.assemble_can_msg(cmd_type='03', cmd_set='0E',
                                    cmd_id='02', data=cmd_data)
        # print(cmd)
        # print("Cmd_data" + cmd)
        self.send_cmd(cmd)

    def print_current_position(self):
        print("Current position: yaw: {yaw}, pitch: {pitch}, roll: {roll}".format(self.yaw, self.pitch, self.roll))

    def ros_init(self):
        rospy.init_node('RS3Pro_Gimbal_Controller', anonymous=True)
        self.rate = rospy.Rate(10) # 10hz
        self.br = tf.TransformBroadcaster()

        self.sub_can_data = rospy.Subscriber('/gimbal_data', Frame, self.can_callback)
        self.pub_can_command = rospy.Publisher('/sent_messages', Frame, queue_size=10)

        self.service_set_angle = rospy.Service(
            'send_joint_cmd', SendJointPos, self.send_joint_pos)
        self.service_set_velocity = rospy.Service(
            'send_joint_speed_cmd', SendJointSpeed, self.send_joint_speed_cmd)

        self.set_hyperparams()

    def ros_spin(self):

        while not rospy.is_shutdown():
            self.request_current_position()
            self.print_current_position()
            self.rate.sleep()
            rospy.spin()




if __name__ == "__main__":
    try:
        rosnode = GimbalController()
        rosnode.ros_init()
        rosnode.ros_spin()
    except rospy.ROSInterruptException:
        pass