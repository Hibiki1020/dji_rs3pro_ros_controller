#!/usr/bin/env python3
#データは全てラジアンの形式で保存

import csv
import re
import struct
from ctypes import *
import numpy as np
import math
import argparse
import yaml
import random
import ctypes as ct
import cv2

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
from cv_bridge import CvBridge, CvBridgeError

#Need in running in ROS
os.chdir(os.path.dirname(os.path.abspath(__file__)))
sys.path.append('../')
from scripts.check_sum import *
from scripts.can_bus_controller import *
from dji_rs3pro_ros_controller.srv import *
from dji_rs3pro_ros_controller.msg import *

class ROSBag2Dataset:
    def __init__(self, CFG):
        self.CFG = CFG
        self.save_data_directory = str(CFG["save_data_directory"])
        self.csv_name = str(CFG["csv_name"])
        
        self.gimbal_eular_angle_topic_name = str(CFG["gimbal_eular_angle_topic_name"])
        self.imu_eular_angle_topic_name = str(CFG["imu_eular_angle_topic_name"])
        self.decompressed_image_topic_name = str(CFG["decompressed_image_topic_name"])
        self.imu_data_topic_name = str(CFG["imu_data_topic_name"])

        self.counter = 0
        self.save_data_checker_gimbal = False
        self.save_data_checker_imu = False
        self.save_data_checker_image = False

        self.imu_data = Imu()
        #Angle Data
        self.gimbal_eular_angle = EularAngle()
        self.imu_eular_angle = EularAngle()
        self.integrated_angle = EularAngle()
        self.previous_angle = EularAngle()
        self.csv_row = []
        #Image
        self.decompressed_image = Image()
        
        #OpenCV
        self.bridge = CvBridge()
        self.color_img_cv = np.empty(0)

    def init_node(self):
        rospy.init_node('rosbag2dataset', anonymous=True)
        self.rate = rospy.Rate(25) # 25hz
        self.init_time = rospy.Time().now()

        self.sub_imu_data = rospy.Subscriber(self.imu_data_topic_name, Imu, self.imu_data_callback)
        self.sub_gimbal_eular_angle = rospy.Subscriber(self.gimbal_eular_angle_topic_name, EularAngle, self.gimbal_eular_angle_callback)
        self.sub_imu_eular_angle = rospy.Subscriber(self.imu_eular_angle_topic_name, EularAngle, self.imu_eular_angle_callback)
        self.sub_decompressed_image = rospy.Subscriber(self.decompressed_image_topic_name, Image, self.decompressed_image_callback)


    def imu_data_callback(self, msg):
        self.imu_data = msg

    def gimbal_eular_angle_callback(self, msg):
        self.previous_angle = self.gimbal_eular_angle
        self.gimbal_eular_angle = msg
        self.process_time = rospy.Time().now() - self.init_time

        self.integrated_angle.roll = (self.gimbal_eular_angle.roll + self.imu_eular_angle.roll) / 2
        self.integrated_angle.pitch = (self.gimbal_eular_angle.pitch + self.imu_eular_angle.pitch) / 2
        self.integrated_angle.yaw = (self.gimbal_eular_angle.yaw + self.imu_eular_angle.yaw) / 2
        
        self.save_data_checker_gimbal = True
        

    def imu_eular_angle_callback(self, msg):
        self.imu_eular_angle = msg

        self.save_data_checker_imu = True

    def decompressed_image_callback(self, msg):
        self.decompressed_image = msg
        try:
            self.color_img_cv = self.bridge.imgmsg_to_cv2(self.decompressed_image, "bgr8")
        except CvBridgeError as e:
            print(e)

        original_col = int(self.color_img_cv.shape[1]) #col"
        original_row = int(self.color_img_cv.shape[0]) #row

        #print(original_col, original_row)

        init_col = (original_col-original_row)//2
        #print(init_col)

        self.cropped_image = self.color_img_cv[0:original_row, init_col:init_col+original_row]
        self.cropped_image = cv2.resize(self.cropped_image, dsize=(224, 224), interpolation=cv2.INTER_AREA)

        # print(self.cropped_image.shape)

        # cv2.imshow("image", self.cropped_image)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        self.save_data_checker_image = True

    def spin_node(self):
        while not rospy.is_shutdown():
            self.rate.sleep()

            if self.save_data_checker_image == True and self.save_data_checker_gimbal == True and self.save_data_checker_imu == True:
                self.save_data()

                print("Count:", self.counter)
                print("Process Time:", self.process_time)
                print("gimbal_angle roll      : {:4f}, pitch: {:4f}, yaw: {:4f}".format(self.gimbal_eular_angle.roll, self.gimbal_eular_angle.pitch, self.gimbal_eular_angle.yaw))
                print("imu_eular_angle roll   : {:4f}, pitch: {:4f}, yaw: {:4f}".format(self.imu_eular_angle.roll, self.imu_eular_angle.pitch, self.imu_eular_angle.yaw))
                print("integrated_angle roll  : {:4f}, pitch: {:4f}, yaw: {:4f}".format(self.integrated_angle.roll, self.integrated_angle.pitch, self.integrated_angle.yaw))
                print("(Degree) Roll diff:{:4f}, Pitch diff:{:4f}, Yaw diff:{:4f}".format((self.gimbal_eular_angle.roll - self.imu_eular_angle.roll)/math.pi*180.0, (self.gimbal_eular_angle.pitch - self.imu_eular_angle.pitch)/math.pi*180.0, (self.gimbal_eular_angle.yaw - self.imu_eular_angle.yaw)/math.pi*180.0))
                print("\n")
                self.counter += 1

        rospy.spin()

    def save_data(self):
        # Save Image
        image_name = "image" + str(self.counter) + ".png"
        image_path = self.save_data_directory + "/camera_image/" + image_name
        cv2.imwrite(image_path, self.cropped_image)

        # Save CSV
        csv_file_path = os.path.join(self.save_data_directory, self.csv_name)
        csv_write = open(csv_file_path, 'a')
        csv_w = csv.writer(csv_write)
        csv_row = [str(self.counter), image_name, str(self.process_time), str(self.integrated_angle.roll), str(self.integrated_angle.pitch), str(self.integrated_angle.yaw)]

        csv_w.writerow(csv_row)
        csv_write.close()


if __name__ == '__main__':
    parser = argparse.ArgumentParser("./rosbag2dataset.py")
    parser.add_argument(
        '--cfg', '-c',
        type=str,
        required=False,
        default="../yaml/rosbag2dataset.yaml",
        help="path to config yaml file"
    )

    FLAGS, unparsed = parser.parse_known_args()

    #Load yaml file
    try:
        print("Opening config file %s", FLAGS.cfg)
        CFG = yaml.safe_load(open(FLAGS.cfg, 'r'))
    except Exception as e:
        print(e)
        print("Error opening config file %s", FLAGS.cfg)
        quit()

    try:
        rosnode = ROSBag2Dataset(CFG)
        rosnode.init_node()
        rosnode.spin_node()

    except rospy.ROSInterruptException:
        pass
