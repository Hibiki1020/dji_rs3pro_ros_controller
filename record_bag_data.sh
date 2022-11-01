#!/bin/bash

jst_ymd_today=$(TZ=UTC-9 date '+%Y%m%d_%H%M%S')
bag_name="dji_rs3pro_"$jst_ymd_today

rosbag record /camera/color/camera_info \
            /camera/color/image_raw/compressed \
            /camera/depth/camera_info \
            /camera/depth/image_rect_raw/compressed \
            /imu/acceleration \
            /imu/angular_velocity \
            /imu/data \
            /imu/dq \
            /imu/dv \
            /imu/mag \
            /imu/time_ref \
            /initialpose \
            /move_base_simple/goal \
            /pressure \
            /rosout \
            /rosout_agg \
            /temperature \
            /tf \
            /tf_static \
            /imu_correct_angle \
            /gimbal_angle \
            received_message \
            -O /home/amsl/bagfiles/$bag_name.bag

