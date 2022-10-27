#!/bin/bash

image_name='dji_rs3pro_ros_controller'
image_tag_gimbal='gimbal'
image_tag_IMU='imu'
image_tag_Realsense='realsense'

docker build -t $image_name:$image_tag_gimbal -f Dockerfile_DJI .
docker build -t $image_name:$image_tag_IMU -f Dockerfile_IMU .
docker build -t $image_name:$image_tag_Realsense -f Dockerfile_Realsense .