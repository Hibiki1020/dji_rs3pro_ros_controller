#!/bin/bash
gnome-terminal --tab -e "bash -c 'roslaunch xsens_mti_driver xsens_mti_node.launch'"
sleep 2s

gnome-terminal --tab -e "bash -c 'roslaunch realsense2_camera rs_camera.launch'"
sleep 2s

gnome-terminal --tab -e "bash -c 'roslaunch dji_rs3pro_ros_controller control_gimbal_angle.launch'"