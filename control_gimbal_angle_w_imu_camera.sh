#!/bin/bash
script_dir=$(cd $(dirname $0); pwd)


gnome-terminal --tab -e "bash -c '$script_dir/shell_scripts/docker_run_realsense.sh'"
sleep 5s

gnome-terminal --tab -e "bash -c '$script_dir/shell_scripts/docker_run_imu.sh'"
sleep 5s

gnome-terminal --tab -e "bash -c '$script_dir/shell_scripts/docker_run_gimbal.sh'"