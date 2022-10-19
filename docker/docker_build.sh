#!/bin/bash

image_name='dji_rs3pro_ros_controller'
image_tag='noetic'

docker build -t $image_name:$image_tag .