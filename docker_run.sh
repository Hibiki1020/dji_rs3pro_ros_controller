#!/bin/bash
image_name='dji_rs3pro_ros_controller'
image_tag='noetic'

script_dir=$(cd $(dirname $0); pwd)
ssd_path='/media/amsl/96fde31e-3b9b-4160-8d8a-a4b913579ca21/'

docker run -it --rm \
    --net="host" \
    --gpus all \
    --name $image_name \
    --shm-size=4096m \
    --privileged \
    --volume="$script_dir/:/home/ros_catkin_ws/src/$image_name" \
    --volume="$ssd_path:/home/ssd_dir/" \
    --device=/dev/ttyUSB0:/dev/ttyUSB0 \
    --device=/dev/ttyUSB1:/dev/ttyUSB1 \
    --device=/dev/ttyUSB2:/dev/ttyUSB2 \
    $image_name:$image_tag \
    bash -c "catkin_make && bash"