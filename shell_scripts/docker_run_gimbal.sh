#!/bin/bash
image_name='dji_rs3pro_ros_controller'
image_tag='gimbal'

script_dir=$(cd $(dirname $0); pwd)
ssd_path='/media/amsl/96fde31e-3b9b-4160-8d8a-a4b913579ca21/'

docker run -it --rm \
    --net="host" \
    --gpus all \
    --name $image_tag \
    --shm-size=1024m \
    --privileged \
    --volume="$script_dir/:/home/ros_catkin_ws/src/$image_name" \
    --volume="$ssd_path:/home/ssd_dir/" \
    --device=/dev/ttyUSB0:/dev/ttyUSB0 \
    --device=/dev/ttyUSB1:/dev/ttyUSB1 \
    --device=/dev/ttyUSB2:/dev/ttyUSB2 \
    --device=/dev/ttyUSB3:/dev/ttyUSB3 \
    --device=/dev/ttyUSB4:/dev/ttyUSB4 \
    $image_name:$image_tag \
    bash -c "catkin_make && slcand -o -s8 -t hw -S 1000000 /dev/ttyUSB0 && ip link set up slcan0 && bash"

    