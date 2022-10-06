#!/bin/bash
IMAGE_NAME=amsl_realsense_ros
CONTAINER_NAME=realsense_ros
TAG_NAME=noetic

if [ "$(uname -m)" == "aarch64" ]; then
    TAG_NAME=jetson
fi

ROS_MASTER_URI="http://`hostname -I | cut -d' ' -f1`:11311"
ROS_IP=`hostname -I | cut -d' ' -f1`
LAUNCH=rs_camera.launch

if [ ! $# -eq 0 ]; then
    IP_CHECK=$(echo $1 | egrep "^(([1-9]?[0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])\.){3}([1-9]?[0-9]|1[0-9]{2}|2[0-4][0-9]|25[0-5])$")
    if [ "${IP_CHECK}" ]; then
        ROS_MASTER_URI="http://$1:11311"
        if [ $# -ge 2 ]; then
            LAUNCH=${@:2}
        fi
    else
        LAUNCH=$@
    fi
fi

echo "IMAGE_NAME=${IMAGE_NAME}:${TAG_NAME}"
echo "CONTAINER_NAME=${CONTAINER_NAME}"
echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "ROS_IP=${ROS_IP}"
echo "LAUNCH=${LAUNCH}"

docker run -it --rm \
    --privileged \
    --volume="/dev:/dev" \
    --env ROS_MASTER_URI=${ROS_MASTER_URI} \
    --env ROS_IP=${ROS_IP} \
    --net="host" \
    --name ${CONTAINER_NAME} \
    ${IMAGE_NAME}:${TAG_NAME} \
    bash -c "roslaunch realsense2_camera ${LAUNCH}"