# dji_rs3pro_ros_controller


## Install

### Setup ROS, catkin workspace and system dependencies
Please install ROS and setup your catkin workspace according to the official ROS tutorials.

Install additional system dependencies:
```bash
sudo apt install can-utils
sudo apt-get install -y libmuparser-dev
```

### Clone
```bash
cd ~/catkin_ws/src/
git clone https://github.com/Hibiki1020/dji_rs3pro_ros_controller.git --recursive
```

### Hardware setup
工事中

### Run
For the CANusb bring up the can interface using:
```bash
sudo slcand -o -s8 -t hw -S 3000000 /dev/ttyUSB0
sudo ip link set up slcan0
```

For the SLCANUINO bring up the can interface using:
```bash
sudo slcan_attach -f -s6 -o /dev/ttyUSB0
sudo slcand -S **1000000** ttyUSB0 can0
sudo ifconfig can0 up
```

Make sure to always specify the correct USB-port.

Launch the ROS-node using
```
roslaunch ~~~~~
```

Make sure to specify the correct CAN-bus in the launch file.
