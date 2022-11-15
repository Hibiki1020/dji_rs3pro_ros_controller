# dji_rs3pro_ros_controller

This program is based on the [ROS controller for DJI RS2](https://github.com/ceinem/dji_rs2_ros_controller) developed by ceinem and [DJI R SDK demo software](https://terra-1-g.djicdn.com/851d20f7b9f64838a34cd02351370894/DJI%20R%20SDK/SDK%20demo%20software.zip).

## System Requirement
* Ubuntu 20.04
* Python3
* ROS Noetic
* gnome-terminal
* Docker

## Hardware Requirement
* DJI RS3 Pro
* DJI Focus Wheel
* Camera (weight 700g or more)
* Cable
* CAN-USB Converter
* PC

## Install

### Setup ROS, catkin workspace and system dependencies
Please install ROS **noetic** and setup your catkin workspace according to the official ROS tutorials.

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

## Hardware setup

### Gimbal
DJI RS3 Pro is used in this method, but you can use other DJI's gimbal such as RS2 as long as DJI doesn't change communication procotol.

At a minimum, the following programs are required to run this program.
* Gimbal
* Smartphone(Android or iPhone) and install DJI's ronin app.
* [DJI Focus Wheel](https://m.dji.com/jp/product/ronin-s-focus-wheel)
* CAN-USB Converter
* Home-made Cable
* PC(Ubuntu 20.04)

### DJI Focus Wheel
This equipment is always required to connect the gimbal to the PC. A switch exists on the focus wheel to switch between CAN and S-BUS, but this time, **switch to CAN**.

### CAN-USB
This time I used [Lawicel's CANUSB](https://www.canusb.com/products/canusb/) as a converter between CAN and USB. Since the USB terminal of this CANUSB is **USB Type B**, a conversion cable to A or C from B is also required.

### Home-made Cable
A home-made cable is required for communication between the focus wheel and the CAN-USB converter, using a 9-pin D-sub terminal (jack) on the CAN-USB converter side and a 4-pin Dupont connector on the focus wheel side. Refer to the following image for cable wiring.

![配線図 (1)](https://user-images.githubusercontent.com/60866340/195969835-803b98da-7b6d-40a7-9d98-76737a79dbaa.png)

### IMU
In this method, I used [MTi-30](https://www.xsens.com/products/mti-10-series?utm_term=xsens%20mti30&utm_medium=ppc&utm_campaign=3DCA+%7C+North+Amer+%7C+Search&utm_source=adwords&hsa_cam=15267582124&hsa_src=g&hsa_mt=e&hsa_ver=3&hsa_net=adwords&hsa_tgt=aud-1596292619963:kwd-1000087377016&hsa_acc=1306794700&hsa_grp=132748046794&hsa_kw=xsens%20mti30&hsa_ad=561599686314&gclid=CjwKCAjwkaSaBhA4EiwALBgQaJQrnVGFkerrfFy4r0kFn3oYyCk_727Y4xqZO1D1FVAVU8B11mbE-hoCY3EQAvD_BwE) for compensate gimbal's low frequency.

Download [software from xsens's site](https://www.xsens.com/software-downloads)

```
cd Downloads/
sudo apt-get install sharutils
tar -xvf MT_Software_Suite_linux-x86_2021.4.tar.gz
sudo ./mtsdk_linux-x86_2021.4.sh 
cp -r /usr/local/xsens/xsens_ros_mti_driver/ ~/catkin_ws/src/
cd ~/catkin_ws
pushd src/xsens_ros_mti_driver/lib/xspublic && make && popd
catkin build (or catkin_make)
```

### Camera
I used a Realsense D435 as my camera. I also used aluminum weights for IMU mounting and to ensure weight balance.

**note**: When a light object such as a webcam is mounted on a gimbal, excessive torque or angular velocity may be applied to the gimbal, possibly damaging the main unit. Therefore, it is advisable to create a balance weight or the like to secure a certain amount of weight. In my case, I use an aluminum weight weighing about 700 grams.

## Run
For the CANusb bring up the can interface using. 

**When inserting the gimbal USB, insert it before the IMU or camera USB**.
If the IMU or camera USB is inserted first, the USB number may be USB1 or USB2.:
```bash
sudo slcand -o -s8 -t hw -S 1000000 /dev/ttyUSB0
sudo ip link set up slcan0
```
When using the CAN-USB converter described in the Hardware Setup section, only the green LED on the converter will light up constantly if communication between the gimbal and the PC is working properly.

Make sure to always specify the correct USB-port.

Launch the ROS-node using
```
roslaunch dji_rs3pro_ros_controller control_gimbal_angle.launch
```

Launch the all equipment using docker
```
./control_gimbal_angle_w_imu_camera.sh

# In gimbal's docker
roslaunch dji_rs3pro_ros_controller control_gimbal_angle.launch
```

If you want to record bagdata, using
```
./record_bag_data.sh
```
Make sure to specify the correct CAN-bus in the launch file and check yaml file.
