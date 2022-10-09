cmake_minimum_required(VERSION 3.0.2)
project(dji_rs3pro_ros_controller)


find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  nav_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  socketcan_bridge
  std_msgs
  tf
  tf2
)

add_service_files(DIRECTORY srv
  FILES
  SendJointPos.srv
  SendJointSpeed.srv
  AddTwolnts.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES dji_rs3pro_ros_controller
  CATKIN_DEPENDS cv_bridge image_transport message_generation nav_msgs nav_msgs roscpp rospy sensor_msgs socketcan_bridge std_msgs tf tf2
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)



catkin_install_python(PROGRAMS
  scripts/can_bus_controller.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)