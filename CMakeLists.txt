cmake_minimum_required(VERSION 3.0.2)
project(vaccum)

## Compile as C++11 (safe even if not using C++)
add_compile_options(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  move_base_msgs
  actionlib
  tf
  cv_bridge
  image_transport
)

## Catkin package definition
catkin_package(
  CATKIN_DEPENDS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    move_base_msgs
    actionlib
    tf
    cv_bridge
    image_transport
)

## Include directories (mostly for completeness)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Install Python scripts
catkin_install_python(
  PROGRAMS
    src/task_manager.py
    src/qr_reader.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

