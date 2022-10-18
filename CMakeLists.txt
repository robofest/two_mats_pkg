cmake_minimum_required(VERSION 3.0.2)
project(two_mats_pkg)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  cv_bridge
  image_transport
  sensor_msgs
  dynamic_reconfigure
)

find_package(OpenCV REQUIRED)
generate_dynamic_reconfigure_options(
  cfg/TwoMats.cfg
)

catkin_package(
    CATKIN_DEPENDS roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include
)
