cmake_minimum_required(VERSION 3.0.2)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  nodelet
  octomap_server
  roscpp
  std_msgs
  tf2_ros
  sensor_msgs
  geometry_msgs
  tf
  depth_image_proc
)
catkin_package(
CATKIN_DEPENDS nodelet octomap_server
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)