cmake_minimum_required(VERSION 3.0.2)
project(perception)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  depth_image_proc
  octomap_server
)

catkin_package(
  # INCLUDE_DIRS include
  # LIBRARIES perception
  CATKIN_DEPENDS roscpp rospy std_msgs depth_image_proc octomap_server
  # DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)


