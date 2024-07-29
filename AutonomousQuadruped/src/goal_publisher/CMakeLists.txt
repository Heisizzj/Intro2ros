cmake_minimum_required(VERSION 3.0.2)
project(goal_publisher)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  actionlib
  move_base_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs actionlib move_base_msgs
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(goal_publisher_node src/goal_publisher_node.cpp)

target_link_libraries(goal_publisher_node
  ${catkin_LIBRARIES}
)
