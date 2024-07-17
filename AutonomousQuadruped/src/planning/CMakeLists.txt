cmake_minimum_required(VERSION 3.0.2)
project(planning)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  eigen_conversions
  geometry_msgs
  mav_msgs
  roscpp
  rospy
  std_msgs
  
)

catkin_package(CATKIN_DEPENDS 
  roscpp 
  std_msgs 
  mav_msgs 
  eigen_conversions
  LinearMath
)

find_package(Eigen3)

include_directories(${catkin_INCLUDE_DIRS})

add_executable(planning_node src/planning_node.cpp)
add_dependencies(planning_node ${catkin_EXPORTED_TARGETS})

target_link_libraries(planning_node ${catkin_LIBRARIES})