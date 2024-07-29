cmake_minimum_required(VERSION 3.0.2)
project(path_height_checker)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  geometry_msgs
  nav_msgs
  octomap_msgs
  message_generation
)

find_package(octomap REQUIRED)

add_message_files(
  FILES
  PathHeightStatus.msg
  PathHeightStatusArray.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
  nav_msgs
)

catkin_package(
  CATKIN_DEPENDS roscpp std_msgs geometry_msgs nav_msgs octomap_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_executable(path_height_checker_node src/path_height_checker_node.cpp)

add_dependencies(path_height_checker_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(path_height_checker_node
  ${catkin_LIBRARIES}
  ${OCTOMAP_LIBRARIES}
)
