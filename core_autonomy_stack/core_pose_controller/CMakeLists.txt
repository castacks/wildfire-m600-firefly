cmake_minimum_required(VERSION 3.0.2)
project(core_pose_controller)


find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_generation
  nav_msgs
  core_pid_controller
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(base REQUIRED)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")


catkin_package(
  INCLUDE_DIRS include
#  CATKIN_DEPENDS rospy roscpp tf dji_sdk message_runtime
)

include_directories(${catkin_INCLUDE_DIRS} include ${base_INCLUDE_DIRS})

add_executable(pose_controller
  src/pose_controller.cpp
)
add_dependencies(pose_controller ${catkin_EXPORTED_TARGETS} ${base_EXPORTED_TARGETS})
target_link_libraries(pose_controller ${catkin_LIBRARIES} ${base_LIBRARIES})
