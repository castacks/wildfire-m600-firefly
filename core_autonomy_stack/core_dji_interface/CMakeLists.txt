cmake_minimum_required(VERSION 2.8.3)
project(core_dji_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  core_drone_interface
  pluginlib
  dji_sdk
  tf
  mav_msgs
)

find_package(base)

catkin_package()

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${base_INCLUDE_DIRS}
  include
)

add_library(dji_interface src/dji_interface.cpp)

target_link_libraries(dji_interface
  ${catkin_LIBRARIES}
  ${base_LIBRARIES}
)
