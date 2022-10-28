cmake_minimum_required(VERSION 3.0.2)
project(core_pid_controller)

find_package(catkin REQUIRED COMPONENTS
  dynamic_reconfigure
  geometry_msgs
  message_generation
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)

find_package(base)

SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

generate_dynamic_reconfigure_options(
  cfg/PID.cfg
)

add_message_files(
  FILES
  PIDInfo.msg
)

generate_messages(
  DEPENDENCIES std_msgs geometry_msgs
)

catkin_package(
  INCLUDE_DIRS include
  msg
  CATKIN_DEPENDS rospy roscpp tf message_runtime
  LIBRARIES pid_controller
)

include_directories(${catkin_INCLUDE_DIRS} include ${base_INCLUDE_DIRS})


add_library(pid_controller
  src/pid_controller.cpp
)
add_dependencies(pid_controller ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${PROJECT_NAME}_gencfg ${base_EXPORTED_TARGETS})
target_link_libraries(pid_controller ${catkin_LIBRARIES} ${base_LIBRARIES})


install(TARGETS pid_controller
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

