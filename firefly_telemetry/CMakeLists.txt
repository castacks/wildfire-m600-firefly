cmake_minimum_required(VERSION 3.0.2)
project(firefly_telemetry)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
  behavior_tree_msgs
)

add_service_files(
  FILES
  SetLocalPosRef.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs  # Or other packages containing msgs
)

ADD_CUSTOM_TARGET(do_always ALL COMMAND ${CMAKE_SOURCE_DIR}/mavlink_dialect/install_pymavlink.sh)

catkin_package(CATKIN_DEPENDS message_runtime)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/gcs_telemetry.py
  src/onboard_telemetry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

