cmake_minimum_required(VERSION 3.0.2)
project(firefly_telemetry)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
)

catkin_package()

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
  src/gcs_telemetry.py
  src/onboard_telemetry.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

