cmake_minimum_required(VERSION 3.0.2)
project(core_drone_interface)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  pluginlib
  mav_msgs
  geometry_msgs
  message_generation
)

find_package(base)

add_service_files(
  FILES
  DroneCommand.srv
  )

generate_messages(DEPENDENCIES std_msgs)

catkin_package()


catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp std_msgs
)


include_directories(
  ${catkin_INCLUDE_DIRS}
  ${base_INCLUDE_DIRS}
  include
)

add_executable(drone_interface_node src/drone_interface_node.cpp)

add_dependencies(drone_interface_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${base_EXPORTED_TARGETS})

target_link_libraries(drone_interface_node
  ${catkin_LIBRARIES}
  ${base_LIBRARIES}
)
