cmake_minimum_required(VERSION 3.0.2)
project(firefly_control)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++17)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  dji_sdk
  sensor_msgs
  behavior_tree
  behavior_tree_msgs
  core_takeoff_landing_planner
  core_trajectory_controller
  core_drone_interface
  core_trajectory_msgs
)

find_package(base)

find_package(DJIOSDK REQUIRED)


catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
 include
  ${catkin_INCLUDE_DIRS}
  ${base_INCLUDE_DIRS}
)


add_executable(waypoint_mission src/waypoint_mission.cpp)
add_dependencies(waypoint_mission ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(waypoint_mission ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})

add_executable(takeoff_land_test src/takeoff_land_test.cpp)
add_dependencies(takeoff_land_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(takeoff_land_test ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})

add_executable(simple_waypoint_test src/simple_waypoint_test.cpp)
add_dependencies(simple_waypoint_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(simple_waypoint_test ${catkin_LIBRARIES} ${DJIOSDK_LIBRARIES})

add_executable(behavior_executive src/behavior_executive.cpp)
add_dependencies(behavior_executive ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS} ${base_EXPORTED_TARGETS})
target_link_libraries(behavior_executive
  ${catkin_LIBRARIES}
  ${base_LIBRARIES}
)
