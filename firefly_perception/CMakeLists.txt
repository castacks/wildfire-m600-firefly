cmake_minimum_required(VERSION 3.0.2)
project(firefly_perception)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  firefly_mapping
  image_transport
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
)
find_package(
  OpenCV 3.2 
)
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES firefly_perception
#  CATKIN_DEPENDS OpenCV cv_bridge firefly_mapping image_transport roscpp rospy sensor_msgs std_msgs
#  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
add_executable(firefly_perception src/firefly_perception.cpp)
target_link_libraries(firefly_perception
                            ${catkin_LIBRARIES}
                            ${OpenCV_LIBRARIES})
add_executable(firefly_perception_continuous src/firefly_perception_continuous.cpp)
target_link_libraries(firefly_perception_continuous
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES})
