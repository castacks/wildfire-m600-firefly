cmake_minimum_required(VERSION 2.8.3)
project(core_pointcloud_map_representation)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  nav_msgs
  roscpp
  rospy
  sensor_msgs
  std_msgs
  tf
  visualization_msgs
  core_map_representation_interface
  pcl_ros
  pcl_conversions
  tflib
  pluginlib
  core_trajectory_msgs
  )

find_package(PCL REQUIRED)

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS rospy roscpp tf core_map_representation_interface pcl_ros pcl_conversions
  LIBRARIES pointcloud_map_representation
)


include_directories(${catkin_INCLUDE_DIRS} include ${PCL_INCLUDE_DIRS})

add_library(pointcloud_map_representation
  src/pointcloud_map_representation.cpp
  )

target_link_libraries(pointcloud_map_representation ${catkin_LIBRARIES} ${base_LIBRARIES} ${PCL_LIBRARIES})


install(TARGETS pointcloud_map_representation
        ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
        RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

