cmake_minimum_required(VERSION 3.0.2)
project(firefly_mapping)

add_compile_options(-std=c++17)

set(PCL_DIR /usr/lib/${CMAKE_HOST_SYSTEM_PROCESSOR}-linux-gnu/cmake/pcl/)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  geometry_msgs
  nav_msgs
  tf
  eigen_conversions
  grid_map_core
  grid_map_msgs
  grid_map_ros
  grid_map_rviz_plugin
  grid_map_sdf
  grid_map_visualization
  PCL
  pcl_conversions
  pcl_ros
)

add_message_files(
        FILES
        ImageWithPose.msg
)
generate_messages(
        DEPENDENCIES
        sensor_msgs
        geometry_msgs
        std_msgs
)

catkin_package(
        CATKIN_DEPENDS roscpp rospy sensor_msgs geometry_msgs message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)

catkin_install_python(PROGRAMS
  src/gps_to_local.py
  scripts/mapping_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

add_executable(onboard_mapping src/onboard_mapping.cpp)
add_dependencies(onboard_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(onboard_mapping ${catkin_LIBRARIES})

add_executable(onboard_mapping_test src/onboard_mapping_test.cpp)
add_dependencies(onboard_mapping_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(onboard_mapping_test ${catkin_LIBRARIES})

add_executable(gcs_mapping src/gcs_mapping.cpp)
add_dependencies(gcs_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(gcs_mapping ${catkin_LIBRARIES})


add_executable(mapping_accuracy src/mapping_accuracy.cpp)
add_dependencies(mapping_accuracy ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(mapping_accuracy ${catkin_LIBRARIES})

add_executable(terrain_mapping src/terrain_mapping.cpp)
add_dependencies(terrain_mapping ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(terrain_mapping ${catkin_LIBRARIES})
