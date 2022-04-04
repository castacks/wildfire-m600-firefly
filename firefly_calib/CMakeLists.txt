cmake_minimum_required(VERSION 3.0.2)
project(firefly_calib)

find_package(catkin REQUIRED)
find_package(roscpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(std_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(OpenCV REQUIRED)

catkin_package()

include_directories(
        ${catkin_INCLUDE_DIRS}
        ${roscpp_INCLUDE_DIRS}
        ${sensor_msgs_INCLUDE_DIRS}
        ${cv_bridge_INCLUDE_DIRS}
        ${std_msgs_INCLUDE_DIRS}
        ${image_transport_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)

add_executable(detect_chessboard src/detect_chessboard.cpp)
target_link_libraries(detect_chessboard
                            ${catkin_LIBRARIES}
                            ${OpenCV_LIBRARIES}
                            ${roscpp_LIBRARIES}
                            ${sensor_msgs_LIBRARIES}
                            ${cv_bridge_LIBRARIES}
                            ${std_msgs_LIBRARIES}
                            ${image_transport_LIBRARIES})

add_executable(intrinsic_calibration src/intrinsic_calibration.cpp)
target_link_libraries(intrinsic_calibration
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        ${roscpp_LIBRARIES}
        ${sensor_msgs_LIBRARIES}
        ${cv_bridge_LIBRARIES}
        ${std_msgs_LIBRARIES}
        ${image_transport_LIBRARIES})



