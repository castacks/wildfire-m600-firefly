###firefly_calib

This package contains output files and tools related to calibrations performed for the firefly project. The imu_calib folder contains configuration and output files related to the IMU calibration which was performed using this library: https://github.com/ori-drs/allan_variance_ros. The imu_cam_calib folder contains configuration and output files related to the camera-IMU calibration which was performed using this library: https://github.com/ethz-asl/kalibr. This package also contains a detect_chessboard tool which can be used to display chessboard corner detections when capturing data from a thermal camera in order to perform a camera-IMU calibration. 

To run the detect_chessboard tool, first clone this repo into a ROS catkin workspace. Build this package using either catkin_make or catkin build. Then call:

    rosrun firefly_calib detect_chessboard

