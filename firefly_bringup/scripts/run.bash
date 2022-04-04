#!/bin/bash
# tip: Make this file a shortcut, e.g. Ctrl-Alt-R
set -e

script_name=$0
script_full_path=$(dirname "$0")

trap 'echo "Killing"; bash "$script_full_path"/stop.bash' INT TERM HUP QUIT

DEFAULT_ROOT="/mnt/nvme0n1/data"

if [ -z "$1" ]; then
	ROOT="$DEFAULT_ROOT"
else
	ROOT="$1"
fi


[ -d "$ROOT" ] && echo "Directory $ROOT exists." || (echo "Error: Directory $ROOT does not exist." && exit 1)

DATETIME=$(date +"%Y-%m-%d_%H-%M-%S")
DATE=$(date +"%Y-%m-%d")
OUT_FOLDER="$ROOT"/"$DATE"
OUTPUT="$OUT_FOLDER"/"$DATETIME"_both
mkdir -p "$OUT_FOLDER"
echo "Saving to $OUTPUT ..."
sleep 2

# Run DJI SDK
# source /home/wildfire/M600_ws/devel/setup.bash && nohup roslaunch dji_sdk sdk.launch &
sleep 4

# run RGB
# gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 ! tee name=t \
# t. ! queue ! "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=60/1" ! nvvidconv flip-method=2 ! nvv4l2h264enc bitrate=20000000 ! h264parse ! mp4mux ! filesink location="$OUTPUT"_rgb.mp4 \
# t. ! queue ! nvvidconv flip-method=2 ! xvimagesink &
gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 !\
queue ! "video/x-raw(memory:NVMM),width=1920,height=1080,framerate=60/1" ! nvvidconv flip-method=2 ! nvv4l2h264enc bitrate=20000000 ! h264parse ! mp4mux ! filesink location="$OUTPUT"_rgb.mp4 &

# run Mavlink
#python3 ~/M600_ws/src/data_collection/ros_mavlink_test.py &

# run thermal
# source /home/wildfire/M600_ws/devel/setup.bash && roslaunch flir_ros_sync flir_ros.launch &
#source /home/wildfire/M600_ws/devel/setup.bash && roslaunch flir_ros_sync flir_ros.launch &

#source /home/wildfire/M600_ws/devel/setup.bash && roslaunch seek_driver seek_driver.launch &

# run ROS record
if rostopic list | grep -q "/rosout"; then
	source /home/wildfire/M600_ws/devel/setup.bash
	rosbag record -a -O "$OUT_FOLDER"/"$DATETIME"_dji_sdk_and_thermal.bag __name:="data_collect" -x "(.*)/compressed(.*)|(.*)/theora(.*)" &
else
	echo "roscore not running, not recording DJI SDK data"
fi


wait
