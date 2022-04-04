#!/bin/bash
source /opt/ros/melodic/setup.bash

echo "pkill gst-launch-1.0"
pkill -f -INT "gst-launch-1.0"
echo "kill node"
rosnode kill "data_collect" || echo "/data_collect rosbag record node not running, nothing to kill"
sleep 1
rosnode kill "flir_nodelet" || echo "/flir_nodelet_1 node not running, nothing to kill"
# rosnode kill "flir_nodelet_2" || echo "/flir_nodelet_2 node not running, nothing to kill"
sleep 1
rosnode kill "flir_nodelet_manager" || echo "/flir_nodelet_manager node not running, nothing to kill"
sleep 2
rosnode kill "seek_driver" || echo "/seek_driver node not running, nothing to kill"
sleep 1
rosnode kill "dji_sdk" || echo "/dji_sdk node not running, nothing to kill"
sleep 1
echo "pkill heartbeat"
pkill -f "blink_heartbeat.bash"
pkill -f "blink_heartbeat.bash"
echo "pkill mavlink_test.py"
pkill -9 -f mavlink_test.py