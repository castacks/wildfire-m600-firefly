#!/bin/bash
set -e

ROOT="/mnt/nvme0n1/data"
DATETIME=$(date +"%Y-%m-%d_%H-%M-%S")
DATE=$(date +"%Y-%m-%d")
OUT_FOLDER="$ROOT"/"$DATE"
OUTPUT="$OUT_FOLDER"/"$DATETIME"
mkdir -p "$OUT_FOLDER"
echo "Saving to $OUTPUT ..."
sleep 2

gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 ! tee name=t \
t. ! queue ! "video/x-raw(memory:NVMM),width=1300,height=750,framerate=40/1" ! nvvidconv flip-method=2 ! nvv4l2h264enc bitrate=20000000 ! h264parse ! mp4mux ! filesink location="$OUTPUT"_rgb.mp4 \
t. ! queue ! nvvidconv flip-method=2 ! xvimagesink
