#!/bin/bash
# tip: Make this file a shortcut, e.g. Ctrl-Alt-R
set -e

script_name=$0
script_full_path=$(dirname "$0")

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

rosbag record -a -O "$OUT_FOLDER"/"$DATETIME"_dji_sdk_and_thermal.bag __name:="data_collect" -x "(.*)/compressed(.*)|(.*)/theora(.*)"
