#!/bin/bash
set -e

gst-launch-1.0 -e nvarguscamerasrc sensor-id=0 ! tee name=t \
t. ! queue ! "video/x-raw(memory:NVMM),width=1300,height=750,framerate=40/1" ! nvvidconv flip-method=2 ! xvimagesink
