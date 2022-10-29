#!/bin/bash

FILE=$(pwd)/pymavlink/

if ! git diff --quiet $FILE; then
    cd "pymavlink/"
    MDEF=$(pwd)/message_definitions python3 -m pip install . -v --upgrade
fi

catkin build -DCMAKE_BUILD_TYPE=Release