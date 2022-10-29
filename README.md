# Project FireFly

# Installation
To build in release mode, run the following command:

    catkin build -DCMAKE_BUILD_TYPE=Release

If you face a build error saying that "common is required but boost was not found", try the following command from https://github.com/PointCloudLibrary/pcl/issues/5052. 

    catkin build -DCMAKE_BUILD_TYPE=Release -DBoost_LIBRARY_DIR_RELEASE=/usr/lib/x86_64-linux-gnu

Note that the sdk.launch file in the dji_sdk package expects the DJI_SDK_APP_ID and DJI_SDK_ENC_KEY to be set.

Install the udev rules by running the following command.

    sudo cp firefly_bringup/99-firefly.rules /etc/udev/rules.d

Install the pymavlink library by running the following commands. Note that MDEF must be an absolute path to the message_definitions folder. You should rerun this command if you make any changes to message_definitions/v1.0/firefly.xml.


    cd pymavlink
    sudo MDEF=$(pwd)/message_definitions python3 -m pip install . -v --upgrade
