# Project FireFly

# Installation
To build in release mode, run the following command:

    catkin build -DCMAKE_BUILD_TYPE=Release

If you face a build error saying that "common is required but boost was not found", try the following command from https://github.com/PointCloudLibrary/pcl/issues/5052. 

    catkin build -DCMAKE_BUILD_TYPE=Release -DBoost_LIBRARY_DIR_RELEASE=/usr/lib/x86_64-linux-gnu

Note that the sdk.launch file in the dji_sdk package expects the DJI_SDK_APP_ID and DJI_SDK_ENC_KEY to be set.

Install the udev rules on the onboard computer by running the following command.

    sudo cp firefly_bringup/99-firefly-onboard.rules /etc/udev/rules.d
    sudo udevadm control --reload-rules && udevadm trigger

Install the udev rules on the ground control station by running the following command.

    sudo cp firefly_bringup/99-firefly-gcs.rules /etc/udev/rules.d
    sudo udevadm control --reload-rules && udevadm trigger

