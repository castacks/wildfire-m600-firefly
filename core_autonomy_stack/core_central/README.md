# Core Central

This repo serves as a starting point for setting up the core autonomy software stack. Platforms that are currently supported are PX4, DJI, and a pure gazebo simulation drone. Below there is a general overview of the software architecture followed by instructions on how to set up and build the code for each specific platform.

For an in-depth walkthrough on the core autonomy stack, you may consult 
[this video](https://www.youtube.com/watch?v=xNz92Qbesug&list=PLpJxwrRy4Qbte02Z8f0hYcQwWW4AdtBSW&index=9) from the AirLab Summer School 2020 (uploaded August 31, 2020).

![Image of YouTube walkthrough](http://i3.ytimg.com/vi/xNz92Qbesug/hqdefault.jpg)

[Link to video.](https://www.youtube.com/watch?v=xNz92Qbesug&list=PLpJxwrRy4Qbte02Z8f0hYcQwWW4AdtBSW&index=9)

## Overview

### System Requirements
- Ubuntu 18.04 - the repo uses ROS melodic, which only officially supports Ubuntu 18. Make sure to use this distro to avoid miscellaneous build errors

### Install Dependencies

Run:
```
sudo apt install ros-melodic-mav-msgs ros-melodic-rosmon python-wstool
```

### Building the Code

The general pattern for building the code is to pull this repo, create a symbolic link to the rosinstall for one of the platforms, pull the repos with `wstool update`, then build with `catkin build`. There may be additional platform specific steps as well.

### Launch File Structure

The launch files are organized into directories for each platform, gazebo/, px4/, and dji/, along with a common/ with launch files usable by each platform. The simulation example launch files have the following format `[platform name]_example_[example type].launch`. The `platform name` will be either gazebo, px4, or dji. The example type describes the example being run, for example a lidar based drone, disparity based drone, or multiple drone simulation. Inside the example launch files, gazebo is launched with a world file and one or more simulated drones running the software stack are created by including a `[platform name]_sim_drone.launch` launch file one or more times.

The `[platform name]_sim_drone.launch` launch files include launch files to spawn the drone in sim, run state estimation, run control, run the rest of the autonomy in the core stack, and run some visualization including rviz and rqt. The state estimation and control are placed in separate launch files because usually the state estimation and control gains are different between sim and real life, so for a real drone you would swap these two launch files out. The launch file that runs the rest of the autonomy can remain the same and should run onboard the drone. The visualization can also remain the same but should run on the base station computer you are using to communicate with the drone.

### Using the stack in your project

When modifying a package in the stack for your project specific needs, you should create a fork with your project's name in that package. The branch that comes with the core autonomy stack (usually master) is locked so that only people who are maintainers of the stack can push to it. If you have a feature you would like to be merged into the core you should create a pull request.

### Bugs / Feedback

If you encounter any bugs or have feedback about how the software or documentation could be improved, contact John Keller (slack: kellerj).

## Gazebo

### Building the Code

Create a catkin workspace, download this repo, use the rosinstall to get the core autonomy packages, and build:
```
mkdir -p ws/src
cd ws/src
git clone git@github.com:Wildfire-Robots/core_central.git
ln -s core_central/rosinstalls/sim.rosinstall .rosinstall
wstool up
cd ..
catkin build
```

### Running the Examples

A video showing what all of the examples should look like is here: (https://youtu.be/w_rxTrj0Io4)
The names of the launch files has slightly changed since this video was made. Instead of running the commands in the video run the commands listed below.

#### Example 1

```
mon launch core_central gazebo_example_empty.launch
```

This example shows how to use the GUI to control the drone in an empty world. First use the `Open config` buttons to select the configuration yaml files, then export the rqt persepctive as shown in the video. This will make it so you don't have to select the configuration files each time you open the GUI. Next press the `Takeoff` button. While the drone is taking off a racetrack trajectory is published. When the `Explore` button is pressed, the local planner will start trying to follow the racetrack.

#### Example 2

```
mon launch core_central gazebo_example_lidar.launch
```

This example shows a drone with a pointcloud based map representation navigating in a world with obstacles. Press `Takeoff` like before. In RVIZ, press the `g` key on the keyboard and click a point that you want the drone to travel to. This will send a waypoint to the drone which it will move towards.

#### Example 3

```
mon launch core_central gazebo_example_disparity.launch
```

This example shows a drone with a disparity based map representation navigating in a world with obstacles. It can be interacted with in the same way as the previous example. Since the same RVIZ configuration is used in all examples, both the Velodyne pointcloud and stereo disparity points are shown, but only one is used depending on the map representation.

#### Example 4

```
mon launch core_central gazebo_example_multiple.launch
```

This example shows two drones in the same world. It is meant to show that the topics are namespaced and tfs prefixed in a way that there is no interference between the two systems. One drone is using a pointcloud based map representation, the other is used a disparity based map representation.

## PX4

### Building the Code

The main difference between the pure gazebo build and the px4 build, is that you must build the firmware, using `make px4_sitl_default gazebo` in addition to doing `catkin build`. Running `make px4_sitl_default gazebo` builds the firmware AND launchs a gazebo simulation. If you just do `make px4_sitl_default`, the px4 sitl will not respond to mavros. With the `gazebo` line it does work but launches gazebo when the build is finished, so you need to manually Ctrl-C on the terminal when gazebo pops up so you can continue building the code. Do the following:

### Install Dependencies

Run:
```
sudo apt install python-jinja2 ros-melodic-geographic-msgs libgeographic-dev geographiclib-tools
sudo pip install numpy toml future

```

### Build Workspace
```
mkdir -p ws/src
cd ws/src
git clone git@bitbucket.org:castacks/core_central.git
ln -s core_central/rosinstalls/px4.rosinstall .rosinstall
wstool up
cd Firmware
make px4_sitl_default gazebo # This will build the firmware and launch a gazebo simulation. When the gazebo window shows up, Ctrl-C on the command line and run the next commands.
```

the following steps only need to be done once to install the datasets:

```
cd ../mavros/mavros/scripts
sudo su
./install_geographiclib_datasets.sh
```
then you can build the workspace as usual:

```
cd /path/to/workspace/
catkin build
```

### Running the Examples

The examples should look the same as the pure gazebo version, but with a px4 drone. Run the commands listed below.

#### Example 1

```
mon launch core_central px4_example_empty.launch
```

#### Example 2

```
mon launch core_central px4_example_lidar.launch
```

#### Example 3

```
mon launch core_central px4_example_disparity.launch
```

#### Example 4

```
mon launch core_central px4_example_multiple.launch
```

## DJI

### Building the Code

Create a catkin workspace, download this repo, use the rosinstall to get the core autonomy packages, and build:
```
mkdir -p ws/src
cd ws/src
git clone git@bitbucket.org:castacks/core_central.git
ln -s core_central/rosinstalls/dji.rosinstall .rosinstall
wstool up
cd ..
catkin build
```

### Running the Examples

The example for DJI is different than the pure gazebo and px4 examples because it uses the DJI Assistant to do hardware in the loop simulation. To do this the USB port on the DJI M210 has to be connected to a windows computer runnning DJI Assistant 2 which is used to run the simulation. The TTL output on the DJI should be connected to the computer running the core autonomy stack. To run without the USB cable connected the Onboard DJI SDK has to be built with advanced sensing disabled which is done by commenting out the `enable_advanced_sensing = true;` line in Onboard-SDK-ROS/dji_sdk/src/modules/dji_sdk_node.cpp.

```
mon launch core_central dji_sim_drone.launch
```