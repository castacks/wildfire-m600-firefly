# Walkthrough Exercise

In this tutorial, we implement the "Autonomously Explore" functionality.
By default, the button does nothing. 
Our implementation will have the drone takeoff, travel to a waypoint, then land.

This is a written transcription of the associated [video tutorial](https://www.youtube.com/watch?v=xNz92Qbesug&t=48m40s). 

## Goals
1. Get familiar with the sim and set up the RQT GUI
2. Create a node that publishes a boolean indicating whether a waypoint has been hit
3. Modify the _Behavior Executive_ to implement the "Hit Waypoint" condition using the boolean
4. Modify the _Behavior Tree_ to make the drone takeoff, explore until the "Hit Waypoint" condition is true, then land

The **completed code** from this tutorial can be found in these repositories. 
- https://bitbucket.org/castacks/core_central/src/tutorial_week_2020/
- https://bitbucket.org/castacks/exploration_tutorial_week_2020/src/master/
- https://bitbucket.org/castacks/core_behavior_executive/src/tutorial_week_2020/
- https://bitbucket.org/castacks/core_gazebo_sim/src/tutorial_week_2020/


-----

## Step 0: Setup
Make sure to first follow the setup in the README.md. 
Follow the Gazebo build instructions.
Then source the `setup.bash` or `setup.zsh` file under `devel/`.

## Step 1: Launch Sim
This step makes sure everything is running correctly.

In one terminal, run:

```bash
roscore
```
In another terminal, run:
```bash
mon launch core_central gazebo_example_lidar.launch
```

[TODO] gui config
[TODO] insert pictures

Gazebo takes a while to launch, especially the first time. Don't worry and be patient.

## Step 2: Create Exploration Node

The Exploration node will simply publish the distance from the drone to the current waypoint.

(The completed changes can be found at 
https://bitbucket.org/castacks/exploration_tutorial_week_2020/src/master)

### 2a: Generate Boilerplate
Run the following:
```bash
cd ws/src/template_node
./node_generation.sh Exploration  # be sure to use PascalCase, to follow C++ class syntax
```

The `node_generation.sh` script generates a new ROS node called "Exploration" in the parent directory.

If you now look under `ws/src/exploration`, you'll see it generated boilerplate:
- The node's CMakeLists.txt includes `base_INCLUDE_DIRS`.
- The node's `include` folder contains the header `exploration.h`. This defines the class `Exploration` that extends `BaseNode`.
- The node's `src` folder (`ws/src/exploration/src`) includes `exploration.cpp`, which contain the function implementations.


### 2b: Specify Dependencies

We first modify the CMakeLists, package.xml, and header files to include the ROS message types that we'll use.

In CMakeLists.txt, add `nav_msgs` and `geometry_msgs` to `find_package()`:
```c++
/** CMakeLists.txt */

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  sensor_msgs
  nav_msgs
  geometry_msgs
  tf
)
```

Add dependencies in package.xml:
```xml
/** package.xml */

  <build_depend>geometry_msgs</build_depend>
  <build_depend>nav_msgs</build_depend>
  <build_depend>tf</build_depend>
  <build_export_depend>geometry_msgs</build_export_depend>
  <build_export_depend>nav_msgs</build_export_depend>
  <build_export_depend>tf</build_export_depend>
  <exec_depend>geometry_msgs</exec_depend>
  <exec_depend>nav_msgs</exec_depend>
  <exec_depend>tf</exec_depend>

```

### 2c: Header File - Declare the "What"

Include libraries in the header file: `include/exploration/exploration.h`:
```c++
/** include/exploration/exploration.h
Place at top of file with other include statements 
*/

#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>  // for the waypoint
#include <nav_msgs/Odometry.h>          // drone odometry
#include <tf/transform_datatypes.h>     // spatial transformations
```


In the same header file, we'll declare the variables and functions:

```c++
/** include/exploration/exploration.h - within the Exploration class, under private attributes */

// variables
bool got_waypoint, got_odom;  // whether we've been given a waypoint, drone's odometry
tf::Vector3 waypoint, odom;   // ??? transformations
bool hit_waypoint;              // whether the drone has reached the waypoint
// publishers
ros::Publisher hit_waypoint_pub; // ???
// subscribers
ros::Subscriber waypoint_sub;
ros::Subscriber odom_sub;
// callbacks
void waypoint_callback(geometry_msgs::PoseStamped msg);
void odom_callback(nav_msgs::Odometry msg);
```

### 2d: Cpp File - Define the "How"
Now let's implement the functionality in the cpp file.

We use "`nh`", the node handler, to subscribe to "custom_waypoint" and "odometry" topics. 
We also use it to publish whether we reached the waypoint.
```c++
/** src/exploration.cpp - within initialize() function */

ros::NodeHandle* nh = get_node_handle();
ros::NodeHandle* pnh = get_private_node_handle();

// init variables
got_waypoint = false;  // whether we know where to go
got_odom = false;      // whether we know where the drone is
hit_waypoint = false;  // whether the drone has hit the waypoint
// init subscribers
waypoint_sub = nh->subscribe("custom_waypoint", 1, &Exploration::waypoint_callback, this);
odom_sub = nh->subscribe("odometry", 1, &Exploration::odom_callback, this);
// init publishers
hit_waypoint_pub = nh->advertise<std_msgs::Bool>("hit_waypoint", 1);
```

Now we implement how our node should execute. 
```c++
/** src/exploration.cpp - within execute() function */

// only execute if (1) we know the drone's odometry (2) we know where to go
if(got_odom && got_waypoint){
    // log the drone's distance to the waypoint
    ROS_INFO_STREAM("distance: " << waypoint.distance(odom));

    // consider that we've hit the waypoint if we're within a distance of 4
    if(!hit_waypoint)
        hit_waypoint = waypoint.distance(odom) < 4.0f;

    // publish whether we've hit the waypoint
    std_msgs::Bool hit_waypoint_msg;
    hit_waypoint_msg.data = hit_waypoint;
    hit_waypoint_pub.publish(hit_waypoint_msg);
}
```

We implement some callbacks to parse the waypoint and odometry information that we subscribed to.
```c++
/** src/exploration.cpp - global scope (not in any function) */

void Exploration::waypoint_callback(geometry_msgs::PoseStamped msg){
    /** 
    Defines what to do with the waypoint message that we subscribed to.
        1) update state to indicate we received the waypoint
        2) store the waypoint position as an <x, y, z> Vector.
    */
    got_waypoint = true;
    waypoint = tf::Vector3(
        msg.pose.position.x, msg.pose.position.y, msg.pose.position.z
    );
}
void Exploration::odom_callback(nav_msgs::Odometry msg){
    /**
    Defines what to do with the drone's odometry message that we subscribed to.
        1) update state to indicate we received the odometry
        2) store the odometry's position information as an <x, y, z> vector
    */
    got_odom = true;
    odom = tf::Vector3(
        msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z
    );
}
```

## Step 3: Launch Exploration Node
Now that we've created a ROS node, let's launch it.

A launch file was auto-generated for our Exploration node at `exploration/launch/exploration.launch`.
We'll add it to core_central's _autonomy.launch_ file.

Edit `core_central/launch/common/autonomy.launch`. Under the "behavior" section, add:
```xml
  <include file="$(find exploration)/launch/exploration.launch" />
```

Now rebuild the project with `catkin build`, and launch the core autonomy stack with:
```bash
mon launch core_central gazebo_example_lidar.launch
```
We can test that it works once RViz loads.
In RViz, set a waypoint by hitting "G" on your keyboard, then click on the map.

If the exploration node was implemented correctly, it will continuously report the distance from the drone to the node.
We can see the distance printed in the terminal:

![exploration_node_distances_in_rosmon](docs/images/exploration_node_distances_in_rosmon.png)



## Step 4: Modify Behavior Executive

We will now modify the Behavior Executive to add functionality for the waypoint.

(The completed changes can be found at https://bitbucket.org/castacks/core_behavior_executive/src/tutorial_week_2020/)

### 4a: Header File
Here we set up some variables that we'll use in in the cpp file.

Edit `core_behavior_executive/include/core_behavior_executive/behavior_executive_behavior_tree.h`:

1)
  ```c++
  /** Place at end of "conditions" comment, around line 42 */

  // conditions
  // ...
  bt::Condition* hit_waypoint_condition;
  ```
2) 
  ```c++
  /** Place at end of "subscribers" comment, around line 74 */

  // subscribers
  // ...
  ros::Subscriber hit_waypoint_subscriber;
  ```
3) 
  ```c++
  /** Place at end of "subscribers" comment, around line 83 */

  // callbacks 
  // ...
  void hit_waypoint_callback(std_msgs::Bool msg);
  ```

### 4b: Cpp File
`core_behavior_executive/src/behavior_executive_behavior_tree.cpp`

1) We define a new branching condition for the Behavior Tree.
The "Hit Waypoint" string will match what we put in our Behavior Tree config file.
  ```c++
  /** Place at end of "init conditions" comment, around line 29 */

  //  init conditions
  // ...
  hit_waypoint_condition = new bt::Condition("Hit Waypoint");
  ```

2) Now we append the condition to the `conditions` list. 
Everything in that list will be automatically published.
  ```c++
  /** Place under the other conditions.push_back(...) lines, around line 45 */

  // conditions.push_back(...);
  // conditions.push_back(in_air_condition);
  conditions.push_back(hit_waypoint_condition);
  ```
3) We subscribe to the "hit_waypoint" topic that our Exploration node publishes, and give it a callback to run when it receives a message.
  ```c++
    /** Place at end of "init subscribers" comment, around line 93 */

    // init subscribers
    // ...
    hit_waypoint_subscriber = nh->subscribe("hit_waypoint", 10, 
        &BehaviorExecutiveBehaviorTree::hit_waypoint_callback, this);
  ```
4) We define that callback to simply pass the boolean message on to our `hit_waypoint_condition`.
  ```c++
  /** Place near the end of the file, around line 320 */

  void BehaviorExecutiveBehaviorTree::hit_waypoint_callback(std_msgs::Bool msg){
    hit_waypoint_condition->set(msg.data);
  }
  ```


## Step 5: Modify Behavior Tree
Finally, we modify the Behavior Tree config file.
When "Autonomously Explore" is clicked, it will takeoff, explore until it hits the waypoint, then land.

(The completed changes can be found at https://bitbucket.org/castacks/core_gazebo_sim/src/tutorial_week_2020/)


Edit 
`src/core_gazebo_sim/config/drone.tree`:

```c++
  /** Add at the bottom of the file */

	->
		(Autonomously Explore Commanded)
		->
			?
				(Takeoff Complete)
				[Takeoff]
			?
				(Hit Waypoint)
				[Explore]
			?
				(Landed)
				[Land]
```

- "`->`" means all children must return true
- "`?`" means just one child must return true
- "`()`" denotes conditions
- "`[]`" denotes actions


## Test It
Launch the sim with:
```bash
mon launch core_central gazebo_example_lidar.launch
```

1) Load the correct behavior tree to `ws/src/core_gazebo_sim/config/drone.tree`. If it loaded, you'll see the "Autonomously Explore" branch in the tree.
2) Load the correct GUI layout
3) Set a waypoint in RViz with the G key and mouse click
4) Click "Autonomously Explore"
5) The drone should takeoff, travel to the waypoint, and land.

Congratulations! You finished the tutorial.