#include <pluginlib/class_list_macros.h>
#include <core_dji_interface/dji_interface.h>
#include <ros/ros.h>
#include <tf/transform_datatypes.h>

DJIInterface::DJIInterface(){
  ros::NodeHandle nh;
  ros::NodeHandle pnh("~");

  // init parameters
  stabilized_body_frame = pnh.param("stabilized_body_frame", std::string("uav1/base_link_stabilized"));
  world_frame = pnh.param("world_frame", std::string("uav1/map"));
  
  // init subscribers
  listener = new tf::TransformListener();

  // init publishers
  setpoint_generic_pub = nh.advertise<sensor_msgs::Joy>("dji_sdk/flight_control_setpoint_generic", 10);
  
  // init services
  sdk_control_authority_client = nh.serviceClient<dji_sdk::SDKControlAuthority>("dji_sdk/sdk_control_authority");
  drone_arm_control_client = nh.serviceClient<dji_sdk::DroneArmControl>("dji_sdk/drone_arm_control");
  set_local_pos_ref_client = nh.serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");
  
  // init variables
  armed = false;
  has_offboard_control = false;
}

bool DJIInterface::request_control(){
  // set local position origin
  // dji_sdk::SetLocalPosRef set_local_pos_ref_srv;
  // bool set_local_pos_ref_success =
  // set_local_pos_ref_client.call(set_local_pos_ref_srv); ROS_INFO_STREAM("Set
  // Local Pos Ref success: " << set_local_pos_ref_success);

  dji_sdk::SDKControlAuthority srv;
  srv.request.control_enable = dji_sdk::SDKControlAuthority::Request::REQUEST_CONTROL;
  bool success = sdk_control_authority_client.call(srv) && srv.response.result;
  if(success)
    has_offboard_control = true;
  return success;
}

bool DJIInterface::arm(){
  dji_sdk::DroneArmControl srv;
  srv.request.arm = dji_sdk::DroneArmControl::Request::ARM_COMMAND;
  bool success = drone_arm_control_client.call(srv) && srv.response.result;
  if(success)
    armed = true;
  return success;
}

bool DJIInterface::disarm(){
  dji_sdk::DroneArmControl srv;
  srv.request.arm = dji_sdk::DroneArmControl::Request::DISARM_COMMAND;
  bool success = drone_arm_control_client.call(srv) && srv.response.result;
  if(success)
    armed = false;
  return success;
}

bool DJIInterface::is_armed(){
  return armed;
}

bool DJIInterface::has_control(){
  return has_offboard_control;
}

void DJIInterface::command_velocity(geometry_msgs::TwistStamped msg){
  try{
    // transform the velocities to the stabilized body frame
    tf::StampedTransform transform;
    listener->waitForTransform(stabilized_body_frame, msg.header.frame_id, msg.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(stabilized_body_frame, msg.header.frame_id, msg.header.stamp, transform);
    transform.setOrigin(tf::Vector3(0, 0, 0)); // only use rotation. we don't want to translate the velocities.
    tf::Vector3 vel(msg.twist.linear.x, msg.twist.linear.y, msg.twist.linear.z);
    tf::Vector3 vel_body_frame = transform*vel;
    //ROS_INFO_STREAM("Before TF: " << vel.x() << " " << vel.y() << " " << vel.z() << " After TF: " << vel_body_frame.x() << " " << vel_body_frame.y() << " " << vel_body_frame.z());

    // construct the DJI command
    sensor_msgs::Joy joy;
    joy.header.stamp = ros::Time::now();
    joy.header.frame_id = "world";
    joy.axes.resize(5);
    joy.axes[0] = vel_body_frame.x(); // x component
    joy.axes[1] = vel_body_frame.y(); // y component
    joy.axes[2] = vel_body_frame.z(); // z component
    joy.axes[3] = msg.twist.angular.z; // yaw/yawrate component
    joy.axes[4] = COMMAND_HORIZONTAL_VELOCITIES | COMMAND_VERTICAL_VELOCITY | COMMAND_YAW_RATE | BODY_FLU_FRAME; // flag
    setpoint_generic_pub.publish(joy);
  }
  catch(tf::TransformException &ex){
    ROS_ERROR_STREAM("TransformException in command_velocity: " << ex.what());
  }
}


void DJIInterface::command_pose(geometry_msgs::PoseStamped msg){
  try{
    // transform the velocities to the stabilized body frame
    tf::StampedTransform transform_body;
    listener->waitForTransform(stabilized_body_frame, msg.header.frame_id, msg.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(stabilized_body_frame, msg.header.frame_id, msg.header.stamp, transform_body);
    tf::Vector3 position(msg.pose.position.x, msg.pose.position.y, msg.pose.position.z);
    tf::Vector3 position_body_frame = transform_body*position;

    tf::StampedTransform transform_world;
    listener->waitForTransform(world_frame, msg.header.frame_id, msg.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(world_frame, msg.header.frame_id, msg.header.stamp, transform_world);
    tf::Vector3 position_world_frame = transform_world*position;
    tf::Quaternion q(msg.pose.orientation.x, msg.pose.orientation.y, msg.pose.orientation.z, msg.pose.orientation.w);
    tf::Quaternion q_world_frame = transform_world*q;
    double roll, pitch, yaw;
    tf::Matrix3x3(q_world_frame).getRPY(roll, pitch, yaw);

    // construct the DJI command
    sensor_msgs::Joy joy;
    joy.header.stamp = ros::Time::now();
    joy.header.frame_id = "world";
    joy.axes.resize(5);
    joy.axes[0] = position_body_frame.x(); // x component
    joy.axes[1] = position_body_frame.y(); // y component
    joy.axes[2] = position_world_frame.z(); // z component
    joy.axes[3] = yaw; // yaw/yawrate component
    joy.axes[4] = COMMAND_POSITION_OFFSETS | COMMAND_ALTITUDE | COMMAND_YAW_ANGLE | BODY_FLU_FRAME; // flag
    setpoint_generic_pub.publish(joy);
  }
  catch(tf::TransformException &ex){
    ROS_ERROR_STREAM("TransformException in command_velocity: " << ex.what());
  }
}

void DJIInterface::command_roll_pitch_yawrate_thrust(mav_msgs::RollPitchYawrateThrust msg){
  // construct the DJI command
  sensor_msgs::Joy joy;
  joy.header.stamp = ros::Time::now();
  joy.header.frame_id = "world";
  joy.axes.resize(5);
  joy.axes[0] = msg.roll; // x component
  joy.axes[1] = msg.pitch; // y component
  joy.axes[2] = msg.thrust.z; // z component
  joy.axes[3] = msg.yaw_rate; // yaw/yawrate component
  joy.axes[4] = COMMAND_ROLL_PITCH | COMMAND_THRUST | COMMAND_YAW_RATE | BODY_FLU_FRAME; // flag
  setpoint_generic_pub.publish(joy);
}

PLUGINLIB_EXPORT_CLASS(DJIInterface, DroneInterface)
