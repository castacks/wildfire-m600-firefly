#include <ros/ros.h>
#include <base/BaseNode.h>
#include <core_pid_controller/pid_controller.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_listener.h>
#include <mav_msgs/RollPitchYawrateThrust.h>

class VelocityControlNode : public BaseNode {
private:
  PIDController* vx_controller;
  PIDController* vy_controller;
  PIDController* vz_controller;
  PIDController* yawrate_controller;

  ros::Subscriber odom_sub, twist_sub;
  tf::TransformListener* listener;

  ros::Publisher command_pub;

  ros::ServiceServer publish_control_server;

  nav_msgs::Odometry odom;
  bool got_odom, got_twist;
  bool should_publish;
  
  tf::Vector3 setpoint_vel_target_frame, setpoint_ang_vel_target_frame;
  tf::Vector3 actual_vel_target_frame, actual_ang_vel_target_frame;
  
  std::string target_frame;
  
public:
  VelocityControlNode();
  virtual bool initialize();
  virtual bool execute();
  virtual ~VelocityControlNode();

  void odom_callback(nav_msgs::Odometry odom);
  void twist_callback(geometry_msgs::TwistStamped twist);
  
  bool publish_control_callback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
};

VelocityControlNode::VelocityControlNode()
  : BaseNode("velocity_control_node"){
}

bool VelocityControlNode::initialize(){
  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();

  // init params
  target_frame = pnh->param(std::string("target_frame"), std::string("base_link_stabilized"));
  
  got_odom = false;
  got_twist = false;
  should_publish = true;

  // init controllers
  vx_controller = new PIDController("~/vx");
  vy_controller = new PIDController("~/vy");
  vz_controller = new PIDController("~/vz");
  yawrate_controller = new PIDController("~/yawrate");
  yawrate_controller->set_calculate_error_func(calculate_error_angle);

  // init subscribers
  twist_sub = nh->subscribe("velocity_setpoint", 10, &VelocityControlNode::twist_callback, this);
  odom_sub = nh->subscribe("odometry", 10, &VelocityControlNode::odom_callback, this);
  listener = new tf::TransformListener();

  // init publishers
  command_pub = nh->advertise<mav_msgs::RollPitchYawrateThrust>("roll_pitch_yawrate_thrust_setpoint", 10);

  // init services
  publish_control_server = pnh->advertiseService("publish_control", &VelocityControlNode::publish_control_callback, this);
  
  return true;
}

bool VelocityControlNode::execute(){
  if(got_odom && got_twist && should_publish){
    vx_controller->set_target(setpoint_vel_target_frame.x());
    vy_controller->set_target(setpoint_vel_target_frame.y());
    vz_controller->set_target(setpoint_vel_target_frame.z());
    yawrate_controller->set_target(setpoint_ang_vel_target_frame.z());
    
    float pitch = vx_controller->get_control(actual_vel_target_frame.x(), 1);
    float roll = vy_controller->get_control(actual_vel_target_frame.y(), 1);
    float thrust = vz_controller->get_control(actual_vel_target_frame.z(), 1);
    float yawrate = yawrate_controller->get_control(actual_ang_vel_target_frame.z(), 1);
    
    mav_msgs::RollPitchYawrateThrust drone_cmd;
    drone_cmd.roll = roll;
    drone_cmd.pitch = pitch;
    drone_cmd.yaw_rate = yawrate;
    drone_cmd.thrust.z = thrust;
    
    command_pub.publish(drone_cmd);
  }
  
  return true;
}

void VelocityControlNode::odom_callback(nav_msgs::Odometry odom){
  try{
    tf::StampedTransform odom_to_target_tf;
    listener->waitForTransform(target_frame, odom.child_frame_id, odom.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(target_frame, odom.child_frame_id, odom.header.stamp, odom_to_target_tf);
    // remove translation, we only care about rotating the velocities
    odom_to_target_tf.setOrigin(tf::Vector3(0, 0, 0));

    tf::Vector3 vel_odom_frame(odom.twist.twist.linear.x, odom.twist.twist.linear.y, odom.twist.twist.linear.z);
    actual_vel_target_frame = odom_to_target_tf*vel_odom_frame;
    tf::Vector3 ang_vel_odom_frame(odom.twist.twist.angular.x, odom.twist.twist.angular.y, odom.twist.twist.angular.z);
    actual_ang_vel_target_frame = odom_to_target_tf*ang_vel_odom_frame;
    
    got_odom = true;
  }
  catch(tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
}

void VelocityControlNode::twist_callback(geometry_msgs::TwistStamped twist){
  try{
    tf::StampedTransform twist_to_target_tf;
    listener->waitForTransform(target_frame, twist.header.frame_id, twist.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(target_frame, twist.header.frame_id, twist.header.stamp, twist_to_target_tf);
    // remove translation, we only care about rotating the velocities
    twist_to_target_tf.setOrigin(tf::Vector3(0, 0, 0));
    
    tf::Vector3 vel_twist_frame(twist.twist.linear.x, twist.twist.linear.y, twist.twist.linear.z);
    setpoint_vel_target_frame = twist_to_target_tf*vel_twist_frame;
    tf::Vector3 ang_vel_twist_frame(twist.twist.angular.x, twist.twist.angular.y, twist.twist.angular.z);
    setpoint_ang_vel_target_frame = twist_to_target_tf*ang_vel_twist_frame;

    got_twist = true;
  }
  catch(tf::TransformException ex){
    ROS_ERROR("%s",ex.what());
  }
}
bool VelocityControlNode::publish_control_callback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response){
  // if we transition from not publishing to publishing, reset the integrators
  if(!should_publish && request.data){
    vx_controller->reset_integral();
    vy_controller->reset_integral();
    vz_controller->reset_integral();
    yawrate_controller->reset_integral();
  }
  
  should_publish = request.data;
  
  response.success = true;
  return true;
}


VelocityControlNode::~VelocityControlNode(){
  
}


BaseNode* BaseNode::get(){
  VelocityControlNode* velocity_control_node = new VelocityControlNode();
  return velocity_control_node;
}
