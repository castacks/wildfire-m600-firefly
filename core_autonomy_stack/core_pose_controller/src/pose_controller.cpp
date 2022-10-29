#include <ros/ros.h>
#include <base/BaseNode.h>
#include <core_pid_controller/pid_controller.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/TwistStamped.h>
#include <std_srvs/SetBool.h>
#include <tf/transform_listener.h>

class PoseControlNode : public BaseNode {
private:
  PIDController* x_controller;
  PIDController* y_controller;
  PIDController* z_controller;
  PIDController* yaw_controller;

  ros::Subscriber tracking_point_sub, odom_sub;
  tf::TransformListener* listener;

  ros::Publisher cmd_vel_pub;

  ros::ServiceServer publish_control_server;

  nav_msgs::Odometry tracking_point, odom;
  bool got_tracking_point, got_odom;
  bool should_publish;
  
  tf::Vector3 tracking_point_target_frame, odom_target_frame;
  tf::Vector3 tracking_point_vel_target_frame, odom_vel_target_frame;
  double tracking_point_yaw_target_frame, odom_yaw_target_frame;
  
  std::string target_frame;

  void tracking_point_callback(nav_msgs::Odometry odom);
  void odom_callback(nav_msgs::Odometry odom);

  bool publish_control_callback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response);
  
public:
  PoseControlNode();
  virtual bool initialize();
  virtual bool execute();
  virtual ~PoseControlNode();

  
};

PoseControlNode::PoseControlNode()
  : BaseNode("pose_control_node"){
  
}

bool PoseControlNode::initialize(){
  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();

  // init params
  target_frame = pnh->param(std::string("target_frame"), std::string("map"));
  
  got_tracking_point = false;
  got_odom = false;
  should_publish = false;

  // init controllers
  x_controller = new PIDController("~/x");
  y_controller = new PIDController("~/y");
  z_controller = new PIDController("~/z");
  yaw_controller = new PIDController("~/yaw");
  yaw_controller->set_calculate_error_func(calculate_error_angle);

  // init subscribers
  tracking_point_sub = nh->subscribe("tracking_point", 10, &PoseControlNode::tracking_point_callback, this);
  odom_sub = nh->subscribe("odometry", 10, &PoseControlNode::odom_callback, this);
  listener = new tf::TransformListener();

  // init publishers
  cmd_vel_pub = nh->advertise<geometry_msgs::TwistStamped>("velocity_setpoint", 10);

  // init services
  publish_control_server = pnh->advertiseService("publish_control", &PoseControlNode::publish_control_callback, this);
  
  return true;
}

bool PoseControlNode::execute(){
  if(got_odom && got_tracking_point && should_publish){
    // set target for each controller
    x_controller->set_target(tracking_point_target_frame.x());
    y_controller->set_target(tracking_point_target_frame.y());
    z_controller->set_target(tracking_point_target_frame.z());
    yaw_controller->set_target(tracking_point_yaw_target_frame);

    // get velocity/yawrate
    double vx = x_controller->get_control(odom_target_frame.x(), tracking_point_vel_target_frame.x());
    double vy = y_controller->get_control(odom_target_frame.y(), tracking_point_vel_target_frame.y());
    double vz = z_controller->get_control(odom_target_frame.z(), tracking_point_vel_target_frame.z());
    double yawrate = yaw_controller->get_control(odom_yaw_target_frame);

    // publish
    geometry_msgs::TwistStamped cmd_vel;
    cmd_vel.header.stamp = tracking_point.header.stamp;
    cmd_vel.header.frame_id = target_frame;
    cmd_vel.twist.linear.x = vx;
    cmd_vel.twist.linear.y = vy;
    cmd_vel.twist.linear.z = vz;
    cmd_vel.twist.angular.z = yawrate;
    cmd_vel_pub.publish(cmd_vel);
  }
  
  return true;
}


void PoseControlNode::tracking_point_callback(nav_msgs::Odometry tracking_point){
  // transform the tracking point to the target frame
  tf::StampedTransform tracking_point_to_target_tf, child_frame_to_target_tf;
  try{
    this->tracking_point = tracking_point;
    listener->waitForTransform(target_frame, tracking_point.header.frame_id, tracking_point.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(target_frame, tracking_point.header.frame_id, tracking_point.header.stamp, tracking_point_to_target_tf);
    listener->waitForTransform(target_frame, tracking_point.child_frame_id, tracking_point.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(target_frame, tracking_point.child_frame_id, tracking_point.header.stamp, child_frame_to_target_tf);
    child_frame_to_target_tf.setOrigin(tf::Vector3(0, 0, 0));

    // transform position to target frame
    tf::Vector3 tracking_point_position(tracking_point.pose.pose.position.x,
					tracking_point.pose.pose.position.y,
					tracking_point.pose.pose.position.z);
    tracking_point_target_frame = tracking_point_to_target_tf*tracking_point_position;

    // transform velocity to target frame
    tf::Vector3 tracking_point_vel(tracking_point.twist.twist.linear.x,
				   tracking_point.twist.twist.linear.y,
				   tracking_point.twist.twist.linear.z);
    tracking_point_vel_target_frame = child_frame_to_target_tf*tracking_point_vel;
    
    // transform yaw to target frame
    tf::Quaternion tracking_point_q(tracking_point.pose.pose.orientation.x,
				 tracking_point.pose.pose.orientation.y,
				 tracking_point.pose.pose.orientation.z,
				 tracking_point.pose.pose.orientation.w);
    tracking_point_yaw_target_frame = tf::getYaw(tracking_point_to_target_tf*tracking_point_q);
    got_tracking_point = true;
  }
  catch (tf::TransformException ex){
    ROS_ERROR("Tracking point TF lookup: %s",ex.what());
  }
}

void PoseControlNode::odom_callback(nav_msgs::Odometry odom){
  // transform the state estimate odom to the target frame
  tf::StampedTransform odom_to_target_tf;
  try{
    this->odom = odom;
    listener->waitForTransform(target_frame, odom.header.frame_id, odom.header.stamp, ros::Duration(0.1));
    listener->lookupTransform(target_frame, odom.header.frame_id, odom.header.stamp, odom_to_target_tf);
    tf::Vector3 odom_position(odom.pose.pose.position.x,
			      odom.pose.pose.position.y,
			      odom.pose.pose.position.z);
    odom_target_frame = odom_to_target_tf*odom_position;
    tf::Quaternion odom_q(odom.pose.pose.orientation.x,
			  odom.pose.pose.orientation.y,
			  odom.pose.pose.orientation.z,
			  odom.pose.pose.orientation.w);
    odom_yaw_target_frame = tf::getYaw(odom_to_target_tf*odom_q);
    got_odom = true;
  }
  catch (tf::TransformException ex){
    ROS_ERROR("Odom TF lookup: %s",ex.what());
  }
}


bool PoseControlNode::publish_control_callback(std_srvs::SetBool::Request& request, std_srvs::SetBool::Response& response){
  // if we transition from not publishing to publishing, reset the integrators
  if(!should_publish && request.data){
    x_controller->reset_integral();
    y_controller->reset_integral();
    z_controller->reset_integral();
    yaw_controller->reset_integral();
    ROS_INFO("Pose controller publish control activated.")
  } else if (should_publish && !request.data) {
    ROS_INFO("Pose controller publish control deactivated.")
  }

  should_publish = request.data;
  
  response.success = true;
  return true;
}

PoseControlNode::~PoseControlNode(){

}


BaseNode* BaseNode::get(){
  PoseControlNode* pose_control_node = new PoseControlNode();
  return pose_control_node;
}
