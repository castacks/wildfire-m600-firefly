#pragma once

#include <base/BaseNode.h>
#include <behavior_tree/behavior_tree.h>
#include <behavior_tree_msgs/BehaviorTreeCommand.h>
#include <behavior_tree_msgs/BehaviorTreeCommands.h>
#include <behavior_tree_msgs/Status.h>
#include <core_drone_interface/DroneCommand.h>
#include <core_takeoff_landing_planner/TakeoffLandingCommand.h>
#include <core_trajectory_controller/TrajectoryMode.h>
#include <core_trajectory_msgs/FixedTrajectory.h>
#include <diagnostic_msgs/KeyValue.h>
#include <std_msgs/Bool.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <string>

class BehaviorExecutive : public BaseNode {
 private:
  // variables
  std::string takeoff_state, landing_state;

  // conditions
  std::vector<bt::Condition *> conditions;

  bt::Condition *request_control_commanded_condition;
  bt::Condition *arm_commanded_condition;
  bt::Condition *disarm_commanded_condition;
  bt::Condition *takeoff_commanded_condition;
  bt::Condition *land_commanded_condition;
  bt::Condition *traj_control_commanded_condition;
  bt::Condition *coverage_planner_commanded_condition;
  bt::Condition *ipp_planner_commanded_condition;

  bt::Condition *offboard_mode_condition;
  bt::Condition *armed_condition;
  bt::Condition *takeoff_complete_condition;
  bt::Condition *landed_condition;
  bt::Condition *in_air_condition;

  // actions
  std::vector<bt::Action *> actions;
  bt::Action *request_control_action;
  bt::Action *arm_action;
  bt::Action *disarm_action;
  bt::Action *takeoff_action;
  bt::Action *land_action;
  bt::Action *traj_control_action;
  bt::Action *coverage_planner_action;
  bt::Action *ipp_planner_action;

  // services
  ros::ServiceClient takeoff_landing_client;
  ros::ServiceClient trajectory_mode_client;
  ros::ServiceClient drone_command_client;
  ros::ServiceClient x_reset_integrator_client, y_reset_integrator_client,
      z_reset_integrator_client, yaw_reset_integrator_client;
  ros::ServiceClient vx_reset_integrator_client, vy_reset_integrator_client,
      vz_reset_integrator_client, yawrate_reset_integrator_client;

  // publishers
  ros::Publisher fixed_trajectory_pub;
  ros::Publisher in_air_pub;

  // subscribers
  ros::Subscriber behavior_tree_command_sub;
  ros::Subscriber takeoff_state_sub;
  ros::Subscriber landing_state_sub;
  ros::Subscriber has_control_sub;
  ros::Subscriber is_armed_sub;

  // callbacks
  void behavior_tree_command_callback(
      behavior_tree_msgs::BehaviorTreeCommands msg);
  void takeoff_state_callback(std_msgs::String msg);
  void landing_state_callback(std_msgs::String msg);
  void has_control_callback(std_msgs::Bool msg);
  void is_armed_callback(std_msgs::Bool msg);

 public:
  BehaviorExecutive(std::string node_name);

  virtual bool initialize();
  virtual bool execute();
  virtual ~BehaviorExecutive();
};
