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
#include <dji_sdk/SetLocalPosRef.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/String.h>
#include <std_srvs/Empty.h>

#include <string>

class BehaviorExecutive : public BaseNode {
 private:
  // variables
  std::string takeoff_state, landing_state;

  // conditions
  std::vector<bt::Condition *> conditions;
  std::vector<bt::Condition *> autonomy_mode_conditions;
  bt::Condition *reset_bt_commanded_condition;
  bt::Condition *disarm_commanded_condition;
  bt::Condition *arm_commanded_condition;
  bt::Condition *set_local_pos_ref_commanded_condition;
  bt::Condition *local_pos_ref_set_condition;
  bt::Condition *request_control_commanded_condition;
  bt::Condition *have_control_condition;
  bt::Condition *autonomy_idle_condition;
  bt::Condition *autonomy_takeoff_condition;
  bt::Condition *autonomy_land_condition;
  bt::Condition *autonomy_traj_control_condition;
  bt::Condition *autonomy_coverage_planner_condition;
  bt::Condition *autonomy_ipp_planner_condition;
  bt::Condition *offboard_mode_condition;
  bt::Condition *takeoff_complete_condition;
  bt::Condition *landed_condition;
  bt::Condition *in_air_condition;
  bt::Condition *got_initial_ipp_plan_condition;
  bt::Condition *get_initial_ipp_plan_commanded;

  // actions
  std::vector<bt::Action *> actions;
  bt::Action *reset_bt_action;
  bt::Action *disarm_action;
  bt::Action *arm_action;
  bt::Action *set_local_pos_ref_action;
  bt::Action *request_control_action;
  bt::Action *idle_action;
  bt::Action *takeoff_action;
  bt::Action *land_action;
  bt::Action *traj_control_action;
  bt::Action *coverage_planner_action;
  bt::Action *ipp_planner_action;
  bt::Action *get_initial_ipp_plan_action;

  // services
  ros::ServiceClient takeoff_landing_client;
  ros::ServiceClient trajectory_mode_client;
  ros::ServiceClient drone_command_client;
  ros::ServiceClient x_reset_integrator_client, y_reset_integrator_client,
      z_reset_integrator_client, yaw_reset_integrator_client;
  ros::ServiceClient vx_reset_integrator_client, vy_reset_integrator_client,
      vz_reset_integrator_client, yawrate_reset_integrator_client;
  ros::ServiceClient publish_control_client;
  ros::ServiceClient set_local_pos_ref_client;

  // publishers
  ros::Publisher fixed_trajectory_pub;
  ros::Publisher in_air_pub;
  ros::Publisher generate_ipp_plan_request_pub;
  ros::Publisher execute_ipp_plan_pub;
  ros::Publisher wait_for_initial_ipp_plan_pub;

  // subscribers
  ros::Subscriber behavior_tree_command_sub;
  ros::Subscriber takeoff_state_sub;
  ros::Subscriber landing_state_sub;
  ros::Subscriber has_control_sub;
  ros::Subscriber got_initial_ipp_plan_sub;

  // callbacks
  void behavior_tree_command_callback(
      behavior_tree_msgs::BehaviorTreeCommands msg);
  void takeoff_state_callback(std_msgs::String msg);
  void landing_state_callback(std_msgs::String msg);
  void has_control_callback(std_msgs::Bool msg);
  void got_initial_ipp_plan_callback(std_msgs::Bool msg);

  void reset_integrators();
  void enable_pose_controller_output();
  void disable_pose_controller_output();

 public:
  BehaviorExecutive(std::string node_name);

  virtual bool initialize();
  virtual bool execute();
  virtual ~BehaviorExecutive();
};
