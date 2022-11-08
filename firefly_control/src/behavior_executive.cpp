#include "behavior_executive.h"

#include <base/BaseNode.h>
#include <std_srvs/SetBool.h>

#include <string>

BehaviorExecutive::BehaviorExecutive(std::string node_name)
    : BaseNode(node_name) {}

bool BehaviorExecutive::initialize() {
  ros::NodeHandle* nh = get_node_handle();
  ros::NodeHandle* pnh = get_private_node_handle();

  // init conditions
  reset_bt_commanded_condition =
      new bt::Condition("Reset Behavior Tree Commanded");
  disarm_commanded_condition = new bt::Condition("Disarm Commanded");
  arm_commanded_condition = new bt::Condition("Arm Commanded");
  set_local_pos_ref_commanded_condition =
      new bt::Condition("Set Local Position Reference Commanded");

  local_pos_ref_set_condition =
      new bt::Condition("Local Position Reference Set");

  request_control_commanded_condition =
      new bt::Condition("Request Control Commanded");
  have_control_condition = new bt::Condition("Have Control");

  autonomy_idle_condition = new bt::Condition("Autonomy Mode Is Idle");
  autonomy_takeoff_condition = new bt::Condition("Autonomy Mode Is Takeoff");
  autonomy_land_condition = new bt::Condition("Autonomy Mode Is Land");
  autonomy_traj_control_condition =
      new bt::Condition("Autonomy Mode Is Traj Control");
  autonomy_coverage_planner_condition =
      new bt::Condition("Autonomy Mode Is Coverage Planner");
  autonomy_ipp_planner_condition =
      new bt::Condition("Autonomy Mode Is IPP Planner");

  offboard_mode_condition = new bt::Condition("Offboard Mode");
  takeoff_complete_condition = new bt::Condition("Takeoff Complete");
  landed_condition = new bt::Condition("Landed");
  in_air_condition = new bt::Condition("In Air");
  got_initial_ipp_plan_condition = new bt::Condition("Got Initial IPP Plan");
  get_initial_ipp_plan_commanded =
      new bt::Condition("Get Initial IPP Plan Commanded");

  conditions.push_back(reset_bt_commanded_condition);
  conditions.push_back(disarm_commanded_condition);
  conditions.push_back(arm_commanded_condition);
  conditions.push_back(set_local_pos_ref_commanded_condition);
  conditions.push_back(local_pos_ref_set_condition);
  conditions.push_back(request_control_commanded_condition);
  conditions.push_back(have_control_condition);
  conditions.push_back(autonomy_idle_condition);
  conditions.push_back(autonomy_takeoff_condition);
  conditions.push_back(autonomy_land_condition);
  conditions.push_back(autonomy_traj_control_condition);
  conditions.push_back(autonomy_coverage_planner_condition);
  conditions.push_back(autonomy_ipp_planner_condition);
  conditions.push_back(offboard_mode_condition);
  conditions.push_back(takeoff_complete_condition);
  conditions.push_back(landed_condition);
  conditions.push_back(in_air_condition);
  conditions.push_back(got_initial_ipp_plan_condition);
  conditions.push_back(get_initial_ipp_plan_commanded);

  autonomy_mode_conditions.push_back(autonomy_idle_condition);
  autonomy_mode_conditions.push_back(autonomy_takeoff_condition);
  autonomy_mode_conditions.push_back(autonomy_land_condition);
  autonomy_mode_conditions.push_back(autonomy_traj_control_condition);
  autonomy_mode_conditions.push_back(autonomy_coverage_planner_condition);
  autonomy_mode_conditions.push_back(autonomy_ipp_planner_condition);

  // init actions
  reset_bt_action = new bt::Action("Reset Behavior Tree");
  disarm_action = new bt::Action("Disarm");
  arm_action = new bt::Action("Arm");
  set_local_pos_ref_action = new bt::Action("Set Local Position Reference");
  request_control_action = new bt::Action("Request Control");

  idle_action = new bt::Action("Autonomy Idle");
  takeoff_action = new bt::Action("Autonomy Takeoff");
  land_action = new bt::Action("Autonomy Land");

  traj_control_action = new bt::Action("Autonomy Traj Control");
  coverage_planner_action = new bt::Action("Autonomy Coverage Planner");
  ipp_planner_action = new bt::Action("Autonomy IPP Planner");

  get_initial_ipp_plan_action = new bt::Action("Idle and Get Initial IPP Plan");

  actions.push_back(reset_bt_action);
  actions.push_back(disarm_action);
  actions.push_back(arm_action);
  actions.push_back(set_local_pos_ref_action);
  actions.push_back(request_control_action);

  actions.push_back(idle_action);
  actions.push_back(takeoff_action);
  actions.push_back(land_action);

  actions.push_back(traj_control_action);
  actions.push_back(coverage_planner_action);
  actions.push_back(ipp_planner_action);
  actions.push_back(get_initial_ipp_plan_action);

  // init services
  takeoff_landing_client =
      nh->serviceClient<core_takeoff_landing_planner::TakeoffLandingCommand>(
          "set_takeoff_landing_command");
  trajectory_mode_client =
      nh->serviceClient<core_trajectory_controller::TrajectoryMode>(
          "set_trajectory_mode");
  drone_command_client =
      nh->serviceClient<core_drone_interface::DroneCommand>("drone_command");
  x_reset_integrator_client =
      nh->serviceClient<std_srvs::Empty>("pose_controller/x/reset_integrator");
  y_reset_integrator_client =
      nh->serviceClient<std_srvs::Empty>("pose_controller/y/reset_integrator");
  z_reset_integrator_client =
      nh->serviceClient<std_srvs::Empty>("pose_controller/z/reset_integrator");
  yaw_reset_integrator_client = nh->serviceClient<std_srvs::Empty>(
      "pose_controller/yaw/reset_integrator");
  vx_reset_integrator_client = nh->serviceClient<std_srvs::Empty>(
      "velocity_controller/vx/reset_integrator");
  vy_reset_integrator_client = nh->serviceClient<std_srvs::Empty>(
      "velocity_controller/vy/reset_integrator");
  vz_reset_integrator_client = nh->serviceClient<std_srvs::Empty>(
      "velocity_controller/vz/reset_integrator");
  yawrate_reset_integrator_client = nh->serviceClient<std_srvs::Empty>(
      "velocity_controller/yawrate/reset_integrator");
  publish_control_client =
      nh->serviceClient<std_srvs::SetBool>("pose_controller/publish_control");
  set_local_pos_ref_client =
      nh->serviceClient<dji_sdk::SetLocalPosRef>("dji_sdk/set_local_pos_ref");

  // init publishers
  fixed_trajectory_pub = nh->advertise<core_trajectory_msgs::FixedTrajectory>(
      "fixed_trajectory", 10);
  in_air_pub = nh->advertise<std_msgs::Bool>("in_air", 1);
  generate_ipp_plan_request_pub =
      nh->advertise<std_msgs::Empty>("generate_ipp_plan_request", 1);
  execute_ipp_plan_pub = nh->advertise<std_msgs::Bool>("execute_ipp_plan", 1);
  wait_for_initial_ipp_plan_pub =
      nh->advertise<std_msgs::Empty>("wait_for_initial_ipp_plan", 1);

  // init subscribers
  behavior_tree_command_sub =
      nh->subscribe("behavior_tree_commands", 10,
                    &BehaviorExecutive::behavior_tree_command_callback, this);
  takeoff_state_sub = nh->subscribe(
      "takeoff_state", 10, &BehaviorExecutive::takeoff_state_callback, this);
  landing_state_sub = nh->subscribe(
      "landing_state", 10, &BehaviorExecutive::landing_state_callback, this);
  has_control_sub = nh->subscribe(
      "has_control", 10, &BehaviorExecutive::has_control_callback, this);
  got_initial_ipp_plan_sub =
      nh->subscribe("got_initial_ipp_plan", 10,
                    &BehaviorExecutive::got_initial_ipp_plan_callback, this);

  return true;
}

static core_trajectory_msgs::FixedTrajectory GetSquareFixedTraj() {
  core_trajectory_msgs::FixedTrajectory fixed_trajectory;
  fixed_trajectory.type = "Rectangle";
  diagnostic_msgs::KeyValue attrib1;
  attrib1.key = "frame_id";
  attrib1.value = "world";
  diagnostic_msgs::KeyValue attrib2;
  attrib2.key = "length";
  attrib2.value = "25";
  diagnostic_msgs::KeyValue attrib3;
  attrib3.key = "width";
  attrib3.value = "25";
  diagnostic_msgs::KeyValue attrib4;
  attrib4.key = "height";
  attrib4.value = "30";
  diagnostic_msgs::KeyValue attrib5;
  attrib5.key = "velocity";
  attrib5.value = "2.0";
  fixed_trajectory.attributes = {attrib1, attrib2, attrib3, attrib4, attrib5};
  return fixed_trajectory;
}

static core_trajectory_msgs::FixedTrajectory GetLawnmowerTraj() {
  core_trajectory_msgs::FixedTrajectory fixed_trajectory;
  fixed_trajectory.type = "Horizontal_Lawnmower";
  diagnostic_msgs::KeyValue attrib1;
  attrib1.key = "frame_id";
  attrib1.value = "world";
  diagnostic_msgs::KeyValue attrib2;
  attrib2.key = "length";
  attrib2.value = "100";
  diagnostic_msgs::KeyValue attrib3;
  attrib3.key = "width";
  attrib3.value = "100";
  diagnostic_msgs::KeyValue attrib4;
  attrib4.key = "height";
  attrib4.value = "30";
  diagnostic_msgs::KeyValue attrib5;
  attrib5.key = "velocity";
  attrib5.value = "2.0";
  diagnostic_msgs::KeyValue attrib6;
  attrib6.key = "stepover_dist";
  attrib6.value = "20.0";
  fixed_trajectory.attributes = {attrib1, attrib2, attrib3,
                                 attrib4, attrib5, attrib6};
  return fixed_trajectory;
}

bool BehaviorExecutive::execute() {
  // Reset behavior tree action
  if (reset_bt_action->is_active()) {
    if (reset_bt_action->active_has_changed()) {
      disable_pose_controller_output();

      for (int i = 0; i < conditions.size(); i++) {
        conditions[i]->set(false);
      }

      reset_bt_action->set_success();
      reset_bt_commanded_condition->set(false);
    }
  }

  // Set local position reference action
  if (set_local_pos_ref_action->is_active()) {
    if (set_local_pos_ref_action->active_has_changed()) {
      disable_pose_controller_output();

      if (!local_pos_ref_set_condition->get()) {
        // Only reset these conditions if this is the first time setting the local pos ref
        takeoff_complete_condition->set(false);
        landed_condition->set(false);
        request_control_commanded_condition->set(false);
        get_initial_ipp_plan_commanded->set(false);
        have_control_condition->set(false);
      }

      dji_sdk::SetLocalPosRef set_local_pos_ref_srv;
      set_local_pos_ref_client.call(set_local_pos_ref_srv);

      set_local_pos_ref_action->set_success();
      set_local_pos_ref_commanded_condition->set(false);
      local_pos_ref_set_condition->set(true);
    }
  }

  // request control action
  if (request_control_action->is_active()) {
    if (request_control_action->active_has_changed()) {
      disable_pose_controller_output();

      core_drone_interface::DroneCommand drone_command_srv;
      drone_command_srv.request.command =
          core_drone_interface::DroneCommand::Request::REQUEST_CONTROL;
      drone_command_client.call(drone_command_srv);

      if (drone_command_srv.response.success) {
        ROS_INFO("SUCCESS REQUEST CONTROL");
        request_control_action->set_success();
        have_control_condition->set(true);
      } else {
        ROS_INFO("FAILED REQUEST CONTROL");
        request_control_action->set_failure();
      }
      request_control_commanded_condition->set(false);
    }
  }

  // arm action
  if (arm_action->is_active()) {
    if (arm_action->active_has_changed()) {
      disable_pose_controller_output();

      // set takeoff and land conditions to false before arming
      takeoff_complete_condition->set(false);
      landed_condition->set(false);

      // arm
      core_drone_interface::DroneCommand drone_command_srv;
      drone_command_srv.request.command =
          core_drone_interface::DroneCommand::Request::ARM;
      drone_command_client.call(drone_command_srv);

      if (drone_command_srv.response.success) {
        ROS_INFO("ARMED SUCCESS");
        arm_action->set_success();
      } else {
        ROS_INFO("ARMED FAILURE");
        arm_action->set_failure();
      }
      arm_commanded_condition->set(false);
    }
  }

  // disarm action
  if (disarm_action->is_active()) {
    if (disarm_action->active_has_changed()) {
      disable_pose_controller_output();

      in_air_condition->set(false);
      core_drone_interface::DroneCommand drone_command_srv;
      drone_command_srv.request.command =
          core_drone_interface::DroneCommand::Request::DISARM;
      drone_command_client.call(drone_command_srv);

      if (drone_command_srv.response.success) {
        ROS_INFO("DISARMED SUCCESS");
        disarm_action->set_success();
      } else {
        ROS_INFO("DISARMED FAILURE");
        disarm_action->set_failure();
      }
      disarm_commanded_condition->set(false);
    }
  }

  if (idle_action->is_active()) {
    if (idle_action->active_has_changed()) {
      disable_pose_controller_output();
    }
  }

  // takeoff action
  if (takeoff_action->is_active()) {
    takeoff_action->set_running();
    in_air_condition->set(true);

    if (takeoff_action->active_has_changed()) {
      // Turn on pose controller output
      enable_pose_controller_output();

      core_takeoff_landing_planner::TakeoffLandingCommand takeoff_srv;
      takeoff_srv.request.command =
          core_takeoff_landing_planner::TakeoffLandingCommand::Request::TAKEOFF;
      takeoff_landing_client.call(takeoff_srv);
    }

    if (takeoff_state == "COMPLETE") {
      takeoff_complete_condition->set(true);
      takeoff_action->set_success();
    }
  }

  // land action
  if (land_action->is_active()) {
    land_action->set_running();

    if (land_action->active_has_changed()) {
      // Turn on pose controller output
      enable_pose_controller_output();

      core_takeoff_landing_planner::TakeoffLandingCommand land_srv;
      land_srv.request.command =
          core_takeoff_landing_planner::TakeoffLandingCommand::Request::LAND;
      takeoff_landing_client.call(land_srv);
    }

    if (landing_state == "COMPLETE") {
      landed_condition->set(true);
      land_action->set_success();
    }
  }

  // follow fixed trajectory action
  if (traj_control_action->is_active()) {
    traj_control_action->set_running();

    if (traj_control_action->active_has_changed()) {
      // Turn on pose controller output
      enable_pose_controller_output();

      core_trajectory_controller::TrajectoryMode srv;
      srv.request.mode =
          core_trajectory_controller::TrajectoryMode::Request::TRACK;
      trajectory_mode_client.call(srv);
      const auto fixed_trajectory = GetSquareFixedTraj();
      fixed_trajectory_pub.publish(fixed_trajectory);
    }
  }

  // follow coverage planner
  if (coverage_planner_action->is_active()) {
    coverage_planner_action->set_running();

    if (coverage_planner_action->active_has_changed()) {
      // Turn on pose controller output
      enable_pose_controller_output();

      core_trajectory_controller::TrajectoryMode srv;
      srv.request.mode =
          core_trajectory_controller::TrajectoryMode::Request::TRACK;
      trajectory_mode_client.call(srv);
      const auto fixed_trajectory = GetLawnmowerTraj();
      fixed_trajectory_pub.publish(fixed_trajectory);
    }
  }

  // follow ipp planner
  if (ipp_planner_action->is_active()) {
    ipp_planner_action->set_running();

    if (ipp_planner_action->active_has_changed()) {
      // Turn on pose controller output
      enable_pose_controller_output();

      core_trajectory_controller::TrajectoryMode srv;
      srv.request.mode =
          core_trajectory_controller::TrajectoryMode::Request::TRACK;
      trajectory_mode_client.call(srv);

      std_msgs::Bool true_msg;
      true_msg.data = true;
      execute_ipp_plan_pub.publish(true_msg);
    }
  } else {
    // Transitioned from active to inactive
    if (ipp_planner_action->active_has_changed()) {
      std_msgs::Bool false_msg;
      false_msg.data = false;
      execute_ipp_plan_pub.publish(false_msg);
    }
  }

  // Generate an initial ipp plan
  if (get_initial_ipp_plan_action->is_active()) {
    if (get_initial_ipp_plan_action->active_has_changed()) {
      std_msgs::Empty empty_msg;
      generate_ipp_plan_request_pub.publish(empty_msg);
      wait_for_initial_ipp_plan_pub.publish(empty_msg);

      for (int i = 0; i < autonomy_mode_conditions.size(); i++) {
        autonomy_mode_conditions[i]->set(false);
      }
      autonomy_idle_condition->set(true);
      get_initial_ipp_plan_commanded->set(false);
    }
  }

  for (int i = 0; i < conditions.size(); i++) conditions[i]->publish();
  for (int i = 0; i < actions.size(); i++)
    if (actions[i]->is_active()) actions[i]->publish();

  std_msgs::Bool in_air_msg;
  in_air_msg.data = in_air_condition->get();
  in_air_pub.publish(in_air_msg);

  return true;
}

void BehaviorExecutive::behavior_tree_command_callback(
    behavior_tree_msgs::BehaviorTreeCommands msg) {
  for (int i = 0; i < msg.commands.size(); i++) {
    std::string condition_name = msg.commands[i].condition_name;
    int status = msg.commands[i].status;

    for (int j = 0; j < conditions.size(); j++) {
      bt::Condition* condition = conditions[j];
      if (condition_name == condition->get_label()) {
        if (status == behavior_tree_msgs::Status::SUCCESS) {
          if (std::find(autonomy_mode_conditions.begin(),
                        autonomy_mode_conditions.end(),
                        condition) != autonomy_mode_conditions.end()) {
            for (int i = 0; i < autonomy_mode_conditions.size(); i++) {
              autonomy_mode_conditions[i]->set(false);
            }
          }
          condition->set(true);
        } else if (status == behavior_tree_msgs::Status::FAILURE) {
          condition->set(false);
        }
      }
    }
  }
}

void BehaviorExecutive::takeoff_state_callback(std_msgs::String msg) {
  takeoff_state = msg.data;
}

void BehaviorExecutive::landing_state_callback(std_msgs::String msg) {
  landing_state = msg.data;
}

void BehaviorExecutive::has_control_callback(std_msgs::Bool msg) {
  offboard_mode_condition->set(msg.data);
}

void BehaviorExecutive::got_initial_ipp_plan_callback(std_msgs::Bool msg) {
  got_initial_ipp_plan_condition->set(msg.data);
}

void BehaviorExecutive::reset_integrators() {
  std_srvs::Empty reset_srv;
  x_reset_integrator_client.call(reset_srv);
  y_reset_integrator_client.call(reset_srv);
  z_reset_integrator_client.call(reset_srv);
  yaw_reset_integrator_client.call(reset_srv);
  vx_reset_integrator_client.call(reset_srv);
  vy_reset_integrator_client.call(reset_srv);
  vz_reset_integrator_client.call(reset_srv);
  yawrate_reset_integrator_client.call(reset_srv);
}

void BehaviorExecutive::enable_pose_controller_output() {
  reset_integrators();

  std_srvs::SetBool publish_control_srv;
  publish_control_srv.request.data = true;
  publish_control_client.call(publish_control_srv);
}

void BehaviorExecutive::disable_pose_controller_output() {
  reset_integrators();

  std_srvs::SetBool publish_control_srv;
  publish_control_srv.request.data = false;
  publish_control_client.call(publish_control_srv);

  core_trajectory_controller::TrajectoryMode srv;
  srv.request.mode =
      core_trajectory_controller::TrajectoryMode::Request::ROBOT_POSE;
  trajectory_mode_client.call(srv);

  for (int i = 0; i < autonomy_mode_conditions.size(); i++) {
    autonomy_mode_conditions[i]->set(false);
  }
}

BehaviorExecutive::~BehaviorExecutive() {}

BaseNode* BaseNode::get() {
  BehaviorExecutive* core_behavior_executive =
      new BehaviorExecutive("BehaviorExecutive");
  return core_behavior_executive;
}
