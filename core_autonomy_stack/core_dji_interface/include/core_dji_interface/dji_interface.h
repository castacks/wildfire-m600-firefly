#ifndef _DJI_INTERFACE_H_
#define _DJI_INTERFACE_H_

#include <core_drone_interface/drone_interface.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <nav_msgs/Odometry.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/DroneArmControl.h>
#include <dji_sdk/SetLocalPosRef.h>
#include <sensor_msgs/Joy.h>
#include <tf/transform_listener.h>
#include <tf/transform_datatypes.h>

class DJIInterface : public DroneInterface {
private:

  // constants for DJI, taken from: http://wiki.ros.org/dji_sdk#Details_on_flight_control_setpoint
  // Commands                                     Frame         Limits
  // horizontal
  uint8_t COMMAND_ROLL_PITCH = 0x00;            // Ground/Body  0.611 rad (35 deg)
  uint8_t COMMAND_HORIZONTAL_VELOCITIES = 0x40; // Ground/Body  +/- 30 m/s
  uint8_t COMMAND_POSITION_OFFSETS = 0x80;      // Ground/Body  N/A
  uint8_t COMMAND_ANGULAR_RATES = 0xC0;         // Ground/Body  5/6pi rad/s
  // vertical
  uint8_t COMMAND_VERTICAL_VELOCITY = 0x00;     // Ground       +/- 5 m/s
  uint8_t COMMAND_ALTITUDE = 0x10;              // Ground       0 to 120 m
  uint8_t COMMAND_THRUST = 0x20;                // Body         0% to 100%
  // yaw
  uint8_t COMMAND_YAW_ANGLE = 0x00;             // Ground       -pi to pi rad
  uint8_t COMMAND_YAW_RATE = 0x08;              // Ground       5/6pi rad/s
  // coordinate frame
  uint8_t GROUND_ENU_FRAME = 0x00;
  uint8_t BODY_FLU_FRAME = 0x02;

  // parameters
  std::string stabilized_body_frame, world_frame;
  
  // variables
  bool armed;
  bool has_offboard_control;

  // publishers
  ros::Publisher setpoint_generic_pub;
  
  // services
  ros::ServiceClient sdk_control_authority_client;
  ros::ServiceClient drone_arm_control_client;
  ros::ServiceClient set_local_pos_ref_client;
  
  // subcribers
  tf::TransformListener* listener;

  // callbacks
  
public:
  DJIInterface();
  
  virtual bool request_control();
  virtual bool arm();
  virtual bool disarm();
  virtual bool is_armed();
  virtual bool has_control();
  
  virtual void command_velocity(geometry_msgs::TwistStamped msg);
  virtual void command_pose(geometry_msgs::PoseStamped msg);
  virtual void command_roll_pitch_yawrate_thrust(mav_msgs::RollPitchYawrateThrust);
};



#endif
