//
// Created by kevin on 4/22/22.
//

#ifndef FIREFLY_CONTROL_TAKEOFF_LAND_TEST_H
#define FIREFLY_CONTROL_TAKEOFF_LAND_TEST_H
// System includes
#include "unistd.h"
#include <iostream>

// DJI SDK includes
#include <dji_sdk/Activation.h>
#include <dji_sdk/DroneTaskControl.h>
#include <dji_sdk/SDKControlAuthority.h>
#include <dji_sdk/dji_sdk.h>

// SDK core library
#include <djiosdk/dji_vehicle.hpp>

// ROS includes
#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>

#define C_EARTH (double)6378137.0
#define C_PI (double)3.141592653589793
#define DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))

typedef struct ServiceAck
{
    bool         result;
    int          cmd_set;
    int          cmd_id;
    unsigned int ack_data;
    ServiceAck(bool res, int set, int id, unsigned int ack)
            : result(res)
            , cmd_set(set)
            , cmd_id(id)
            , ack_data(ack)
    {
    }
    ServiceAck()
    {
    }
} ServiceAck;

ServiceAck activate();

ServiceAck obtainCtrlAuthority();

ServiceAck takeoff();

ServiceAck land();

void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);

#endif //FIREFLY_CONTROL_TAKEOFF_LAND_TEST_H
