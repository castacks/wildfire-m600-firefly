//
// Created by kevin on 4/22/22.
//

#include "takeoff_land_test.h"

ros::ServiceClient drone_activation_service;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
sensor_msgs::NavSatFix gps_pos;
ros::Subscriber gps_pos_subscriber;

void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    gps_pos = *msg;
}

ServiceAck activate() {
    dji_sdk::Activation activation;
    drone_activation_service.call(activation);
    if (!activation.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", activation.response.cmd_set,
                 activation.response.cmd_id);
        ROS_WARN("ack.data: %i", activation.response.ack_data);
    }
    return ServiceAck(activation.response.result, activation.response.cmd_set,
                      activation.response.cmd_id, activation.response.ack_data);
}

ServiceAck obtainCtrlAuthority() {
    dji_sdk::SDKControlAuthority sdkAuthority;
    sdkAuthority.request.control_enable = 1;
    sdk_ctrl_authority_service.call(sdkAuthority);
    if (!sdkAuthority.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", sdkAuthority.response.cmd_set,
                 sdkAuthority.response.cmd_id);
        ROS_WARN("ack.data: %i", sdkAuthority.response.ack_data);
    }
    return ServiceAck(sdkAuthority.response.result, sdkAuthority.response.cmd_set,
                      sdkAuthority.response.cmd_id,
                      sdkAuthority.response.ack_data);
}

ServiceAck takeoff() {
    dji_sdk::DroneTaskControl droneTaskControl;
    droneTaskControl.request.task = 4;
    drone_task_service.call(droneTaskControl);
    if (!droneTaskControl.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set,
                 droneTaskControl.response.cmd_id);
        ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
    }
    return ServiceAck(
            droneTaskControl.response.result, droneTaskControl.response.cmd_set,
            droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data);
}

ServiceAck land() {
    dji_sdk::DroneTaskControl droneTaskControl;
    droneTaskControl.request.task = 6;
    drone_task_service.call(droneTaskControl);
    if (!droneTaskControl.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", droneTaskControl.response.cmd_set,
                 droneTaskControl.response.cmd_id);
        ROS_WARN("ack.data: %i", droneTaskControl.response.ack_data);
    }
    return ServiceAck(
            droneTaskControl.response.result, droneTaskControl.response.cmd_set,
            droneTaskControl.response.cmd_id, droneTaskControl.response.ack_data);
}

int main(int argc, char **argv) {
    ros::init(argc, argv, "sdk_demo_mission");
    ros::NodeHandle nh;
    drone_activation_service =
            nh.serviceClient<dji_sdk::Activation>("dji_sdk/activation");
    sdk_ctrl_authority_service = nh.serviceClient<dji_sdk::SDKControlAuthority>(
            "dji_sdk/sdk_control_authority");
    drone_task_service =
            nh.serviceClient<dji_sdk::DroneTaskControl>("dji_sdk/drone_task_control");
    gps_pos_subscriber = nh.subscribe<sensor_msgs::NavSatFix>(
            "dji_sdk/gps_position", 10, &gpsPosCallback);

    // Activate
    if (activate().result) {
        ROS_INFO("Activated successfully");
    } else {
        ROS_WARN("Failed activation");
        return -1;
    }

    // Obtain Control Authority
    ServiceAck ack = obtainCtrlAuthority();
    if (ack.result) {
        ROS_INFO("Obtain SDK control Authority successfully");
    } else {
        if (ack.ack_data == 3 && ack.cmd_set == 1 && ack.cmd_id == 0) {
            ROS_INFO("Obtain SDK control Authority in progess, "
                     "send the cmd again");
            obtainCtrlAuthority();
        } else {
            ROS_WARN("Failed Obtain SDK control Authority");
            return -1;

        }
    }

    ros::spinOnce();

    // Takeoff
    if (takeoff().result) {
        ROS_INFO("Takeoff command sent successfully");
    } else {
        ROS_WARN("Failed sending takeoff command");
        ros::spin();
    }
    ros::Duration(15).sleep();

    ROS_INFO("land");
    if (land().result) {
        ROS_INFO("Land command sent successfully");
    } else {
        ROS_WARN("Failed sending land command");
        ros::spin();
    }

    ros::spin();

}
