#include <simple_waypoint_test.h>

using namespace DJI::OSDK;

// global variables
ros::ServiceClient waypoint_upload_service;
ros::ServiceClient waypoint_action_service;
ros::ServiceClient drone_activation_service;
ros::ServiceClient sdk_ctrl_authority_service;
ros::ServiceClient drone_task_service;
sensor_msgs::NavSatFix gps_pos;
ros::Subscriber gps_pos_subscriber;

void gpsPosCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    gps_pos = *msg;
}

bool runWaypointMission(uint8_t numWaypoints, int responseTimeout) {
    ros::spinOnce();

    // Takeoff
    if (takeoff().result) {
        ROS_INFO("Takeoff command sent successfully");
    } else {
        ROS_WARN("Failed sending takeoff command");
        return false;
    }
    ros::Duration(15).sleep();

    // Waypoint Mission : Initialization
    dji_sdk::MissionWaypointTask waypointTask;
    setWaypointInitDefaults(waypointTask);

    // Waypoint Mission: Create Waypoints
    float64_t increment = 0.000001 / C_PI * 180;
    float32_t start_alt = 15;
    ROS_INFO("Creating Waypoints..\n");
    std::vector<WayPointSettings> generatedWaypts =
            createWaypoints(numWaypoints, increment, start_alt);

    // Waypoint Mission: Upload the waypoints
    ROS_INFO("Uploading Waypoints..\n");
    uploadWaypoints(generatedWaypts, responseTimeout, waypointTask);

    // Waypoint Mission: Init mission
    ROS_INFO("Initializing Waypoint Mission..\n");
    if (initWaypointMission(waypointTask).result) {
        ROS_INFO("Waypoint upload command sent successfully");
    } else {
        ROS_WARN("Failed sending waypoint upload command");
        return false;
    }

    // Waypoint Mission: Start
    if (missionAction(DJI_MISSION_TYPE::WAYPOINT,
                      MISSION_ACTION::START)
            .result) {
        ROS_INFO("Mission start command sent successfully");
    } else {
        ROS_WARN("Failed sending mission start command");
        return false;
    }

    return true;
}

void setWaypointDefaults(WayPointSettings *wp) {
    wp->damping = 0;
    wp->yaw = 0;
    wp->gimbalPitch = 0;
    wp->turnMode = 0;
    wp->hasAction = 0;
    wp->actionTimeLimit = 100;
    wp->actionNumber = 0;
    wp->actionRepeat = 0;
    for (int i = 0; i < 16; ++i) {
        wp->commandList[i] = 0;
        wp->commandParameter[i] = 0;
    }
}

void setWaypointInitDefaults(dji_sdk::MissionWaypointTask &waypointTask) {
    waypointTask.velocity_range = 10;
    waypointTask.idle_velocity = 2;
    waypointTask.action_on_finish = dji_sdk::MissionWaypointTask::FINISH_RETURN_TO_HOME;
    waypointTask.mission_exec_times = 1;
    waypointTask.yaw_mode = dji_sdk::MissionWaypointTask::YAW_MODE_LOCK;
    waypointTask.trace_mode = dji_sdk::MissionWaypointTask::TRACE_POINT;
    waypointTask.action_on_rc_lost = dji_sdk::MissionWaypointTask::ACTION_FREE;
    waypointTask.gimbal_pitch_mode = dji_sdk::MissionWaypointTask::GIMBAL_PITCH_FREE;
}

std::vector<DJI::OSDK::WayPointSettings> createWaypoints(int numWaypoints, float64_t distanceIncrement,
                                                         float32_t start_alt) {
    // Create Start Waypoint
    WayPointSettings start_wp;
    setWaypointDefaults(&start_wp);
    start_wp.latitude = gps_pos.latitude;
    start_wp.longitude = gps_pos.longitude;
    start_wp.altitude = start_alt;
    ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n", gps_pos.latitude,
             gps_pos.longitude, start_alt);

    std::vector<DJI::OSDK::WayPointSettings> wpVector =
            generateWaypointsPolygon(&start_wp, distanceIncrement, numWaypoints);
    return wpVector;
}

std::vector<DJI::OSDK::WayPointSettings> generateWaypointsPolygon(WayPointSettings *start_data, float64_t increment,
                                                                  int num_wp) {
    // Let's create a vector to store our waypoints in.
    std::vector<DJI::OSDK::WayPointSettings> wp_list;

    // Some calculation for the polygon
    float64_t extAngle = 2 * M_PI / num_wp;

    // First waypoint
    start_data->index = 0;
    wp_list.push_back(*start_data);

    // Iterative algorithm
    for (int i = 1; i < num_wp; i++) {
        WayPointSettings wp;
        WayPointSettings *prevWp = &wp_list[i - 1];
        setWaypointDefaults(&wp);
        wp.index = i;
        wp.latitude = (prevWp->latitude + (increment * cos(i * extAngle)));
        wp.longitude = (prevWp->longitude + (increment * sin(i * extAngle)));
        wp.altitude = (prevWp->altitude + 1);
        wp_list.push_back(wp);
    }

    // Come back home
    start_data->index = num_wp;
    wp_list.push_back(*start_data);

    return wp_list;
}

void uploadWaypoints(std::vector<DJI::OSDK::WayPointSettings> &wp_list,
                     int responseTimeout, dji_sdk::MissionWaypointTask &waypointTask) {
    dji_sdk::MissionWaypoint waypoint;
    for (std::vector<WayPointSettings>::iterator wp = wp_list.begin();
         wp != wp_list.end(); ++wp) {
        ROS_INFO("Waypoint created at (LLA): %f \t%f \t%f\n ", wp->latitude,
                 wp->longitude, wp->altitude);
        waypoint.latitude = wp->latitude;
        waypoint.longitude = wp->longitude;
        waypoint.altitude = wp->altitude;
        waypoint.damping_distance = 0;
        waypoint.target_yaw = 0;
        waypoint.target_gimbal_pitch = 0;
        waypoint.turn_mode = 0;
        waypoint.has_action = 0;
        waypointTask.mission_waypoint.push_back(waypoint);
    }
}


ServiceAck initWaypointMission(dji_sdk::MissionWaypointTask &waypointTask) {
    dji_sdk::MissionWpUpload missionWpUpload;
    missionWpUpload.request.waypoint_task = waypointTask;
    waypoint_upload_service.call(missionWpUpload);
    if (!missionWpUpload.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", missionWpUpload.response.cmd_set,
                 missionWpUpload.response.cmd_id);
        ROS_WARN("ack.data: %i", missionWpUpload.response.ack_data);
    }
    return ServiceAck(
            missionWpUpload.response.result, missionWpUpload.response.cmd_set,
            missionWpUpload.response.cmd_id, missionWpUpload.response.ack_data);
}

ServiceAck missionAction(DJI::OSDK::DJI_MISSION_TYPE type,
                         DJI::OSDK::MISSION_ACTION action) {
    dji_sdk::MissionWpAction missionWpAction;
    missionWpAction.request.action = action;
    waypoint_action_service.call(missionWpAction);
    if (!missionWpAction.response.result) {
        ROS_WARN("ack.info: set = %i id = %i", missionWpAction.response.cmd_set,
                 missionWpAction.response.cmd_id);
        ROS_WARN("ack.data: %i", missionWpAction.response.ack_data);
    }
    return {static_cast<bool>(missionWpAction.response.result),
            missionWpAction.response.cmd_set,
            missionWpAction.response.cmd_id,
            missionWpAction.response.ack_data};


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

    // ROS stuff
    waypoint_upload_service = nh.serviceClient<dji_sdk::MissionWpUpload>(
            "dji_sdk/mission_waypoint_upload");
    waypoint_action_service = nh.serviceClient<dji_sdk::MissionWpAction>(
            "dji_sdk/mission_waypoint_action");

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

    // Setup variables for use
    uint8_t wayptPolygonSides;
    int responseTimeout = 1;

    // Waypoint call
    wayptPolygonSides = 6;
    runWaypointMission(wayptPolygonSides, responseTimeout);

    ros::spin();

    return 0;
}
