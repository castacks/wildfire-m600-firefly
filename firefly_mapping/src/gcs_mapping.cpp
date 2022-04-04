#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <chrono>
#include <std_msgs/Empty.h>


class GCSMapping {

public:
    GCSMapping() {
        new_fire_sub = nh.subscribe("new_fire_bins", 1000, &GCSMapping::new_fire_bins_callback, this);
        new_no_fire_sub = nh.subscribe("new_no_fire_bins", 1000, &GCSMapping::new_no_fire_bins_callback, this);
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("observed_firemap", 10);
        map_pub_timer = nh.createTimer(ros::Duration(1.0), &GCSMapping::publish_map_callback, this);
        clear_sub = nh.subscribe("clear_map", 1000, &GCSMapping::clear, this);

        outputMap.header.frame_id = "world";
        outputMap.info.resolution = 0.5;
        outputMap.info.width = 400; //Number of Cells
        outputMap.info.height = 400; //Number of Cells
        outputMap.info.origin.position.x = -100; //In meters
        outputMap.info.origin.position.y = -100; //In meters
        outputMap.data = std::vector<std::int8_t> (400*400, 50); // Initialize map to 50 percent certainty
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber new_fire_sub, new_no_fire_sub;
    ros::Publisher map_pub;
    ros::Timer map_pub_timer;
    ros::Subscriber clear_sub;

    nav_msgs::OccupancyGrid outputMap;

    bool new_update = true;

    void new_fire_bins_callback(const std_msgs::Int32MultiArray& msg) {
        for (int bin: msg.data) {
            outputMap.data[bin] = 100;
        }
        new_update = true;
    }

    void new_no_fire_bins_callback(const std_msgs::Int32MultiArray& msg) {
        for (int bin: msg.data) {
            outputMap.data[bin] = 0;
        }
        new_update = true;
    }

    void publish_map_callback(const ros::TimerEvent& e) {
        if (new_update) {
            map_pub.publish(outputMap);
            new_update = false;
        }
    }

    void clear(const std_msgs::Empty &empty_msg) {
        std::cout << "Clearing Map" << std::endl;
        outputMap.data = std::vector<std::int8_t> (400*400, 50); // Set map to 50 percent certainty
        map_pub.publish(outputMap);
    }

};
int main(int argc, char** argv) {
    ros::init(argc, argv, "gcs_mapping");
    GCSMapping node;
    ros::spin();
    return 0;
}