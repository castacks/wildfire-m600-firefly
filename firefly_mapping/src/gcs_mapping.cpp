/********************************************************************************
 * Project FireFly : 2022 MRSD - Team D                                         *
 * Authors: Arjun Chauhan, Kevin Gmelin, Sabrina Shen, Manuj Trehan and Akshay Venkatesh      *
 *                                                                              *
 * GCSMapping : Ground Control Station Fire Map Generation Module               *
 *                                                                              *
 * Receives map updates from UAV over MAVLink, updates occupancy grid and       *
 * publishes updated map to be subscribed to for visualization on RViz          *
 *                                                                              *
 * Created:  03 Apr 2022                                                        *
********************************************************************************/

#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int32MultiArray.h>
#include <chrono>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Pose.h>
#include <eigen_conversions/eigen_msg.h>



class GCSMapping {

public:
    GCSMapping(): pnh("~") {
        new_fire_sub = nh.subscribe("new_fire_bins", 1000, &GCSMapping::new_fire_bins_callback, this);
        new_no_fire_sub = nh.subscribe("new_no_fire_bins", 1000, &GCSMapping::new_no_fire_bins_callback, this);
        init_to_no_fire_with_pose_sub = nh.subscribe("init_to_no_fire_with_pose_bins", 1000, &GCSMapping::init_to_no_fire_with_pose_bins_callback, this);
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("observed_firemap", 10);
        map_pub_timer = nh.createTimer(ros::Duration(1.0), &GCSMapping::publish_map_callback, this);
        clear_sub = nh.subscribe("clear_map", 1000, &GCSMapping::clear, this);

        pnh.param<float>("resolution", resolution, 0.5);  
        pnh.param<float>("min_x", minX, -100.0);  
        pnh.param<float>("max_x", maxX, 100.0);  
        pnh.param<float>("min_y", minY, -100.0);  
        pnh.param<float>("max_y", maxY, 100.0);  

        mapWidth = int((maxX - minX) / resolution);
        mapHeight = int((maxY - minY) / resolution);

        outputMap.header.frame_id = "world";
        outputMap.info.resolution = 0.5;
        outputMap.info.width = mapWidth; //Number of Cells
        outputMap.info.height = mapHeight; //Number of Cells
        outputMap.info.origin.position.x = minX; //In meters
        outputMap.info.origin.position.y = minY; //In meters
        outputMap.data = std::vector<std::int8_t> (mapWidth * mapHeight, 50); // Initialize map to 50 percent certainty

        K_inv << 1.0/fx,  0.0,    -cx/fx,
                0.0,     1.0/fy, -cy/fy,
                0.0,     0.0,     1.0;
    }

private:
    ros::NodeHandle nh;
    ros::NodeHandle pnh;
    ros::Subscriber new_fire_sub, new_no_fire_sub, init_to_no_fire_with_pose_sub;
    ros::Publisher map_pub;
    ros::Timer map_pub_timer;
    ros::Subscriber clear_sub;

    nav_msgs::OccupancyGrid outputMap;

    bool new_update = true;

    Eigen::Vector3d ground_normal{0, 0, 1}; //Should point up from ground - if pointing into ground, will cause errors
    float ground_offset = 0;

    float fx = 338.136183;
    float fy = 336.039570;
    float cx = 160.829;
    float cy = 112.614;

    float resolution;
    float minX;
    float maxX;
    float minY;
    float maxY;
    int mapWidth; // Number of cells
    int mapHeight; // Number of cells

    long mapped_bins = 0;

    Eigen::Matrix3d K_inv;

    void new_fire_bins_callback(const std_msgs::Int32MultiArray& msg) {
        for (int bin: msg.data) {
            if (outputMap.data[bin] == 50) {
                mapped_bins++;
            }
            outputMap.data[bin] = 100;
        }
        new_update = true;
    }

    void new_no_fire_bins_callback(const std_msgs::Int32MultiArray& msg) {
        for (int bin: msg.data) {
            if (outputMap.data[bin] == 50) {
                mapped_bins++;
            }
            outputMap.data[bin] = 0;
        }
        new_update = true;
    }

    void init_to_no_fire_with_pose_bins_callback(const geometry_msgs::Pose& msg) {
        Eigen::Quaterniond cam_quat;
        tf::quaternionMsgToEigen(msg.orientation, cam_quat);

        Eigen::Matrix3d pixelToRay = cam_quat.matrix() * K_inv;
        Eigen::Vector3d camCenter {msg.position.x, msg.position.y, msg.position.z};

        float camCenterGroundDist = ground_offset - ground_normal.dot(camCenter);

        for (size_t i = 0; i < 320; i++) {
            for (size_t j = 0; j < 240; j++) {
                Eigen::Vector3d pixelHomogenous{i, j, 1};
                Eigen::Vector3d rayDir = pixelToRay * pixelHomogenous;

                float rayPerpendicularComponent = ground_normal.dot(rayDir);
                if (rayPerpendicularComponent >= 0) {
                    continue;
                }

                float rayLambda = camCenterGroundDist / rayPerpendicularComponent;

                Eigen::Vector3d intersect = camCenter + rayLambda * rayDir;

                if (intersect(0) > maxX
                    || intersect(0) < minX
                    || intersect(1) > maxY
                    || intersect(1) < minY) {
                    continue;
                }

                size_t gridRow = (size_t) ((intersect(1)-minY)/resolution);
                size_t gridCol = (size_t) ((intersect(0)-minX)/resolution);

                int mapBin = gridCol + gridRow * outputMap.info.width;
                if (outputMap.data[mapBin] == 50) {
                    outputMap.data[mapBin] = 0;
                    mapped_bins++;
                }

            }
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
        ROS_INFO("Clearing Map");
        outputMap.data = std::vector<std::int8_t> (mapWidth * mapHeight, 50); // Set map to 50 percent certainty
        map_pub.publish(outputMap);
        mapped_bins = 0;
    }

};
int main(int argc, char** argv) {
    ros::init(argc, argv, "gcs_mapping");
    GCSMapping node;
    ros::spin();
    return 0;
}