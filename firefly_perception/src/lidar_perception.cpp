/********************************************************************************
 * Project FireFly : 2022 MRSD - Team D                                         *
 * Authors: Arjun Chauhan, Kevin Gmelin, Sabrina Shen, Manuj Trehan and Akshay Venkatesh      *
 *                                                                              *
 * ThermalImageReader : Thermal camera data processing module                   *
 *                                                                              *
 * Reads thermal camera image topic, applies thresholding for fire detection,   *
 * publishes segmented image with current pose from which image was taken       *
 *                                                                              *
 * Created:  03 Apr 2022                                                        *
********************************************************************************/

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <vector>
#include <algorithm>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <firefly_mapping/ImageWithPose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/crop_box.h>
#include <pcl_ros/transforms.h>
#include "pcl_ros/point_cloud.h"




class LidarReader 
{
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    sensor_msgs::PointCloud2 input_pointcloud_;

    ros::Publisher lidar_mapping_pub_;
    ros::Publisher lidar_obstacle_pub_;
    ros::Publisher lidar_altitude_pub_;
    ros::Publisher altitude_pub_;

    ros::Subscriber lidar_subscriber;

    int len = 10;
    // std::vector<float> height_vec(10);

public:
    LidarReader() : private_nh_("~")
    {
        lidar_subscriber = nh_.subscribe("velodyne_points", 1,
                                   &LidarReader::point_cloud_extractor, this);
        
        lidar_mapping_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZ>>("lidar_cropped_mapping", 1);
        lidar_obstacle_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZ>>("lidar_cropped_obstacle", 1);
        lidar_altitude_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZ>>("altitude", 1);
        altitude_pub_ = nh_.advertise<std_msgs::Float64>("perception_height_estimation", 2);

// ros::Publisher publishingObj = nodeHandler.advertise<std_msgs::Float64>("floating_numbers", 2);

    }

    ~LidarReader()
    {
        std::cout<<"GET GOT. BYE LIDAR \n";
    }

    void point_cloud_extractor(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PCLPointCloud2 pcl_pc2;

        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);

        /*
            given min and max of 0 to 130, assume c-cw/cw??
            priority should be mapping. FOV angle range : TBD #TODO based on camera fov and mounting of camera
            camera fov : 105* horizontal 75* vertical

            dimension of 2d point cloud is 16 (rings vertically) x 1824 (in a ring covering 360*)
            assuming that the lidar's 0* is in line with the horizon
                1. crop out 0 - 40* for obstacle avoidance  (points[:, :203])
                2. crop out 40 - 130* for mapping node  (points[:, 204 : 658])
        */

        std::vector<std::vector<uint8_t>> mapping_point_cloud;
        std::vector<std::vector<uint8_t>> obstacle_point_cloud;
        
        // (x, y, z)
        // x is depth
        // y is breadth of scan (i think its in radians) -ve value is towards right
        // z is vertical axis (i.e. number of lines of scan)    
        pcl::PCLPointCloud2 point_cloud2_mappping;
        pcl::CropBox<pcl::PointXYZ> cropBoxFilterMapping (true);
        cropBoxFilterMapping.setInputCloud (temp_cloud);    
        Eigen::Vector4f min_pt_mapping (-1.0f, 0.0f, -50.0f, 1.0f);
        Eigen::Vector4f max_pt_mapping (1.0f, 100.0f, 50.0f, 1.0f);
        pcl::PointCloud<pcl::PointXYZ> cloud_out_mapping;
        cropBoxFilterMapping.setMin (min_pt_mapping);
        cropBoxFilterMapping.setMax (max_pt_mapping);
        cropBoxFilterMapping.filter (cloud_out_mapping);

        pcl::toPCLPointCloud2(cloud_out_mapping, point_cloud2_mappping);
        point_cloud2_mappping.header.frame_id = "/uav1/lidar";


        lidar_mapping_pub_.publish(point_cloud2_mappping);

        // pcl::CropBox<pcl::PointXYZ> cropBoxFilterObstacle (true);
        // cropBoxFilterObstacle.setInputCloud (temp_cloud);
        // Eigen::Vector4f min_pt_obstacle (0.0f, -2.0f, -10.0f, 1.0f);
        // Eigen::Vector4f max_pt_obstacle (100.0f, 1.0f, 10.0f, 1.0f);
        // pcl::PointCloud<pcl::PointXYZ> cloud_out_obstacle;
        // cropBoxFilterObstacle.filter (cloud_out_obstacle);
        // lidar_obstacle_pub_.publish(cloud_out_obstacle);

        pcl::CropBox<pcl::PointXYZ> cropBoxFilterAltitude (true);
        cropBoxFilterAltitude.setInputCloud (temp_cloud);
        Eigen::Vector4f min_pt_altitude (-0.1f, 0.0f, -0.1f, 1.0f);
        Eigen::Vector4f max_pt_altitude (0.1f, 100.0f, 0.1f, 1.0f);
        pcl::PointCloud<pcl::PointXYZ> cloud_out_altitude;
        cropBoxFilterAltitude.setMin (min_pt_altitude);
        cropBoxFilterAltitude.setMax (max_pt_altitude);
        cropBoxFilterAltitude.filter (cloud_out_altitude);
        cloud_out_altitude.header.frame_id = "/uav1/lidar";
        lidar_altitude_pub_.publish(cloud_out_altitude);
        
        float height;
        for (int i = 0; i < cloud_out_altitude.points.size(); i++)
        {
            height += cloud_out_altitude.points[i].y;
        }
        
        height /= cloud_out_altitude.points.size();
        // height_vec.push_back(height)

        //take avg over all measurements in time
        float height_publish = 0;
        // for (auto i : height_vec)
        // {
        //     height_publish += i;
        // }

        height_publish = height;

        if (height_publish != height_publish)
            height_publish = -1;

        std_msgs::Float64 altitude;
        altitude.data = height_publish;

        altitude_pub_.publish(altitude);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LidarNode");

    // ThermalImageReader ic;
    LidarReader ic; 

    ros::spin();
    return 0;
}
