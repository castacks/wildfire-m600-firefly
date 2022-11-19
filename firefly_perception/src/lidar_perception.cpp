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

public:
    LidarReader() : private_nh_("~")
    {
        lidar_subscriber = nh_.subscribe("velodyne_points", 1, &LidarReader::point_cloud_extractor, this);
        
        lidar_mapping_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZ>>("lidar_cropped_mapping", 1);
        lidar_obstacle_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZ>>("lidar_cropped_obstacle", 1);
        lidar_altitude_pub_ = nh_.advertise< pcl::PointCloud<pcl::PointXYZ>>("altitude", 1);
        altitude_pub_ = nh_.advertise<std_msgs::Float64>("perception_height_estimation", 2);
    }

    ~LidarReader()
    {
        ROS_DEBUG("GET GOT. BYE LIDAR");
    }

    void point_cloud_extractor(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_pcl(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromROSMsg(*msg, *cloud_pcl);
        
        pcl::PCLPointCloud2 point_cloud2_mappping;
        pcl::CropBox<pcl::PointXYZ> cropBoxFilterMapping (true);
        cropBoxFilterMapping.setInputCloud (cloud_pcl);    
        Eigen::Vector4f min_pt_mapping (-20.0f, 0.0f, -100.0f, 1.0f);
        Eigen::Vector4f max_pt_mapping (20.0f, 100.0f, 100.0f, 1.0f);
        pcl::PointCloud<pcl::PointXYZ> cloud_out_mapping;
        cropBoxFilterMapping.setMin (min_pt_mapping);
        cropBoxFilterMapping.setMax (max_pt_mapping);
        cropBoxFilterMapping.filter (cloud_out_mapping);
        pcl::toPCLPointCloud2(cloud_out_mapping, point_cloud2_mappping);
        point_cloud2_mappping.header.frame_id = "/uav1/lidar";

        lidar_mapping_pub_.publish(point_cloud2_mappping);

        pcl::CropBox<pcl::PointXYZ> cropBoxFilterAltitude (true);
        cropBoxFilterAltitude.setInputCloud (cloud_pcl);
        Eigen::Vector4f min_pt_altitude (-0.1f, 0.0f, -0.1f, 1.0f);
        Eigen::Vector4f max_pt_altitude (0.1f, 100.0f, 0.1f, 1.0f);
        pcl::PointCloud<pcl::PointXYZ> cloud_out_altitude;
        cropBoxFilterAltitude.setMin (min_pt_altitude);
        cropBoxFilterAltitude.setMax (max_pt_altitude);
        cropBoxFilterAltitude.filter (cloud_out_altitude);
        cloud_out_altitude.header.frame_id = "/uav1/lidar";
        lidar_altitude_pub_.publish(cloud_out_altitude);
        
        float height = 0;
        for (int i = 0; i < cloud_out_altitude.points.size(); i++)
        {
            height += cloud_out_altitude.points[i].y;
        }
        
        if (cloud_out_altitude.points.size() != 0 ) {
            height /= cloud_out_altitude.points.size();
        }
        else {
            height = -1;
        }
            
        std_msgs::Float64 altitude;
        altitude.data = height;
        altitude_pub_.publish(altitude);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "LidarPerception");
    LidarReader ic; 
    ros::spin();
    return 0;
}

