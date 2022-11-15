

#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

// #include <tflib/tflib.h>
#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <grid_map_ros/grid_map_ros.hpp>
#include <vector>

#include "sensor_msgs/PointCloud2.h"

using namespace grid_map;

class TerrainMapping {
 public:
  TerrainMapping() : pnh("~"), map({"elevation"}) {
    cloud_sub = nh.subscribe("velodyne_points", 1000,
                             &TerrainMapping::cloudCallback, this);

    grid_map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1);

    map.setFrameId("/uav1/map");
    map.setGeometry(Length(1000, 1000), 0.5);
    map.add("elevation", grid_map::Matrix::Constant(2000, 2000, 0));
    ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
             map.getLength().x(), map.getLength().y(), map.getSize()(0),
             map.getSize()(1));

    map_pub_timer = nh.createTimer(ros::Duration(1.0),
                                   &TerrainMapping::publish_map_callback, this);
  }

 private:
  ros::NodeHandle nh;
  ros::NodeHandle pnh;
  ros::Subscriber cloud_sub;
  ros::Publisher grid_map_pub;
  ros::Timer map_pub_timer;

  GridMap map;

  tf::TransformListener listener;

  void cloudCallback(const sensor_msgs::PointCloud2& cloud) {
    // Convert all points to /uav1/map frame
    // Loop through all points and adjust our stored map accordingly
    // Publish map (probably do in separate timer)

    try {
      sensor_msgs::PointCloud2 cloud_target_frame;
      // tf::StampedTransform lidar_to_target_frame_transform;

      // listener.waitForTransform("uav1/map", "uav1/lidar", cloud.header.stamp,
      //                           ros::Duration(0.1));
      // listener.lookupTransform("uav1/map", "uav1/lidar", cloud.header.stamp,
      //                          lidar_to_target_frame_transform);

      pcl_ros::transformPointCloud("uav1/map", cloud, cloud_target_frame,
                                   listener);

      pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud2_map_pcl(
          new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud2_sensor_pcl(
          new pcl::PointCloud<pcl::PointXYZ>());

      pcl::fromROSMsg(cloud, *pointcloud2_sensor_pcl);
      pcl::fromROSMsg(cloud_target_frame, *pointcloud2_map_pcl);

      unsigned int size = (unsigned int)pointcloud2_map_pcl->points.size();

      for (unsigned int k = 0; k < size; ++k) {
        const pcl::PointXYZ& pt_cloud = pointcloud2_map_pcl->points[k];

        // check for invalid measurements
        if (isnan(pt_cloud.x) || isnan(pt_cloud.y) || isnan(pt_cloud.z))
          continue;

        const auto& sensor_x = pointcloud2_sensor_pcl->points[k].x;
        const auto& sensor_y = pointcloud2_sensor_pcl->points[k].y;
        const auto& sensor_z = pointcloud2_sensor_pcl->points[k].z;
        const float measurement_distance =
            sqrt(pow(sensor_x, 2) + pow(sensor_y, 2) + pow(sensor_z, 2));

        static constexpr float MAX_OBSERVABLE_DISTANCE = 100;
        if (MAX_OBSERVABLE_DISTANCE < measurement_distance) continue;
      }
    } catch (tf::TransformException& ex) {
    }
  }

  void publish_map_callback(const ros::TimerEvent& e) {
    ros::Time time = ros::Time::now();
    map.setTimestamp(time.toNSec());
    grid_map_msgs::GridMap message;
    GridMapRosConverter::toMessage(map, message);
    grid_map_pub.publish(message);
    ROS_INFO_THROTTLE(1.0, "Grid map (timestamp %f) published.",
                      message.info.header.stamp.toSec());
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "firefly_terrain_mapping");
  TerrainMapping node;
  ros::spin();
  return 0;
}