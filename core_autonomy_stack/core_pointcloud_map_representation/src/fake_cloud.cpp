#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "fake_cloud");
  ros::NodeHandle n;
  ros::Publisher debug_pub =
      n.advertise<sensor_msgs::PointCloud2>("uav1/velodyne_points", 10);
  ros::Rate loop_rate(10);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_cloud{
      new pcl::PointCloud<pcl::PointXYZRGB>()};

  pcl::PointXYZRGB debug_point1;
  debug_point1.x = 0;
  debug_point1.y = 10;
  debug_point1.z = 30;
  debug_point1.r = 0;
  debug_point1.g = 0;
  debug_point1.b = 255;
  debug_cloud->points.push_back(debug_point1);

    pcl::PointXYZRGB debug_point2;
  debug_point2.x = 0;
  debug_point2.y = 10;
  debug_point2.z = 29;
  debug_point2.r = 0;
  debug_point2.g = 0;
  debug_point2.b = 255;
  debug_cloud->points.push_back(debug_point2);

    pcl::PointXYZRGB debug_point3;
  debug_point3.x = 0;
  debug_point3.y = 10;
  debug_point3.z = 31;
  debug_point3.r = 0;
  debug_point3.g = 0;
  debug_point3.b = 255;
  debug_cloud->points.push_back(debug_point3);

    pcl::PointXYZRGB debug_point4;
  debug_point4.x = -1;
  debug_point4.y = 10;
  debug_point4.z = 30;
  debug_point4.r = 0;
  debug_point4.g = 0;
  debug_point4.b = 255;
  debug_cloud->points.push_back(debug_point4);

    pcl::PointXYZRGB debug_point5;
  debug_point5.x = 1;
  debug_point5.y = 10;
  debug_point5.z = 30;
  debug_point5.r = 0;
  debug_point5.g = 0;
  debug_point5.b = 255;
  debug_cloud->points.push_back(debug_point5);

  while (ros::ok()) {
    sensor_msgs::PointCloud2 debug_cloud2;
    pcl::toROSMsg(*debug_cloud, debug_cloud2);
    debug_cloud2.header.frame_id = "uav1/map";
    debug_pub.publish(debug_cloud2);

    ros::spinOnce();

    loop_rate.sleep();
  }

  return 0;
}