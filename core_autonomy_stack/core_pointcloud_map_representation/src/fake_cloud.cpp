#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

#include "ros/ros.h"

int main(int argc, char **argv) {
  ros::init(argc, argv, "fake_cloud");
  ros::NodeHandle n;
  ros::Publisher debug_pub =
      n.advertise<sensor_msgs::PointCloud2>("uav1/points", 10);
  ros::Rate loop_rate(10);
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_cloud{
      new pcl::PointCloud<pcl::PointXYZRGB>()};

  pcl::PointXYZRGB debug_point;
  debug_point.x = 0;
  debug_point.y = 30;
  debug_point.z = 30;
  debug_point.r = 0;
  debug_point.g = 0;
  debug_point.b = 255;

  debug_cloud->points.push_back(debug_point);

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