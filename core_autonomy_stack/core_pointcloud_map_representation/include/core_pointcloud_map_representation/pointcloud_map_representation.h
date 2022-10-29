#ifndef _POINTCLOUD_MAP_REPRESENTATION_H_
#define _POINTCLOUD_MAP_REPRESENTATION_H_

//#include <map_representation_interface/map_representation_interface.h>
#include <core_map_representation_interface/map_representation.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Range.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf/transform_listener.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <vector>

struct PointCloudNode {
  pcl::KdTreeFLANN<pcl::PointXYZ>::Ptr kdtree;
  pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
  nav_msgs::Odometry odom;
  tf::Transform target_to_lidar_frame_transform;
  
  bool range_up_valid, range_down_valid;
  tf::Vector3 range_point_up, range_point_down;
  double up_radius, down_radius;
};

class PointCloudMapRepresentation : public MapRepresentation {
 private:
  // parameters
  double node_spacing;
  int node_limit;
  float node_decay_time;
  std::string target_frame, lidar_frame;
  double lidar_vertical_fov;
  double robot_radius;
  int obstacle_check_points;
  double obstacle_check_radius;

  // state estimate
  bool got_odom;
  nav_msgs::Odometry odom;

  // ranges
  bool range_up_valid, range_down_valid;
  ros::Time range_up_time, range_down_time;
  tf::Vector3 range_point_up, range_point_down;
  double up_radius, down_radius;

  // visualization
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr debug_cloud;
  visualization_msgs::MarkerArray markers;
  visualization_msgs::Marker points;

  // node buffer
  bool should_add_node;
  std::vector<PointCloudNode> node_buffer;
  PointCloudNode current_node;

  // subscribers
  ros::Subscriber cloud_sub, odom_sub, range_up_sub, range_down_sub;
  tf::TransformListener* listener;

  // publishers
  ros::Publisher debug_pub, pose_graph_pub, cloud_map_pub;

  // callbacks
  void cloud_callback(sensor_msgs::PointCloud2 cloud);
  void odom_callback(nav_msgs::Odometry odom);
  void range_up_callback(sensor_msgs::Range range);
  void range_down_callback(sensor_msgs::Range range);
  
  std::vector<PointCloudNode> get_all_nodes();
  bool get_point_in_frame(std::string target_frame, std::string frame_id, ros::Time time, tf::Vector3 point_range_frame, tf::Vector3* point_target_frame);
  
 public:
  PointCloudMapRepresentation();//ros::NodeHandle* nh);
  virtual double distance_to_obstacle(geometry_msgs::PoseStamped pose, tf::Vector3 direction);
  virtual tf::Vector3 distance_vector_to_obstacle(geometry_msgs::PoseStamped pose, tf::Vector3 direction);
  virtual void publish_debug();

  //virtual std::vector<std::vector<double> > get_values(std::vector<core_trajectory_msgs::TrajectoryXYZVYaw> trajectories);
  virtual std::vector<std::vector<double> > get_values(std::vector< std::vector<geometry_msgs::PointStamped> > trajectories);
  virtual void clear();
};

#endif
