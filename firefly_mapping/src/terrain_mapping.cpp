#include <grid_map_msgs/GridMap.h>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <tf/transform_listener.h>

#include <visualization_msgs/MarkerArray.h>

#include <cmath>
#include <grid_map_ros/grid_map_ros.hpp>
#include <unordered_map>
#include <vector>

#include "sensor_msgs/PointCloud2.h"
#include "std_msgs/Bool.h"

using namespace grid_map;

class TerrainMapping {
 public:
  TerrainMapping() : pnh("~"), map({"elevation"}) {
    cloud_sub = nh.subscribe("lidar_cropped_mapping", 1,
                             &TerrainMapping::cloudCallback, this);
    enable_terrain_mapping_sub = nh.subscribe("enable_terrain_mapping", 1,
                                  &TerrainMapping::enableCallback, this);

    grid_map_pub = nh.advertise<grid_map_msgs::GridMap>("grid_map", 1);

    pnh.param<float>("terrain_resolution", resolution, 0.5);
    pnh.param<float>("min_x", minX, -100.0);
    pnh.param<float>("max_x", maxX, 100.0);
    pnh.param<float>("min_y", minY, -100.0);
    pnh.param<float>("max_y", maxY, 100.0);

    terrain_mapping_enabled = false;

    float lengthX = maxX - minX;
    float lengthY = maxY - minY;

    map.setFrameId("uav1/map");
    map.setGeometry(grid_map::Length(lengthX, lengthY), resolution);
    mapWidth = map.getSize()(0);
    mapHeight = map.getSize()(1);
    map.add("elevation", grid_map::Matrix::Constant(
                             mapWidth, mapHeight,
                             0));  // Initialize map to 50 percent certainty
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
  ros::Subscriber enable_terrain_mapping_sub;
  ros::Publisher grid_map_pub;
  ros::Timer map_pub_timer;

  GridMap map;

  tf::TransformListener listener;

  float resolution;
  float minX;
  float maxX;
  float minY;
  float maxY;
  int mapWidth;   // Number of cells
  int mapHeight;  // Number of cells
  bool terrain_mapping_enabled;

  void cloudCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud) {
    if (!terrain_mapping_enabled) {
      return;
    }
    auto start = std::chrono::high_resolution_clock::now();
    std::unordered_map<int, float> map_bin_to_max_altitude;

    try {
      sensor_msgs::PointCloud2 cloud_target_frame;
      // listener.waitForTransform("uav1/map", "uav1/lidar", cloud.header.stamp,
      //                           ros::Duration(0.1));

      pcl_ros::transformPointCloud("uav1/map", *cloud, cloud_target_frame,
                                   listener);

      pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud2_map_pcl(
          new pcl::PointCloud<pcl::PointXYZ>());
      pcl::PointCloud<pcl::PointXYZ>::Ptr pointcloud2_sensor_pcl(
          new pcl::PointCloud<pcl::PointXYZ>());

      pcl::fromROSMsg(*cloud, *pointcloud2_sensor_pcl);
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
        static constexpr float MIN_OBSERVABLE_DISTANCE = 1.0;
        if (MAX_OBSERVABLE_DISTANCE < measurement_distance || measurement_distance < MIN_OBSERVABLE_DISTANCE) continue;

        const int grid_map_col =  mapHeight - (const int)((pt_cloud.y - minY) / resolution);
        const int grid_map_row = mapWidth - (const int)((pt_cloud.x - minX) / resolution);

        const bool valid_row = (0 <= grid_map_row) && (grid_map_row < mapWidth);
        const bool valid_col = (0 <= grid_map_col) && (grid_map_col < mapHeight);

        if (!valid_row || !valid_col) {
          continue;
        }

        int mapBin = grid_map_col + grid_map_row * mapHeight;

        const auto search = map_bin_to_max_altitude.find(mapBin);
        if (search == map_bin_to_max_altitude.end() ||
            pt_cloud.z > search->second) {
          map_bin_to_max_altitude[mapBin] = pt_cloud.z;
        }
      }
    } catch (tf::TransformException& ex) {
      ROS_ERROR_STREAM(
          "Transform Exception in distance_to_obstacle while looking up tf: "
          << ex.what());
    }

    for (const auto iter : map_bin_to_max_altitude) {
      const auto map_bin = iter.first;
      const auto bin_height = iter.second;

      const int grid_row = floor(map_bin / mapHeight);
      const int grid_col = map_bin % mapHeight;

      grid_map::Index index{grid_row, grid_col};
      map.at("elevation", index) = bin_height;
    }
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
    std::cout << "Mapping of point cloud took: " << duration.count() << " milliseconds" << std::endl;
  }

  void enableCallback(const std_msgs::Bool& msg) {
    terrain_mapping_enabled = msg.data;
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
