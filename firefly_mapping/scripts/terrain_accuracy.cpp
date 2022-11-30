/********************************************************************************
 * Project FireFly : 2022 MRSD - Team D                                         *
 * Authors: Arjun Chauhan, Kevin Gmelin, Sabrina Shen, Manuj Trehan and Akshay Venkatesh      *
 *                                                                              *
 * MappingAccuracy : System accuracy computation module                         *
 *                                                                              *
 * Performs computaion of detection and assocaition accuracy of real-time       *
 * fire map being generated using pre-loaded ground truth hotspot GPS locations *
 *                                                                              *
 * Created:  11 Apr 2022                                                        *
********************************************************************************/
#include "ros/ros.h"
#include <unordered_map>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <XmlRpcValue.h>
#include <vector>
#include <utility>
#include <map>
#include <cmath>

#include <grid_map_msgs/GridMap.h>
#include <grid_map_ros/grid_map_ros.hpp>
#include <nav_msgs/Odometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <sensor_msgs/PointCloud2.h>

#define C_EARTH (float)6378137.0
#define C_PI (float)3.14159265
#define terrain_DEG2RAD(DEG) ((DEG) * ((C_PI) / (180.0)))
#define terrain_RAD2DEG(RAD) ((RAD) * (180.0) / (C_PI))

void gpsConvertENU(float &ENU_x, float &ENU_y,
                    float gps_t_lon, float gps_t_lat,
                    float gps_r_lon, float gps_r_lat)
{
    float d_lon = gps_t_lon - gps_r_lon;
    float d_lat = gps_t_lat - gps_r_lat;
    ENU_y = terrain_DEG2RAD(d_lat) * C_EARTH;
    ENU_x = terrain_DEG2RAD(d_lon) * C_EARTH * cos(terrain_DEG2RAD(gps_t_lat));
};

class TerrainAccuracy {

    public:
        TerrainAccuracy(std::string filename = "/home/wildfire/m600_ws/src/firefly/firefly_mapping/scripts/MergedCloudClean.pcd") {
            map_sub = nh.subscribe("uav1/grid_map", 10, &TerrainAccuracy::map_callback, this);
            elev_acc_pub = nh.advertise<std_msgs::Float32>("average_elevation_accuracy", 10);
            max_err_pub = nh.advertise<std_msgs::Float32>("max_elevation_error", 10);
            gt_map_pub = nh.advertise<grid_map_msgs::GridMap>("gt_map", 1);
            map_pub_timer = nh.createTimer(ros::Duration(1.0),
                                   &TerrainAccuracy::publish_map_callback, this);

            
            nh.param<float>("terrain_resolution", resolution, 2.5);
            nh.param<float>("min_x", minX, -250.0);
            nh.param<float>("max_x", maxX, 250.0);
            nh.param<float>("min_y", minY, -250.0);
            nh.param<float>("max_y", maxY, 250.0);

            float lengthX = maxX - minX;
            float lengthY = maxY - minY;

            terrain_gt_grid.setFrameId("uav1/map");
            terrain_gt_grid.setGeometry(grid_map::Length(lengthX, lengthY), resolution);
            mapWidth = terrain_gt_grid.getSize()(0);
            mapHeight = terrain_gt_grid.getSize()(1);
            terrain_gt_grid.add("elevation", grid_map::Matrix::Constant( mapWidth, mapHeight, 0));
            ROS_INFO("Created map with size %f x %f m (%i x %i cells).",
                    terrain_gt_grid.getLength().x(), terrain_gt_grid.getLength().y(), terrain_gt_grid.getSize()(0),
                    terrain_gt_grid.getSize()(1));

            init_gt(filename);
        }

    private:
        ros::NodeHandle nh;
        ros::Subscriber map_sub;
        ros::Subscriber clear_sub;
        ros::Publisher elev_acc_pub;
        ros::Publisher max_err_pub;
        ros::Timer map_pub_timer;
        ros::Publisher gt_map_pub;

        pcl::PointCloud<pcl::PointXYZ>::Ptr terrain_gt_cloud_raw; 
        pcl::PointCloud<pcl::PointXYZ>::Ptr terrain_gt_cloud; 
        std::unordered_map<int, float> terrain_gt_map;
        grid_map::GridMap terrain_gt_grid;

        std_msgs::Float32 avg_acc;
        std_msgs::Float32 max_err;
        tf::TransformListener listener;

        float resolution;
        float minX;
        float maxX;
        float minY;
        float maxY;
        int mapWidth; 
        int mapHeight;  

        void init_gt(std::string filename) {       
            pcl::PointCloud<pcl::PointXYZ>::Ptr terrain_gt_cloud_raw (new pcl::PointCloud<pcl::PointXYZ>);
            if(pcl::io::loadPCDFile<pcl::PointXYZ> (filename, *terrain_gt_cloud_raw) == -1) // load point cloud file
            {   
                PCL_ERROR("Could not read the file");
                return;
            }
            std::cout<<"Loaded "<<terrain_gt_cloud_raw->width * terrain_gt_cloud_raw->height
                    <<"data points from MallGroundtruth.pcd" <<std::endl;
            
            try {
                // define the transform of the ground truth data
                Eigen::Affine3f transform = Eigen::Affine3f::Identity();

                float theta = M_PI/15; // The angle of rotation in radians
                transform.rotate (Eigen::AngleAxisf (theta, Eigen::Vector3f::UnitZ()));


                while (!ros::param::has("/uav1/local_pos_ref_longitude") || 
                        !ros::param::has("/uav1/local_pos_ref_latitude") || 
                        !ros::param::has("/uav1/local_pos_ref_altitude"))
                {
                    ROS_WARN("Parameters have not been set, please set parameters before continuing");
                    ros::Duration(2).sleep();
                }
                
                //get local position from params
                float t_x;
                float t_y;
                float t_z;
                float ENU_x;
                float ENU_y;
                ros::param::get("/uav1/local_pos_ref_longitude", t_x); //-79.944
                ros::param::get("/uav1/local_pos_ref_latitude", t_y); //40.441
                ros::param::get("/uav1/local_pos_ref_altitude", t_z); //258.979
                float r_lon = -79.9442565;
                float r_lat = 40.44184940;
                
                gpsConvertENU(ENU_x, ENU_y, t_x, t_y, r_lon, r_lat);

                // transform.translation() << -70.0, 30.0, -263.546875;
                ROS_INFO("THE PARAMS: %f, %f, %f",-ENU_x, -ENU_y, -t_z);
                transform.translation() << -ENU_x, -ENU_y, -t_z;

                // executing the transformation
                pcl::PointCloud<pcl::PointXYZ>::Ptr terrain_gt_cloud (new pcl::PointCloud<pcl::PointXYZ>);
                pcl::transformPointCloud (*terrain_gt_cloud_raw, *terrain_gt_cloud, transform);

                // Parse through available ground truth maps
                unsigned int size = (unsigned int)terrain_gt_cloud->points.size();
                for (unsigned int k = 0; k < size; ++k) {
                    const pcl::PointXYZ& pt_cloud = terrain_gt_cloud->points[k];

                    // check for invalid measurements
                    if (isnan(pt_cloud.x) || isnan(pt_cloud.y) || isnan(pt_cloud.z)){
                        ROS_INFO("Nan detected, skipping point");
                        continue;
                    }

                    // find grid row and column
                    const int grid_map_col =  mapHeight - (const int)((pt_cloud.y - minY) / resolution);
                    const int grid_map_row = mapWidth - (const int)((pt_cloud.x - minX) / resolution);

                    // check validity of grid bin
                    const bool valid_row = (0 <= grid_map_row) && (grid_map_row < mapWidth);
                    const bool valid_col = (0 <= grid_map_col) && (grid_map_col < mapHeight);
                    if (!valid_row || !valid_col) {
                         ROS_INFO("Point outside of bounds for the map area");
                        continue;
                    }

                    // fill bin with maximum height
                    int mapBin = grid_map_col + grid_map_row * mapHeight;
                    const auto search = terrain_gt_map.find(mapBin);
                    if (search == terrain_gt_map.end() || pt_cloud.z  > search->second) {
                        terrain_gt_map[mapBin] = pt_cloud.z;
                    }
                }
            } catch (tf::TransformException& ex) {
            ROS_ERROR_STREAM(
                "Transform Exception in distance_to_obstacle while looking up tf: "
                << ex.what());
            }
            
            // convert unordered map to grid map
            for (const auto iter : terrain_gt_map) {
                const auto map_bin = iter.first;
                const auto bin_height = iter.second;

                const int grid_row = floor(map_bin / mapHeight);
                const int grid_col = map_bin % mapHeight;

                grid_map::Index index{grid_row, grid_col};
                terrain_gt_grid.at("elevation", index) = bin_height;
            }

        }

        void publish_map_callback(const ros::TimerEvent& e) {
            // publish map at frequency
            ros::Time time = ros::Time::now();
            terrain_gt_grid.setTimestamp(time.toNSec());
            grid_map_msgs::GridMap message;
            grid_map::GridMapRosConverter::toMessage(terrain_gt_grid, message);
            gt_map_pub.publish(message);
            ROS_INFO_THROTTLE(1.0, "Groundtruth at timestamp %f published", message.info.header.stamp.toSec());
        }

        void map_callback(const grid_map_msgs::GridMap& terrain_map_msg) {
            
            float avg_acc_f = float(0);
            float max_err_f = float(0);

            grid_map::GridMap terrain_map;
            grid_map::GridMapRosConverter::fromMessage(terrain_map_msg, terrain_map);
            Eigen::Array2d lengths = terrain_map.getLength();


            int terrainHeight = floor(terrain_map_msg.info.length_y / terrain_map_msg.info.resolution);
            int terrainWidth = floor(terrain_map_msg.info.length_x / terrain_map_msg.info.resolution);

            float error_count = 0.0;
            
            for (grid_map::GridMapIterator iterator(terrain_map); !iterator.isPastEnd(); ++iterator) {
                const grid_map::Index index(*iterator);

                int r = index(0);
                int c = index(1);
                int map_bin = r*mapWidth+c;

                float bin_height = terrain_map.at("elevation", index);

                const auto search = terrain_gt_map.find(map_bin);
                if (search != terrain_gt_map.end()&&bin_height!=0) {
                    const int grid_row = floor(map_bin / mapHeight);
                    const int grid_col = map_bin % mapHeight;

                    float gt = terrain_gt_map[map_bin];
                    float diff = abs(gt-bin_height);
                    avg_acc_f = avg_acc_f+diff;
                    error_count = error_count+1;
                    if (diff>max_err_f){
                        max_err_f = diff;
                    }
                    // ROS_INFO("Processing error for: %f, %f and got a height diff of %f given max %f", gt , bin_height, diff, max_err_f);
                }
                else{
                    const int grid_row = floor(map_bin / terrainHeight);
                    const int grid_col = map_bin % terrainHeight;
                    // ROS_INFO("Ground truth bin was not found for index: %i, %i, with height: %f", grid_row , grid_col, bin_height);
                }
                if(bin_height>0){
                    // ROS_INFO("Got bin height: %f", bin_height);
                }
            }

            avg_acc.data = avg_acc_f/error_count;
            max_err.data = max_err_f;
            ROS_INFO("Terrain map received. Average Accuracy: %f , Maximum Error: %f", avg_acc.data, max_err.data);
            elev_acc_pub.publish(avg_acc);
            max_err_pub.publish(max_err);
        }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "terrain_accuracy");
    TerrainAccuracy node;
    ros::spin();
    return 0;
}