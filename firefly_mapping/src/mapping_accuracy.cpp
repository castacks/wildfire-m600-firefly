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
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Empty.h>
#include <XmlRpcValue.h>
#include <vector>
#include <utility>
#include <map>
#include <cmath>

/*
vector of pairs: gts
map->gtfire:  ground truth locations of fires initialized from gps_hotspots.yaml
map->fire_bin_to_gt: constantly maintained ground truth map for the expected values of each bin wrt gtfires. used to compute accuracy truth values.
*/



class MappingAccuracy {

    public:
        MappingAccuracy() {

            nh.param<float>("resolution", resolution, 0.5);  
            nh.param<float>("min_x", minX, -100.0);
            nh.param<float>("max_x", maxX, 100.0);
            nh.param<float>("min_y", minY, -100.0);
            nh.param<float>("max_y", maxY, 100.0);
            mapWidth = (maxX - minX);
            mapHeight = (maxY - minY);

            init_gts();
            map_sub = nh.subscribe("observed_firemap", 10, &MappingAccuracy::map_callback, this);
            new_fire_sub = nh.subscribe("new_fire_bins", 1000, &MappingAccuracy::new_fire_bins_callback, this);
            new_no_fire_sub = nh.subscribe("new_no_fire_bins", 1000, &MappingAccuracy::new_no_fire_bins_callback, this);
            detect_acc_pub = nh.advertise<std_msgs::Float32>("detection_accuracy", 10);
            assoc_acc_pub = nh.advertise<std_msgs::Float32>("association_accuracy", 10);
            clear_sub = nh.subscribe("clear_map", 1000, &MappingAccuracy::clear, this);
        }

        void map_callback(const nav_msgs::OccupancyGrid& map) {
            current_map = map;
        }

        void clear(const std_msgs::Empty &empty_msg) {
            std::cout << "Clearing map" << std::endl;
            assoc_acc_nr = 0;
            detect_acc_nr = 0;
            fire_bin_to_gt.clear();
            for(auto& i : associated_gts) {
                i.second = 0;
            }
            detect_acc.data = detect_acc_nr/fire_bin_to_gt.size();
            assoc_acc.data = assoc_acc_nr/assoc_acc_dr;
            detect_acc_pub.publish(detect_acc);
            assoc_acc_pub.publish(assoc_acc);
        }

        void new_fire_bins_callback(const std_msgs::Int32MultiArray& msg) {
            // calculate euclidean distance. update associated_gts and fire_bin_to_gt. update accuracies
            int row, col;

            for(int bin : msg.data) {
                // size_t gridRow = (size_t) ((intersect(1)-minY)/resolution);
                // size_t gridCol = (size_t) ((intersect(0)-minX)/resolution);
                // int mapBin = gridCol + gridRow * outputMap.info.width;
                row = bin / (int)(mapHeight/resolution);
                col = bin % (int)(mapHeight/resolution);
                int min_index = -1;
                float min_dist = -1, dist;
                for(int i = 0; i < gtfire.size(); ++i) { // get closest gtfire bin and index
                    dist = sqrt(pow((gtfire[i].first - row), 2) + pow((gtfire[i].second - col), 2));
                    dist = dist * resolution; // Since bins are 0.5 meters wide
                    if(min_index == -1 or dist < min_dist) {
                        min_dist = dist;
                        min_index = i;
                    }
                }
                if(fire_bin_to_gt.find(std::make_pair(row, col)) == fire_bin_to_gt.end()) //New fire bin not in map
                {
                    if(min_dist <= detection_radius) {
                        ++detect_acc_nr;
                        if(associated_gts[gtfire[min_index]] == 0) ++assoc_acc_nr;
                        ++associated_gts[gtfire[min_index]];
                        fire_bin_to_gt[std::make_pair(row, col)] = min_index;
                    }
                    else {
                        fire_bin_to_gt[std::make_pair(row, col)] = -1;
                    }
                }
            }
            // update accuracies
            detect_acc.data = detect_acc_nr/fire_bin_to_gt.size();
            assoc_acc.data = assoc_acc_nr/assoc_acc_dr;
            detect_acc_pub.publish(detect_acc);
            assoc_acc_pub.publish(assoc_acc);
        }

        void new_no_fire_bins_callback(const std_msgs::Int32MultiArray& msg) {
            // update associated_gts and fire_bin_to_gt. update accuracies

            int row, col;
            std::pair<int, int> p;

            for(int bin : msg.data) {
                row = bin / (int)(mapHeight/resolution);
                col = bin % (int)(mapHeight/resolution);
                p = std::make_pair(row, col);
                if(fire_bin_to_gt.find(p) != fire_bin_to_gt.end()) //Converting fire bin to no fire bin
                {
                    if(fire_bin_to_gt[p] != -1) { //Fire bin is near a hotspot
                        int min_index = fire_bin_to_gt[p];
                        --detect_acc_nr;
                        if(associated_gts[gtfire[min_index]] == 1) --assoc_acc_nr;
                        --associated_gts[gtfire[min_index]];
                    }
                     fire_bin_to_gt.erase(p);
                }
            }
            // update accuracies
            detect_acc.data = detect_acc_nr/fire_bin_to_gt.size();
            assoc_acc.data = assoc_acc_nr/assoc_acc_dr;
            detect_acc_pub.publish(detect_acc);
            assoc_acc_pub.publish(assoc_acc);
        }


    private:
        ros::NodeHandle nh;
        nav_msgs::OccupancyGrid current_map;
        ros::Subscriber map_sub;
        ros::Subscriber new_fire_sub;
        ros::Subscriber new_no_fire_sub;
        ros::Subscriber clear_sub;
        ros::Publisher detect_acc_pub;
        ros::Publisher assoc_acc_pub;
        std::vector<std::pair<int, int> > gtfire;
        std::map<std::pair<int, int>, int> associated_gts;
        std::map<std::pair<int, int>, int> fire_bin_to_gt;
        float assoc_acc_nr = 0;
        float assoc_acc_dr = 0;
        float detect_acc_nr = 0;
        float minX;
        float minY;
        float maxX;
        float maxY;
        float resolution;
        float mapWidth;
        float mapHeight;
        std_msgs::Float32 detect_acc;
        std_msgs::Float32 assoc_acc;
        const float detection_radius = 5.0;

        void init_gts() {
            // init gt vector - read from YAML

            XmlRpc::XmlRpcValue gt_locs;
            ros::param::get("gt_locs", gt_locs);
            std::pair<int, int> p;

            for (size_t i = 0; i < gt_locs.size()/2; ++i) {

                double x_xml = gt_locs[2*i];
                double y_xml = gt_locs[2*i+1];

                int row = (int) ((y_xml - minY)/resolution);
                int col = (int) ((x_xml - minX)/resolution);



                p = std::make_pair(row, col);

                gtfire.push_back(p);
                associated_gts[p] = 0;
            }
            assoc_acc_dr = gtfire.size();
        }


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapping_accuracy");
    while(!ros::param::has("gt_locs")) continue;
    MappingAccuracy node;
    ros::spin();
    return 0;
}