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
map->gtmap: constantly maintained ground truth map for the expected values of each bin wrt gtfires. used to compute accuracy truth values.
*/



class MappingAccuracy {

    public:
        MappingAccuracy() {

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
            detect_acc_dr = 0;
            gtmap.clear();
            for(auto& i : associated_gts) {
                i.second = 0;
            }
        }

        void new_fire_bins_callback(const std_msgs::Int32MultiArray& msg) {
            // calculate euclidean distance. update associated_gts and gtmap. update accuracies
            int row, col;

            for(int bin : msg.data) {
                row = bin/400;
                col = bin%400;
                int min_index = -1;
                float min_dist = -1, dist;
                for(int i = 0; i < gtfire.size(); ++i) { // get closest gtfire bin and index
                    dist = sqrt(pow((gtfire[i].first - row), 2) + pow((gtfire[i].second - col), 2));
                    if(min_index == -1 or dist < min_dist) {
                        min_dist = dist;
                        min_index = i;
                    }
                }
                if(gtmap.find(std::make_pair(row, col)) == gtmap.end())
                {

                    ++detect_acc_dr;
                    if(min_dist <= detection_radius) {
                        ++detect_acc_nr;
                        if(associated_gts[gtfire[min_index]] == 0) ++assoc_acc_nr;
                        ++associated_gts[gtfire[min_index]];
                        gtmap[std::make_pair(row, col)] = min_index;
                    }
                    else {
                        gtmap[std::make_pair(row, col)] = -1;
                    }
                }
                else{
                    if(gtmap[std::make_pair(row,col)] != -1)
                    {
                        ++detect_acc_nr;
                        if(associated_gts[gtfire[min_index]] == 0) ++assoc_acc_nr;
                        ++associated_gts[gtfire[min_index]];
                    }
                    else{
                        --detect_acc_nr;
                    }
                }

            }
            // update accuracies
            detect_acc.data = detect_acc_nr/detect_acc_dr;
            assoc_acc.data = assoc_acc_nr/assoc_acc_dr;
            detect_acc_pub.publish(detect_acc);
            assoc_acc_pub.publish(assoc_acc);
        }

        void new_no_fire_bins_callback(const std_msgs::Int32MultiArray& msg) {
            // update associated_gts and gtmap. update accuracies

            int row, col;
            std::pair<int, int> p;

            for(int bin : msg.data) {
                row = bin/400;
                col = bin%400;

                int min_index = -1;
                float min_dist = -1, dist;
                p = std::make_pair(row, col);
                if(gtmap.find(p) == gtmap.end())
                {
                    ++detect_acc_dr;
                    for(int i = 0; i < gtfire.size(); ++i) { // get closest gtfire bin and index
                        dist = sqrt(pow((gtfire[i].first - row), 2) + pow((gtfire[i].second - col), 2));
                        if(min_index == -1 or dist < min_dist) {
                            min_dist = dist;
                            min_index = i;
                        }
                    }
                    if(min_dist <= detection_radius) {
                        gtmap[std::make_pair(row, col)] = min_index;
                    }
                    else {
                        ++detect_acc_nr;
                        gtmap[std::make_pair(row, col)] = -1;
                    }
                }
                else
                {

                    if(gtmap[p] != -1) {
                        min_index = gtmap[p];
                        --detect_acc_nr;
                        if(associated_gts[gtfire[min_index]] == 1) --assoc_acc_nr;
                        --associated_gts[gtfire[min_index]];
                    }
                    else{
                        ++detect_acc_nr;
                    }
                }
            }
            // update accuracies
            detect_acc.data = detect_acc_nr/detect_acc_dr;
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
        std::map<std::pair<int, int>, int> gtmap;
        float assoc_acc_nr = 0;
        float assoc_acc_dr = 0;
        float detect_acc_nr = 0;
        float detect_acc_dr = 0;
        std_msgs::Float32 detect_acc;
        std_msgs::Float32 assoc_acc;
        const float detection_radius = 2.0;

        void init_gts() {
            // init gt vector - read from YAML

            XmlRpc::XmlRpcValue gt_locs;
            ros::param::get("gt_locs", gt_locs);
            std::pair<int, int> p;

            for (size_t i = 0; i < gt_locs.size()/2; ++i) {

                double x_xml = gt_locs[2*i];
                double y_xml = gt_locs[2*i+1];

                int row = (int) ((y_xml + 100)/0.5);
                int col = (int) ((x_xml + 100)/0.5);



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