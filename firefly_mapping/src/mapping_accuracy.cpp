#include "ros/ros.h"
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int32MultiArray.h>
#include <vector>
#include <utility>
#include <map>
#include <cmath>

/*
vector of pairs: gts
map: indices of gt bins mapped to number of associations
map: new detection index mapped to gt vector index
*/


const std::vector<float> gt_gps;


class MappingAccuracy {

    public:
        MappingAccuracy() {

            init_gts();
            map_sub = nh.subscribe("observed_firemap", 10, &MappingAccuracy::map_callback, this);
            new_fire_sub = nh.subscribe("new_fire_bins", 1000, &MappingAccuracy::new_fire_bins_callback, this);
            new_no_fire_sub = nh.subscribe("new_no_fire_bins", 1000, &MappingAccuracy::new_no_fire_bins_callback, this);


        }

        void map_callback(const nav_msgs::OccupancyGrid& map) {
            current_map = map;
        }

        void new_fire_bins_callback(const std_msgs::Int32MultiArray& msg) {
            // calculate euclidean distance. update associated_gts and new_detections. update accuracies
            int row, col;

            for(int bin : msg.data) {
                ++detect_acc_dr;
                row = bin/400;
                col = bin%400;
                int min_index = -1;
                float min_dist = -1, dist;
                for(int i = 0; i < gt.size(); ++i) { // get closest gt bin and index
                    dist = sqrt(pow((gt[i].first - row), 2) + pow((gt[i].second - col), 2));
                    if(min_dist != -1 and dist < min_dist) {
                        min_dist = dist;
                        min_index = i;
                    }
                }
                if(min_dist <= 5) {
                    ++detect_acc_nr;
                    if(associated_gts[gt[min_index]] == 0) ++assoc_acc_nr;
                    ++associated_gts[gt[min_index]];
                    new_detections[std::make_pair(row, col)] = min_index;
                }
                else {
                    new_detections[std::make_pair(row, col)] = -1;
                }

            }
            // update accuracies
            detect_acc = detect_acc_nr/detect_acc_dr;
            assoc_acc = assoc_acc_nr/assoc_acc_dr;
        }

        void new_no_fire_bins_callback(const std_msgs::Int32MultiArray& msg) {
            // update associated_gts and new_detections. update accuracies

            int row, col;

            for(int bin : msg.data) {
                --detect_acc_dr;
                row = bin/400;
                col = bin%400;
                // int min_index = -1;
                // float min_dist = -1, dist;
                // for(int i = 0; i < gt.size(); ++i) { // get closest gt bin and index
                //     dist = sqrt(pow((gt[i].first - row), 2) + pow((gt[i].second - col), 2));
                //     if(min_dist != -1 and dist < min_dist) {
                //         min_dist = dist;
                //         min_index = i;
                //     }
                // }
                int min_index = -1;
                auto p = std::make_pair(row, col);
                if(new_detections[p] != -1) {
                    min_index = new_detections[p];
                    new_detections[p] = -1;
                    --detect_acc_nr;
                    if(associated_gts[gt[min_index]] == 1) --assoc_acc_nr;
                    --associated_gts[gt[min_index]];
                }

                // if(min_dist <= 5) {
                //     ++detect_acc_nr;
                //     if(associated_gts[gt[min_index]] == 0) ++assoc_acc_nr;
                //     ++associated_gts[gt[min_index]];
                //     new_detections[std::make_pair(row, col)] = min_index;
                // }
                // else {
                //     new_detections[std::make_pair(row, col)] = -1;
                // }

            }
            // update accuracies
            detect_acc = detect_acc_nr/detect_acc_dr;
            assoc_acc = assoc_acc_nr/assoc_acc_dr;
        }


    private:
        ros::NodeHandle nh;
        nav_msgs::OccupancyGrid current_map;
        ros::Subscriber map_sub;
        ros::Subscriber new_fire_sub;
        ros::Subscriber new_no_fire_sub;
        std::vector<std::pair<int, int> > gt;
        std::map<std::pair<int, int>, int> associated_gts;
        std::map<std::pair<int, int>, int> new_detections;
        float assoc_acc = 0;
        float detect_acc = 0;
        float assoc_acc_nr = 0;
        float assoc_acc_dr = 0;
        float detect_acc_nr = 0;
        float detect_acc_dr = 0;

        void init_gts() {
            // init gt vector - read from YAML
            for (size_t i = 0; i < gt_gps.size()/2; ++i) {
                int row = (int) ((gt_gps[2*i]-current_map.info.origin.position.y)/current_map.info.resolution);
                int col = (int) ((gt_gps[2*i+1]-current_map.info.origin.position.x)/current_map.info.resolution);

                std::pair<int, int> p(row, col);

                gt.push_back(p);
                associated_gts[p] = 0;
            }
            assoc_acc_dr = gt.size();
        }


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "mapping_accuracy");
    // TODO: read params into gt_gps
    MappingAccuracy node;
    ros::spin();
    return 0;
}