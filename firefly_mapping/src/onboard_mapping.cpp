#include "ros/ros.h"
#include "firefly_mapping/ImageWithPose.h"
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/UInt8MultiArray.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Empty.h>
#include <chrono>
#include <unordered_set>


class OnboardMapping {

public:
    OnboardMapping() {
        image_sub = nh.subscribe("image_to_project", 1000, &OnboardMapping::project_image, this);
        map_pub = nh.advertise<nav_msgs::OccupancyGrid>("observed_firemap", 10);
        new_fire_pub = nh.advertise<std_msgs::Int32MultiArray>("new_fire_bins", 10);
        new_no_fire_pub = nh.advertise<std_msgs::Int32MultiArray>("new_no_fire_bins", 10);
//        clear_srv = nh.advertiseService("clear_map", &OnboardMapping::clear, this);
        clear_sub = nh.subscribe("clear_map", 1000, &OnboardMapping::clear, this);

        K_inv << 1.0/fx,  0.0,    -cx/fx,
                 0.0,     1.0/fy, -cy/fy,
                 0.0,     0.0,     1.0;

        outputMap.header.frame_id = "world";
        outputMap.info.resolution = 0.5;
        outputMap.info.width = 400; //Number of Cells
        outputMap.info.height = 400; //Number of Cells
        outputMap.info.origin.position.x = -100; //In meters
        outputMap.info.origin.position.y = -100; //In meters
        outputMap.data = std::vector<std::int8_t> (400*400, 50); // Initialize map to 50 percent certainty
        map = std::vector<float> (400*400, 0.5); // Initialize map to 50 percent certainty
        map_pub.publish(outputMap);
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    ros::Publisher map_pub, new_fire_pub, new_no_fire_pub;
//    ros::ServiceServer clear_srv;
    ros::Subscriber clear_sub;

    nav_msgs::OccupancyGrid outputMap;
    std::vector<float> map; // Internal map representation

    Eigen::Vector3d ground_normal{0, 0, 1}; //Should point up from ground - if pointing into ground, will cause errors
    float ground_offset = 0;

//    float fx = 411.3366;
//    float fy = 412.6712;
//    float cx = 320.4049;
//    float cy = 251.967;
    float fx = 317.1595;
    float fy = 324.8607;
    float cx = 100;
    float cy = 75;

    float resolution = 0.5;
    float minX = -100;
    float maxX = 100;
    float minY = -100;
    float maxY = 100;

    float fpr = 0.05; // False positives divided by all negative cases
    float fnr = 0.05; // False negatives divided by all positive cases
    float tpr = 1 - fnr; // True positives divided by all positives
    float tnr = 1 - fpr; // True negatives divided by all negatives

    Eigen::Matrix3d K_inv;

    void project_image(const firefly_mapping::ImageWithPose& msg) {
        auto start = std::chrono::high_resolution_clock::now();
        Eigen::Quaterniond cam_quat;
        tf::quaternionMsgToEigen(msg.pose.orientation, cam_quat);

        Eigen::Matrix3d pixelToRay = cam_quat.matrix() * K_inv;
        Eigen::Vector3d camCenter {msg.pose.position.x, msg.pose.position.y, msg.pose.position.z};

        float camCenterGroundDist = ground_offset - ground_normal.dot(camCenter);

        std::cout << "Projecting and filtering!" << std::endl;

        std::unordered_set<int> new_fire_bins;
        std::unordered_set<int> new_no_fire_bins;

        for (size_t i = 0; i < msg.image.width; i++) {
            for (size_t j = 0; j < msg.image.height; j++) {
                Eigen::Vector3d pixelHomogenous{i, j, 1};
                Eigen::Vector3d rayDir = pixelToRay * pixelHomogenous;

                float rayPerpendicularComponent = ground_normal.dot(rayDir);
                if (rayPerpendicularComponent >= 0) {
                    continue;
                }

                float rayLambda = camCenterGroundDist / rayPerpendicularComponent;

                Eigen::Vector3d intersect = camCenter + rayLambda * rayDir;

                if (intersect(0) > maxX
                    || intersect(0) < minX
                    || intersect(1) > maxY
                    || intersect(1) < minY) {
                    continue;
                }

                size_t gridRow = (size_t) ((intersect(1)-minY)/resolution);
                size_t gridCol = (size_t) ((intersect(0)-minX)/resolution);

                int mapBin = gridCol + gridRow * outputMap.info.width;
                uint8_t pixelValue = msg.image.data[i + j * msg.image.width];
                float prior = map[mapBin];

                if (pixelValue == 0) {
                    float probOfNegative = fnr * prior + tnr * (1 - prior);
                    float posterior = (fnr/probOfNegative) * prior;
                    map[mapBin] = posterior;
                    outputMap.data[mapBin] = posterior*100;

                    if ((prior >= 0.5) && (posterior < 0.5)) { // If bin changed value
                        new_fire_bins.erase(mapBin);
                        new_no_fire_bins.insert(mapBin);
                    }
                }
                else {
                    float probOfPositive = tpr * prior + fpr * (1 - prior);
                    float posterior = (tpr/probOfPositive) * prior;
                    map[mapBin] = posterior;
                    outputMap.data[mapBin] = posterior*100;

                    if ((prior <= 0.5) && (posterior > 0.5)) { // If bin changed value
                        new_no_fire_bins.erase(mapBin);
                        new_fire_bins.insert(mapBin);
                    }
                }

            }
        }

        std_msgs::Int32MultiArray new_fire_bins_msg;
        for (int bin: new_fire_bins) {
            new_fire_bins_msg.data.push_back(bin);
        }
        new_fire_pub.publish(new_fire_bins_msg);

        std_msgs::Int32MultiArray new_no_fire_bins_msg;
        for (int bin: new_no_fire_bins) {
            new_no_fire_bins_msg.data.push_back(bin);
        }
        new_no_fire_pub.publish(new_no_fire_bins_msg);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Projection of single image took: " << duration.count() << " milliseconds" << std::endl;

        map_pub.publish(outputMap);
        return;
    }

//    bool clear(std_srvs::Empty::Request &req, std_srvs::Empty::Response &res) {
//        std::cout << "Clearing Map" << std::endl;
//        outputMap.data = std::vector<std::int8_t> (400*400, 50); // Set map to 50 percent certainty
//        map = std::vector<float> (400*400, 0.5); // Set map to 50 percent certainty
//        map_pub.publish(outputMap);
//        return true;
//    }
    void clear(const std_msgs::Empty &empty_msg) {
        std::cout << "Clearing Map" << std::endl;
        outputMap.data = std::vector<std::int8_t> (400*400, 50); // Set map to 50 percent certainty
        map = std::vector<float> (400*400, 0.5); // Set map to 50 percent certainty
        map_pub.publish(outputMap);
    }
};
int main(int argc, char** argv) {
    ros::init(argc, argv, "firefly_mapping");
    OnboardMapping node;
    ros::spin();
    return 0;
}