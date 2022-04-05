#include "ros/ros.h"
#include "firefly_mapping/ImageWithPose.h"
#include <eigen_conversions/eigen_msg.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Int32MultiArray.h>
#include <std_msgs/Empty.h>
#include <chrono>
#include <unordered_set>


class OnboardMappingTest {

public:
    OnboardMappingTest() {
        image_sub = nh.subscribe("image_to_project", 1000, &OnboardMappingTest::project_image, this);
        new_fire_pub = nh.advertise<std_msgs::Int32MultiArray>("new_fire_bins", 10);

        K_inv << 1.0/fx,  0.0,    -cx/fx,
                 0.0,     1.0/fy, -cy/fy,
                 0.0,     0.0,     1.0;
    }

private:
    ros::NodeHandle nh;
    ros::Subscriber image_sub;
    ros::Publisher new_fire_pub;

    Eigen::Vector3d ground_normal{0, 0, 1}; //Should point up from ground - if pointing into ground, will cause errors
    float ground_offset = 0;

    float fx = 338.136183;
    float fy = 336.039570;
    float cx = 160.829;
    float cy = 112.614;

    float resolution = 0.5;
    float minX = -100;
    float maxX = 100;
    float minY = -100;
    float maxY = 100;

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
                uint8_t pixelValue = msg.image.data[i + j * msg.image.width];
                if (pixelValue == 0) {
                    continue;
                }

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

                int mapBin = gridCol + gridRow * 400;

                new_fire_bins.insert(mapBin);

            }
        }

        std_msgs::Int32MultiArray new_fire_bins_msg;
        for (int bin: new_fire_bins) {
            new_fire_bins_msg.data.push_back(bin);
        }
        new_fire_pub.publish(new_fire_bins_msg);

        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        std::cout << "Projection of single image took: " << duration.count() << " milliseconds" << std::endl;

        return;
    }

};
int main(int argc, char** argv) {
    ros::init(argc, argv, "firefly_mapping_test");
    OnboardMappingTest node;
    ros::spin();
    return 0;
}