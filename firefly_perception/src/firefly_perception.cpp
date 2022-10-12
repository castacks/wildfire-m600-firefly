/********************************************************************************
 * Project FireFly : 2022 MRSD - Team D                                         *
 * Authors: Arjun Chauhan, Kevin Gmelin, Sabrina Shen and Akshay Venkatesh      *
 *                                                                              *
 * ThermalImageReader : Thermal camera data processing module                   *
 *                                                                              *
 * Reads thermal camera image topic, applies thresholding for fire detection,   *
 * publishes segmented image with current pose from which image was taken       *
 *                                                                              *
 * Created:  03 Apr 2022                                                        *
********************************************************************************/

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/PointCloud2.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <std_msgs/Empty.h>
#include <firefly_mapping/ImageWithPose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/transforms.h>

class ThermalImageReader
{
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_thresh;
    image_transport::Subscriber image_sub_gray;
    ros::Publisher image_pub_;
    ros::Subscriber img_extract_sub;
    cv::Mat thresh;
    cv_bridge::CvImage cv_img;
    tf::TransformListener listener;
    tf::StampedTransform img_transform;
    bool img_with_tf_ready = false;

    int threshold;
    bool continuous;
    bool continuousMappingEnabled = false;
    bool show_video;


public:
    ThermalImageReader()
            : it_(nh_), private_nh_("~")
    {
        image_sub_thresh = it_.subscribe("/seek_camera/temperatureImageCelcius", 1,
                                   &ThermalImageReader::imageCbThresh, this);
        image_sub_gray = it_.subscribe("/seek_camera/displayImage", 1,
                                   &ThermalImageReader::imageCbGray, this);

        img_extract_sub = nh_.subscribe("extract_frame",1,  &ThermalImageReader::img_extract_cb,  this);

        image_pub_ = nh_.advertise<firefly_mapping::ImageWithPose>("image_to_project", 1, this);

        private_nh_.param<int>("threshold", threshold, 50);  
        private_nh_.param<bool>("continuous", continuous, false);  
        private_nh_.param<bool>("show_video", show_video, false);  

    }

    ~ThermalImageReader()
    {
        cv::destroyWindow("Thresholded Image");
        cv::destroyWindow("Gray Image");
    }

    void imageCbThresh(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv::Mat img = cv_bridge::toCvShare(msg,"32FC1")->image;

            cv::Mat thresh;
            cv::threshold(img, thresh, threshold, 255, CV_THRESH_BINARY);
            thresh.convertTo(cv_img.image, CV_8U);

            if (show_video) {
                cv::imshow("Thresholded Image", cv_img.image);
            }


            listener.lookupTransform("world", "thermal/camera_link",
                                         ros::Time(0), img_transform);

            img_with_tf_ready = true;

            cv::waitKey(3);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
        catch (tf::TransformException &ex) {
                img_with_tf_ready = false;
                return;
        }

        if (continuous && continuousMappingEnabled) {
            firefly_mapping::ImageWithPose reply;
            cv_img.toImageMsg(reply.image);

            reply.pose.position.x = img_transform.getOrigin().x();
            reply.pose.position.y = img_transform.getOrigin().y();
            reply.pose.position.z = img_transform.getOrigin().z();

            reply.pose.orientation.x = img_transform.getRotation().x();
            reply.pose.orientation.y = img_transform.getRotation().y();
            reply.pose.orientation.z = img_transform.getRotation().z();
            reply.pose.orientation.w = img_transform.getRotation().w();

            image_pub_.publish(reply);
        }
    }

    void imageCbGray(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv::Mat img = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8)->image;
            if (show_video) {
                cv::imshow("Gray Image", img);
            }
            cv::waitKey(3);
        }

        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void img_extract_cb(std_msgs::Empty msg)
    {
        if (continuous) {
            continuousMappingEnabled = !continuousMappingEnabled;

        }
        else {
            if (!img_with_tf_ready) {
                ROS_ERROR("Called extract_frame but either img or tf not available");
                return;
            }
            firefly_mapping::ImageWithPose reply;
            cv_img.toImageMsg(reply.image);

            reply.pose.position.x = img_transform.getOrigin().x();
            reply.pose.position.y = img_transform.getOrigin().y();
            reply.pose.position.z = img_transform.getOrigin().z();

            reply.pose.orientation.x = img_transform.getRotation().x();
            reply.pose.orientation.y = img_transform.getRotation().y();
            reply.pose.orientation.z = img_transform.getRotation().z();
            reply.pose.orientation.w = img_transform.getRotation().w();

            image_pub_.publish(reply);

        }
        
    }
};


class LidarReader 
{
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    
    sensor_msgs::PointCloud2 input_pointcloud_;

    ros::Publisher lidar_mapping_pub_;
    ros::Publisher lidar_obstacle_pub_;

    ros::Subscriber lidar_subscriber;

public:
    LidarReader() : private_nh_("~")
    {
        lidar_subscriber = nh_.subscribe("/velodyne_points", 1,
                                   &LidarReader::point_cloud_extractor, this);

    }

    ~LidarReader()
    {
        std::cout<<"GET GOT. BYE LIDAR \n";
    }

    void point_cloud_extractor(const sensor_msgs::PointCloud2::ConstPtr& msg)
    {
        // if (msg->is_dense)
        //     std::cout  << "\t Height : " << msg->height << "\t Width : " << msg->width << std::endl;
        // std::cout << msg->data->size() << " \t  " << msg->row_step << std::endl;

        pcl::PCLPointCloud2 pcl_pc2;

        pcl_conversions::toPCL(*msg, pcl_pc2);
        pcl::PointCloud<pcl::PointXYZ> temp_cloud;
        pcl::fromPCLPointCloud2(pcl_pc2, temp_cloud);

//         pcl::PCLPointCloud2 pcl_pc2;
// pcl_conversions::toPCL(*cloud_msg,pcl_pc2);
// pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud(new pcl::PointCloud<pcl::PointXYZ>);
// pcl::fromPCLPointCloud2(pcl_pc2,*pt_cloud);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ThermalCameraSave");
    ros::init(argc, argv, "LidarNode");

    // ThermalImageReader ic;
    LidarReader ic; 

    ros::spin();
    return 0;
}

