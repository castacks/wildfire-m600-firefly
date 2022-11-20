/********************************************************************************
 * Project FireFly : 2022 MRSD - Team D                                         *
 * Authors: Arjun Chauhan, Kevin Gmelin, Sabrina Shen, Manuj Trehan and Akshay Venkatesh      *
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
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <vector>
#include <algorithm>
#include <std_msgs/Empty.h>
#include <firefly_mapping/ImageWithPose.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

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
    bool show_thresh_video;
    bool show_gray_video;


public:
    ThermalImageReader()
            : it_(nh_), private_nh_("~")
    {
        image_sub_thresh = it_.subscribe("seek_camera/temperatureImageCelcius", 1,
                                   &ThermalImageReader::imageCbThresh, this);
        image_sub_gray = it_.subscribe("seek_camera/displayImage", 1,
                                   &ThermalImageReader::imageCbGray, this);

        img_extract_sub = nh_.subscribe("extract_frame",1,  &ThermalImageReader::img_extract_cb,  this);

        image_pub_ = nh_.advertise<firefly_mapping::ImageWithPose>("image_to_project", 1, this);

        private_nh_.param<int>("threshold", threshold, 50);  
        private_nh_.param<bool>("continuous", continuous, false);  
        private_nh_.param<bool>("show_thresh_video", show_thresh_video, false);  
        private_nh_.param<bool>("show_gray_video", show_gray_video, false);  

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

            if (show_thresh_video) {
                cv::imshow("Thresholded Image", cv_img.image);
            }

            listener.lookupTransform("uav1/map", "uav1/thermal/camera_link",
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
            if (show_gray_video) {
               cv::Mat resized;
               cv::resize(img, resized, cv::Size(640,480));
               cv::namedWindow("Thermal grayscale image", 1);
               cv::moveWindow("Thermal grayscale image", 1300, 0);
               cv::imshow("Thermal grayscale image", resized);
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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ThermalCameraSave");
    ThermalImageReader ic;
    ros::spin();
    return 0;
}

