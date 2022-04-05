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
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_thresh;
    image_transport::Subscriber image_sub_gray;
    ros::Publisher image_pub_;
    ros::Subscriber img_extract_sub;
    cv::Mat thresh;
    cv_bridge::CvImage cv_img;
    tf::TransformListener listener;




public:
    int i{0};
    int j{0};
    ThermalImageReader()
            : it_(nh_)
    {
        image_sub_thresh = it_.subscribe("/seek_camera/temperatureImageCelcius", 1,
                                   &ThermalImageReader::imageCbThresh, this);
        image_sub_gray = it_.subscribe("/seek_camera/displayImage", 1,
                                   &ThermalImageReader::imageCbGray, this);

        img_extract_sub = nh_.subscribe("extract_frame",1,  &ThermalImageReader::img_extract_cb,  this);

        image_pub_ = nh_.advertise<firefly_mapping::ImageWithPose>("image_to_project", 1, this);
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
            // cv_img = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            cv::Mat img = cv_bridge::toCvShare(msg,"32FC1")->image;
            // std::cout<<"Image details : "<<img.size().width<<" "<<img.size().height<<"\n";

            cv::Mat thresh;
            cv::threshold(img, thresh, 25, 255, CV_THRESH_BINARY);
            thresh.convertTo(cv_img.image, CV_8U);
//            cv_img->image = thresh;

            // std::vector<float> unik = ThermalImageReader::unique(img, true);
            // for (unsigned int i = 0; i < unik.size(); i++)
            //     std::cout << unik[i] << " ";
            
            ++i;
            std::string s_bmp = "images/"+std::to_string(i)+".bmp";
            std::string s_seg = "images/"+std::to_string(i)+"_segmented.png";
            // std::cout<<std::to_string(i)+".bmp"<<" \n";
            
            // cv::imwrite(s_bmp,img);
            // cv::imwrite(s_seg,thresh);

            cv::imshow("Thresholded Image", cv_img.image);

            cv::waitKey(3);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        // Output modified video stream
    }

    void imageCbGray(const sensor_msgs::ImageConstPtr& msg)
    {
        try
        {
            cv::Mat img = cv_bridge::toCvShare(msg,sensor_msgs::image_encodings::BGR8)->image;
            ++j;
            std::string s_bmp = "images/"+std::to_string(j)+"_gray.png";
            
            // cv::imwrite(s_bmp,img);

            cv::imshow("Gray Image", img);

            cv::waitKey(3);
        }

        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    std::vector<float> unique(const cv::Mat& input, bool sort = false)
    {
        if (input.channels() > 1 || input.type() != CV_32F) 
        {
            std::cerr << "unique !!! Only works with CV_32F 1-channel Mat" << std::endl;
            return std::vector<float>();
        }

        std::vector<float> out;
        for (int y = 0; y < input.rows; ++y)
        {
            const float* row_ptr = input.ptr<float>(y);
            for (int x = 0; x < input.cols; ++x)
            {
                float value = row_ptr[x];

                if ( std::find(out.begin(), out.end(), value) == out.end() )
                    out.push_back(value);
            }
        }

        if (sort)
            std::sort(out.begin(), out.end());

        return out;
    }

    void img_extract_cb(std_msgs::Empty msg)
    {
        firefly_mapping::ImageWithPose reply;
        cv_img.toImageMsg(reply.image);

        

        tf::StampedTransform transform;
        try {
          listener.lookupTransform("world", "thermal/camera_link",
                                   ros::Time(0), transform);
        }
        catch (tf::TransformException &ex) {
          ROS_ERROR("%s",ex.what());
          ros::Duration(1.0).sleep();
          //continue;
        }
        //listener.lookupTransform("base_link", "world",  ros::Time::now(), transform);

        reply.pose.position.x = transform.getOrigin().x();
        reply.pose.position.y = transform.getOrigin().y();
        reply.pose.position.z = transform.getOrigin().z();

        reply.pose.orientation.x = transform.getRotation().x();
        reply.pose.orientation.y = transform.getRotation().y();
        reply.pose.orientation.z = transform.getRotation().z();
        reply.pose.orientation.w = transform.getRotation().w();

        image_pub_.publish(reply);
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "ThermalCameraSave");
    ThermalImageReader ic;
    ros::spin();
    return 0;
}

