#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/calib3d.hpp>
#include <vector>

static const std::string OPENCV_WINDOW = "Image window";
//cv::Size chessboardSize = cv::Size(11, 9);
cv::Size chessboardSize = cv::Size(8, 6);

class Chessboard_Detector
{
    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Subscriber image_sub_;
    image_transport::Publisher image_pub_;

public:
    Chessboard_Detector()
            : it_(nh_)
    {
        image_sub_ = it_.subscribe("/thermal/image", 1,
                                   &Chessboard_Detector::imageCb, this);
        image_pub_ = it_.advertise("/image_converter/output_video", 1);

        cv::namedWindow(OPENCV_WINDOW);
    }

    ~Chessboard_Detector()
    {
        cv::destroyWindow(OPENCV_WINDOW);
    }

    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        }
        catch (cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }

        std::vector<cv::Point2f> corners;
        bool foundCorners;

        cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_BGR2GRAY);
        cv::bitwise_not(cv_ptr->image, cv_ptr->image);
//        cv::bitwise_not(cv_ptr->image, cv_ptr->image);
        foundCorners = findChessboardCorners(cv_ptr->image, chessboardSize, corners, cv::CALIB_CB_ADAPTIVE_THRESH + cv::CALIB_CB_NORMALIZE_IMAGE);

        if (foundCorners)
        {
            cv::cornerSubPix(cv_ptr->image, corners, chessboardSize, cv::Size(-1, -1),
                             cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 50, 0.1));

            cv::cvtColor(cv_ptr->image, cv_ptr->image, cv::COLOR_GRAY2RGB);
            cv::drawChessboardCorners(cv_ptr->image, chessboardSize, corners, foundCorners);
        }

        // Update GUI Window
        cv::imshow(OPENCV_WINDOW, cv_ptr->image);
        cv::waitKey(3);

        // Output modified video stream
        image_pub_.publish(cv_ptr->toImageMsg());
    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "detect_chessboard");
    Chessboard_Detector ic;
    ros::spin();
    return 0;
}

