/**
 * @attention Copyright (C) 2017
 * @attention Carnegie Mellon University
 * @attention All rights reserved
 *
 * @author: AirLab / Field Robotics Center
 * @author: Geetesh Dubey
 *
 * @attention This code was modified under award #A018532.
 * @attention LIMITED RIGHTS:
 * @attention The US Government is granted Limited Rights to this Data.
 *            Use, duplication, or disclosure is subject to the
 *            restrictions as stated in award #A014692
 *  @author: Geetesh Dubey
 *
 */
/*
 * Copyright (c) 2016 Carnegie Mellon University, Author <gdubey@andrew.cmu.edu>
 *
 * For License information please see the LICENSE file in the root directory.
 *
 */

//Applies C-Space expansion on disparity images.
#include "ros/ros.h"
#include "sensor_msgs/Image.h"
#include "sensor_msgs/CameraInfo.h"
#include <sensor_msgs/Range.h>
#include "stereo_msgs/DisparityImage.h"
#include "geometry_msgs/PolygonStamped.h"
#include "visualization_msgs/MarkerArray.h"
#include "std_msgs/Header.h"
#include <cv_bridge/cv_bridge.h>
#include "opencv2/core/core.hpp"
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <image_geometry/pinhole_camera_model.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include "tf/tf.h"
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <vector>
#include <fstream>
#include <stdio.h>
#include <tictoc_profiler/profiler.hpp>

#include <Eigen/Core>
#include <opencv2/core/eigen.hpp>
#define SCALE 2.0 // num_units/pixel
//#define lut_max_disparity_   165 // units
//#define robot_radius_      2.0 // in meters
//for 2times Robsize = 0.0173 at 40m
//15 = 0.0677
#define DEPTH_ERROR_COEFF 0.0177

typedef pcl::PointCloud<pcl::PointXYZI> PointCloud;

class disparity_expansion{
public:
  disparity_expansion(ros::NodeHandle& nh);

  ros::Publisher expansion_cloud_pub;
  ros::Publisher expanded_disparity_fg_pub;
  ros::Publisher expanded_disparity_bg_pub;
  ros::Publisher expansion_poly_pub;
  ros::Subscriber subs_;
  ros::Subscriber cam_info_sub_;
  ros::Subscriber disparity_sub_;
  ros::Subscriber teraranger_sub_;

  float baseline;
  float downsample_scale;
  bool LUT_ready;
  bool got_cam_info;
  bool is_wire_;
  float wire_thresh_;
  float min_wire_depth_;
  image_geometry::PinholeCameraModel model_;

  void getCamInfo(const sensor_msgs::CameraInfo::ConstPtr &msg_info);
  void stereoDisparityCb(const stereo_msgs::DisparityImage::ConstPtr &msg_disp);
  void generateExpansionLUT();

  struct cell
  {
    unsigned int idx1;
    unsigned int idx2;
  };
  int lut_max_disparity_;//   165 // units
  float robot_radius_;//      2.0 // in meters
  float padding_;
  float bg_multiplier_;
  float pixel_error_;
  cell table_u[200][640];//[640];
  cell table_v[200][480];//[512];
  float table_d[200];
  float fx_,fy_,cx_,cy_;
  unsigned int width,height;
  void stereoDisparityProc(sensor_msgs::ImageConstPtr msg_disp);
};

void disparity_expansion::generateExpansionLUT()
{
  bool debug = false;
  if(debug)
    {
      ROS_WARN("Expansion LUT Debug disabled creation");
      LUT_ready = true;
    }
  if(LUT_ready)
    {
      ROS_ERROR("LUT all ready");
      return;
    }
  float center_x = cx_;
  float center_y = cy_;
  float constant_x = 1.0 / fx_;
  float constant_y = 1.0 / fy_;
  ROS_INFO("Fx Fy Cx Cy: %f %f , %f %f \nW H Baseline: %d %d %f",fx_,fy_,cx_,cy_,width,height,baseline);
  float r = robot_radius_;//expansion radius in cm
  int u1,u2,v1,v2;
  float x,y,z;
  float disparity;
  
  
  for(unsigned int disp_idx = 1;disp_idx< lut_max_disparity_ ;disp_idx++)
    {
      //        z = depth*0.01;//cm to m
      disparity = disp_idx/SCALE;//1 cell = 0.5m, z is in meters
      r = robot_radius_;// * exp(DEPTH_ERROR_COEFF*z);
      z = baseline * fx_ / disparity;

      float disp_new = baseline * fx_/(z - robot_radius_) + 0.5;
      table_d[disp_idx] = disp_new;


      for ( int v = ( int ) height - 1; v >= 0; --v )
        {
	  y = ( v - center_y ) * z * constant_y;

	  float beta = atan2(z,y);
	  float beta1 = asin(r/sqrt(z*z + y*y));

	  float r1 = z/tan(beta+beta1);
	  float r2 = z/tan(beta-beta1);
	  v1 = fy_*r1/z + center_y;
	  v2 = fy_*r2/z + center_y;

	  if((v2-v1)<0)
	    ROS_ERROR("Something MESSED %d %d %d",v1,v2,disp_idx);

	  if(v1 < 0) v1 = 0;
	  if(v1 > (height-1)) v1 = height-1;

	  if(v2 < 0) v2 = 0;
	  if(v2 > (height-1)) v2 = height-1;

	  table_v[disp_idx][v].idx1 = v1 ;
	  table_v[disp_idx][v].idx2 = v2 ;
        }

      for ( int u = ( int ) width - 1; u >= 0; --u )
        {
	  x = ( u - center_x ) * z * constant_x;

	  float alpha = atan2(z,x);
	  float alpha1 = asin(r/sqrt(z*z + x*x));

	  float r1 = z/tan(alpha+alpha1);
	  float r2 = z/tan(alpha-alpha1);
	  u1 = fx_*r1/z + center_x;
	  u2 = fx_*r2/z + center_x;

	  if((u2-u1)<0)
	    ROS_ERROR("Something MESSED %d %d %d",u1,u2,disp_idx);

	  if(u1 < 0) u1 = 0;
	  if(u1 > (width-1)) u1 = width-1;

	  if(u2 < 0) u2 = 0;
	  if(u2 > (width-1)) u2 = width-1;

	  table_u[disp_idx][u].idx1 = u1 ;
	  table_u[disp_idx][u].idx2 = u2 ;

        }
    }
  
  ROS_WARN("Expansion LUT created: LUT MAX: %d , ROBOT SIZE: %f",lut_max_disparity_/2,robot_radius_);
  LUT_ready = true;
}

void disparity_expansion::getCamInfo(const sensor_msgs::CameraInfo::ConstPtr& msg_info)
{
  if(got_cam_info)
    return;
  model_.fromCameraInfo ( msg_info );
  ROS_INFO_ONCE("Cam Info Recvd Fx Fy Cx Cy: %f %f , %f %f",model_.fx(),model_.fy(),model_.cx(),model_.cy());
  cx_ = model_.cx()/downsample_scale;
  cy_ = model_.cy()/downsample_scale;
  fx_ = fy_ = model_.fx()/downsample_scale;
    width = model_.reducedResolution().width/downsample_scale;
    height = model_.reducedResolution().height/downsample_scale;
//    width = msg_info->width/downsample_scale;
//    height = msg_info->height/downsample_scale;
    float baseline_temp = -msg_info->P[3]/msg_info->P[0];
  if(baseline_temp != 0.0)
    baseline = baseline_temp;
  baseline *=downsample_scale;
  ROS_WARN("Transformed Cam Info Recvd Fx Fy Cx Cy: %f %f , %f %f Baseline: %f with downsamplescale: %f",model_.fx(),model_.fy(),model_.cx(),model_.cy(),baseline,downsample_scale);
  generateExpansionLUT();
  got_cam_info = true;
}

void disparity_expansion::stereoDisparityCb(const stereo_msgs::DisparityImage::ConstPtr &msg_disp){
  sensor_msgs::ImageConstPtr I,I2;
  //I = msg_disp->image;

  sensor_msgs::Image I3 = msg_disp->image;
  I3.header = msg_disp->header;
  I2.reset(new sensor_msgs::Image(I3));

  stereoDisparityProc(I2);
}

void disparity_expansion::stereoDisparityProc(sensor_msgs::ImageConstPtr msg_disp){
  static int counter=0;

  ca::Profiler::tictoc("expansion_cb");

  if(!LUT_ready)
    {
      ROS_INFO_THROTTLE(1,"LUT not ready yet, not processing disparity");
      return;
    }

  cv_bridge::CvImagePtr fg_msg(new cv_bridge::CvImage());
  cv_bridge::CvImagePtr bg_msg(new cv_bridge::CvImage());


  cv_bridge::CvImageConstPtr cv_ptrdisparity;
  try
    {
      cv_ptrdisparity = cv_bridge::toCvShare(msg_disp);
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  cv::Mat disparity32F = cv_ptrdisparity->image;
    if(0)
  cv::resize(disparity32F, disparity32F, cv::Size(), 1.0/downsample_scale, 1.0/downsample_scale, cv::INTER_AREA);
    else
    {
        // Map the OpenCV matrix with Eigen:
//        Eigen::Map<Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>> A_Eigen(disparity32F.ptr<float>(), disparity32F.rows, disparity32F.cols);
        Eigen::MatrixXf dst(disparity32F.rows,disparity32F.cols);
        Eigen::MatrixXf resizedEigIm(int(disparity32F.rows/downsample_scale),int(disparity32F.cols/downsample_scale));
        cv::cv2eigen(disparity32F, dst);

        const int downsize = int(downsample_scale);

        for(uint i=0 ; i<disparity32F.rows ; i=i+downsample_scale)
        {
            for(uint j=0 ; j<disparity32F.cols ; j=j+downsample_scale)
            {
//                float max = dst.block<downsize,downsize>(i,j).maxCoeff();
                float max = dst.block(i,j,downsize,downsize).maxCoeff();
                resizedEigIm(int(i/downsample_scale),int(j/downsample_scale))= max;
            }
        }
        disparity32F.release();
        cv::eigen2cv(resizedEigIm, disparity32F);
        ROS_INFO_THROTTLE(1,"AFTER RESIZING %d %d",disparity32F.rows,disparity32F.cols);
    }
  
  cv::Mat disparity_fg;
  cv::Mat disparity_bg;
  cv::Mat disparity32F_bg;
  
  try
    {

        disparity_fg.create(disparity32F.rows,disparity32F.cols,disparity32F.type());
        disparity_fg.setTo(NAN);
        disparity_fg.copyTo(disparity_bg);
        disparity_fg.copyTo(disparity32F_bg);
    }
  catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
  
  fg_msg->header   = msg_disp->header;
  fg_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  bg_msg->header   = msg_disp->header;
  bg_msg->encoding = sensor_msgs::image_encodings::TYPE_32FC1;
  if(1)//make cloud
    {
      // Use correct principal point from calibration
      float center_x = cx_;
      float center_y = cy_;
      
      float constant_x = 1.0 / fx_;
      float constant_y = 1.0 / fy_;
      
      {
	ROS_INFO_ONCE("IMG TYPE 32FC, GOOD");
	ros::Time start = ros::Time::now();
	
	float padding = padding_;

    cv::Mat mask = disparity32F >= wire_thresh_;

    if(1)//specklefilter
    {
        cv::filterSpeckles(mask,0,400,4);
        if(1)//dilate
        {
            int dilation_size = 4;//2;
            cv::Mat disparity32F_filtered,disparity32F_filtered2;
            disparity32F.copyTo(disparity32F_filtered,mask);
            cv::Mat element = cv::getStructuringElement( cv::MORPH_RECT,
                                                         cv::Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                                         cv::Point( dilation_size, dilation_size ) );
//            cv::erode(disparity32F_filtered2,disparity32F_filtered,element, cv::Point(-1, -1), 2);
            cv::dilate(mask,mask,element, cv::Point(-1, -1), 2);
//            disparity32F_filtered2.convertTo(disparity32F_filtered,CV_8U);
//            disparity32F.copyTo(di_msg->image,disparity32F_filtered);
        }
    }

    disparity_fg.setTo(baseline * fx_/min_wire_depth_,mask);
    disparity_bg.setTo(baseline * fx_/(min_wire_depth_+padding),mask);

	
	fg_msg->image = disparity_fg;
	bg_msg->image = disparity_bg;
	
	
	ROS_INFO_THROTTLE(2.0, "Time: %f \t%f",(ros::Time::now()-start).toSec(),1/(ros::Time::now()-start).toSec());
	
	
	expanded_disparity_fg_pub.publish(fg_msg->toImageMsg());
	expanded_disparity_bg_pub.publish(bg_msg->toImageMsg());

	if(expansion_poly_pub.getNumSubscribers() > 0 )//create expansion PCD
	  {
	    visualization_msgs::MarkerArray marker_arr;
	    visualization_msgs::Marker marker;
	    marker.header = msg_disp->header;
	    marker.ns = "occ_space";
	    marker.id = 0;
	    marker.type = visualization_msgs::Marker::LINE_STRIP;
	    marker.lifetime = ros::Duration(0.5);
	    marker.action = visualization_msgs::Marker::ADD;
	    marker.pose.position.x =0;//xyz_centroid[0];
	    marker.pose.position.y =0;//xyz_centroid[1];
	    marker.pose.position.z =0;//xyz_centroid[2];

	    tf::quaternionTFToMsg(tf::createQuaternionFromRPY(0.0,0.0,0.0), marker.pose.orientation);
	    //                marker.pose.orientation.w = 1;
	    float marker_scale = 0.51;
	    marker.scale.x = marker_scale;
	    marker.scale.y = marker_scale;
	    marker.scale.z = marker_scale;
	    marker.color.a = 0.3; // Don't forget to set the alpha!
	    marker.color.r = 0.0;
	    marker.color.g = 0.0;
	    marker.color.b = 0.0;

	    geometry_msgs::PolygonStamped poly;
	    poly.header = msg_disp->header;
	    int v = 120;
	    float prev_depth = 0.0;
	    for ( int v = ( int ) 0; v <= 239; v+=10 )
	      {
		for ( int u = ( int ) width - 1; u >= 0; u-- )
		  {
		    float depth_value = baseline * fx_ / fg_msg->image.at<float>(v,u);//free_msg->image.at<cv::Vec4f>(v,u)[0];
		    float depth_diff = fabs(depth_value-prev_depth);
		    prev_depth = depth_value;
		    if(!std::isnan(depth_value) && !std::isinf(depth_value) && depth_diff < 0.5)
		      {
			marker.color.r = 1.0 * (fg_msg->image.at<float>(v,u)-pixel_error_)/fg_msg->image.at<float>(v,u);
			marker.color.g = 1.0 * (pixel_error_)/fg_msg->image.at<float>(v,u);
			geometry_msgs::Point gm_p;
			gm_p.x = ( u - center_x ) * depth_value * constant_x;
			gm_p.y = ( v - center_y ) * depth_value * constant_y;
			gm_p.z = depth_value;
			marker.points.push_back(gm_p);

			depth_value = baseline * fx_ / bg_msg->image.at<float>(v,u);
			gm_p.x = ( u - center_x ) * depth_value * constant_x;
			gm_p.y = ( v - center_y ) * depth_value * constant_y;
			gm_p.z = depth_value;
			marker.points.push_back(gm_p);
		      }
		    else
		      {
			marker_arr.markers.push_back(marker);
			marker.points.clear();
			marker.id++;
		      }
		  }
	      }
	    marker_arr.markers.push_back(marker);
	    expansion_poly_pub.publish(marker_arr);
	  }

        counter++;
        if (counter==2) counter=0;

	if((expansion_cloud_pub.getNumSubscribers() > 0 ) && (counter == 0))//create expansion PCD
	  {
	    PointCloud::Ptr cloud (new PointCloud);
	    cloud->header.frame_id = msg_disp->header.frame_id;
	    cloud->height = 1;
	    cloud->width =1;
	    cloud->is_dense = false;
	    int point_counter=0;
	    pcl::PointXYZI pt_fg,pt_bg,pt_free1,pt_free2,pt_real;

	    for ( int v = ( int ) height - 1; v >= 0; v-=4 )
	      {
		for ( int u = ( int ) width - 1; u >= 0; u-=4 )
		  {

		    // Fill in XYZ
		    float depth_value = baseline * fx_ / fg_msg->image.at<float>(v,u);
		    pt_fg.x = ( u - center_x ) * depth_value * constant_x;
		    pt_fg.y = ( v - center_y ) * depth_value * constant_y;
		    pt_fg.z = depth_value;
		    pt_fg.intensity = 220;

		    depth_value = baseline * fx_ / bg_msg->image.at<float>(v,u);
		    pt_bg.x = ( u - center_x ) * depth_value * constant_x;
		    pt_bg.y = ( v - center_y ) * depth_value * constant_y;
		    pt_bg.z = depth_value;
		    pt_bg.intensity = 120;
		    {
		      point_counter++;
		      cloud->points.push_back ( pt_fg );
		      point_counter++;
		      cloud->points.push_back ( pt_bg );

		    }
		  }
	      }
	    cloud->width  = point_counter;
	    cloud->header.seq = msg_disp->header.seq;
	    cloud->header.stamp = pcl_conversions::toPCL(msg_disp->header.stamp);

	    sensor_msgs::PointCloud2 cloud_PC2;
	    pcl::toROSMsg(*cloud,cloud_PC2);
	    expansion_cloud_pub.publish(cloud_PC2);
	  }
      }
    }
  ca::Profiler::tictoc("expansion_cb");
  return;
}

disparity_expansion::disparity_expansion(ros::NodeHandle& nh){
  expansion_cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/expansion/cloud_fg", 10);
  expansion_poly_pub = nh.advertise<visualization_msgs::MarkerArray>("/expansion/occ_marker", 10);
  expanded_disparity_fg_pub = nh.advertise<sensor_msgs::Image>("/ceye/left/expanded_disparity_fg", 10);
  expanded_disparity_bg_pub = nh.advertise<sensor_msgs::Image>("/ceye/left/expanded_disparity_bg", 10);
  ROS_INFO("into constr with nodehandle %s",nh.getNamespace().c_str());
  LUT_ready = false;
  got_cam_info = false;
  is_wire_ = true;
  cam_info_sub_ = nh.subscribe("/nerian_sp1/right/camera_info", 1,&disparity_expansion::getCamInfo,this);
  disparity_sub_ = nh.subscribe("/nerian_sp1/disparity_map_32F", 1,&disparity_expansion::stereoDisparityProc,this);

  nh.param("robot_radius", robot_radius_, (float)2.0);
  nh.param("lut_max_disparity", lut_max_disparity_, 164);
  nh.param("padding", padding_, (float)2.0);
  nh.param("bg_multiplier", bg_multiplier_, (float)5.0);
  nh.param("sensor_pixel_error", pixel_error_, (float)0.5);
  nh.param("wire_thresh", wire_thresh_, (float)0.1);
  nh.param("min_wire_depth", min_wire_depth_, (float)4.0);

  ros::NodeHandle global_nh("/oa");
  double temp;
  global_nh.param("downsample_scale", temp, 1.0);
  downsample_scale = (float)temp;
  global_nh.param("baseline", baseline, (float)0.10);

  cx_ = 317.20617294311523;
  cy_ = 233.2914752960205;
  fx_ = fy_ = 307.4838344732113;
  width = 640;
  height = 480;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "disparity_expansion");
  //	cv::initModule_nonfree();//THIS LINE IS IMPORTANT for using surf and sift features of opencv
  ros::NodeHandle nh("~");
  disparity_expansion d(nh);

  ca::Profiler::enable();

  ros::spin();

  ca::Profiler::print_aggregated(std::cerr);

  return 0;
}
