/* 
 *  Software License Agreement (BSD License)
 *
 *  Copyright (c) 2016, Natalnet Laboratory for Perceptual Robotics
 *  All rights reserved.
 *  Redistribution and use in source and binary forms, with or without modification, are permitted provided
 *  that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this list of conditions and
 *     the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and
 *     the following disclaimer in the documentation and/or other materials provided with the distribution.
 * 
 *  3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or
 *     promote products derived from this software without specific prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, *  INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE
 *  USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */
#include <cstdio>
#include <cstdlib>
#include <Eigen/Geometry>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <geometry.h>
#include <rgbd_loader.h>
#include <optical_flow_visual_odometry.h>
#include <reconstruction_visualizer.h>

#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <cv_bridge/cv_bridge.h>   
#include <image_transport/image_transport.h>


#define APPROXIMATE


#ifdef EXACT
#include <message_filters/sync_policies/exact_time.h>
#endif
#ifdef APPROXIMATE
#include <message_filters/sync_policies/approximate_time.h>
#endif



using namespace std;
using namespace message_filters;
using namespace cv;


void callback(const sensor_msgs::ImageConstPtr& msg_rgb ,const sensor_msgs::ImageConstPtr& msg_depth);
void initRos(int argc, char** argv, string rgb_topic, string depth_topic);
void optical_flow(cv::Mat frame, cv::Mat depth);

Intrinsics intr(0);
OpticalFlowVisualOdometry vo(intr);
ReconstructionVisualizer visualizer;
int i = 0;

int main(int argc, char** argv){
  string rgb_topic;
  string depth_topic;
  rgb_topic = "camera/rgb/image_raw"; 
  depth_topic = "camera/depth/image_raw";

  initRos(argc, argv, rgb_topic, depth_topic);


  return 0;
}


void callback(const sensor_msgs::ImageConstPtr& msg_rgb ,const sensor_msgs::ImageConstPtr& msg_depth){

  cv_bridge::CvImagePtr img_ptr_rgb;
  cv_bridge::CvImagePtr img_ptr_depth;
  try{
    img_ptr_depth = cv_bridge::toCvCopy(*msg_depth, sensor_msgs::image_encodings::TYPE_16UC1);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }
  try{
    img_ptr_rgb = cv_bridge::toCvCopy(*msg_rgb, sensor_msgs::image_encodings::TYPE_8UC3);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception:  %s", e.what());
    return;
  }

  Mat& mat_depth = img_ptr_depth->image;
  Mat& mat_rgb = img_ptr_rgb->image;

  optical_flow(mat_rgb,mat_depth);
}
void initRos(int argc, char** argv, string rgb_topic, string depth_topic){
  ros::init(argc, argv, "optical_flor_visual_odometry");
  ros::NodeHandle nh;


  message_filters::Subscriber<sensor_msgs::Image> subscriber_depth(nh , rgb_topic , 1);
  message_filters::Subscriber<sensor_msgs::Image> subscriber_rgb(nh , depth_topic , 1);


  #ifdef EXACT
      typedef sync_policies::ExactTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  #endif
  #ifdef APPROXIMATE
      typedef sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  #endif


  // ExactTime or ApproximateTime take a queue size as its constructor argument, hence MySyncPolicy(10)
  Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), subscriber_rgb, subscriber_depth );
  sync.registerCallback(boost::bind(&callback, _1, _2));


  ros::spin();
}
void optical_flow(cv::Mat frame, cv::Mat depth){


  //Estimate current camera pose
  vo.computeCameraPose(frame, depth);

  //View tracked points
  for(size_t k = 0; k < vo.tracker_.curr_pts_.size(); k++){
    Point2i pt1 = vo.tracker_.prev_pts_[k];
    Point2i pt2 = vo.tracker_.curr_pts_[k];
    circle(frame, pt1, 1, CV_RGB(255,0,0), -1);
    circle(frame, pt2, 3, CV_RGB(0,0,255), -1);
    line(frame, pt1, pt2, CV_RGB(0,0,255));
  }

  if(i == 0) visualizer.addReferenceFrame(vo.pose_, "origin");
  visualizer.addQuantizedPointCloud(vo.curr_dense_cloud_, 0.3, vo.pose_);
  visualizer.viewReferenceFrame(vo.pose_);
  visualizer.viewPointCloud(vo.curr_dense_cloud_, vo.pose_);
  //visualizer.viewQuantizedPointCloud(vo.curr_dense_cloud_, 0.02, vo.pose_);

  visualizer.spinOnce();

  //Show RGB-D image
  imshow("Image view", frame);
  imshow("Depth view", depth);

  i++;
}