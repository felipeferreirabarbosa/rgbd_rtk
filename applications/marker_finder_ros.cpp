///ROS
#include <cstdio>
#include <cstdlib>
#include <ros/ros.h>
#include <sstream>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
///Opencv
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>
//Aruco
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
//Eigen
#include <Eigen/Geometry>
//Our includes
#include <geometry.h>
#include <rgbd_loader.h>
#include <klt_tracker.h>
#include <motion_estimator_ransac.h>
#include <reconstruction_visualizer.h>


using namespace std;
using namespace cv;
using namespace aruco;

KLTTracker tracker;
Intrinsics intr(0);
MotionEstimatorRANSAC motion_estimator(intr);
//ReconstructionVisualizer visualizer;
//eigen
Eigen::Affine3f cam_pose = Eigen::Affine3f::Identity();
Eigen::Affine3f trans = Eigen::Affine3f::Identity();
//pcl
pcl::PointCloud<PointT>::Ptr prev_cloud(new pcl::PointCloud<PointT>);
pcl::PointCloud<PointT>::Ptr curr_cloud(new pcl::PointCloud<PointT>);
//aruco
MarkerDetector marker_detector;
CameraParameters camera_params;
vector<Marker> markers;
float marker_size;
bool key_frame;

int i=0;

   
Eigen::Affine3f convertMarkerPoseToEigen(const Mat Rvec, const Mat Tvec){
Mat R = Mat::eye(3, 3, CV_32FC1);
Eigen::Affine3f P = Eigen::Affine3f::Identity();

Rodrigues(Rvec, R);

P(0,0) = R.at<float>(0,0); P(0,1) = R.at<float>(0,1); P(0,2) = R.at<float>(0,2);
P(1,0) = R.at<float>(1,0); P(1,1) = R.at<float>(1,1); P(1,2) = R.at<float>(1,2);
P(2,0) = R.at<float>(2,0); P(2,1) = R.at<float>(2,1); P(2,2) = R.at<float>(2,2);
P(0,3) = Tvec.at<float>(0,0); P(1,3) = Tvec.at<float>(1,0); P(2,3) = Tvec.at<float>(2,0);

return P;
}





void callback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD);
void rosMarkerFinder(cv::Mat rgb , cv::Mat depth);


int main(int argc, char** argv){    

  camera_params.readFromXMLFile(argv[1]);
  marker_size = stof(argv[2]);
  marker_detector.setDictionary("ARUCO_MIP_36h12", 0);

  //ROS steps


  ros::init(argc, argv, "image_converter");
  ros::start();
  ros::NodeHandle nh;

  message_filters::Subscriber<sensor_msgs::Image> rgb_sub(nh, "/camera/rgb/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> depth_sub(nh, "camera/depth/image_raw", 1);
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> MySyncPolicy;
  //ApproximateTime takes a queue size as its constructor argument, hence MySyncPolicy(10)
  message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), rgb_sub,depth_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
 }

void callback(const sensor_msgs::ImageConstPtr& msgRGB,const sensor_msgs::ImageConstPtr& msgD){

  // Copy the ros image message to cv::Mat.
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try{
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  cv_bridge::CvImageConstPtr cv_ptrD;
  try{
    cv_ptrD = cv_bridge::toCvShare(msgD);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  rosMarkerFinder(cv_ptrRGB->image, cv_ptrD->image);
}

void rosMarkerFinder(cv::Mat rgb , cv::Mat depth){

  *curr_cloud = getPointCloud(rgb, depth, intr);

  //Track feature points in the current frame
  key_frame=tracker.track(rgb);
  //Estimate motion between the current and the previous frame/point clouds

  if(i > 0){
    trans = motion_estimator.estimate(tracker.prev_pts_, prev_cloud, tracker.curr_pts_, curr_cloud);
    cam_pose = cam_pose*trans;
  }

  //Detect and view Aruco markers
  marker_detector.detect(rgb, markers, camera_params, marker_size);
  
  for (size_t j = 0; j < markers.size(); j++){
    markers[j].draw(rgb, Scalar(0,0,255), 1);
    CvDrawingUtils::draw3dAxis(rgb, markers[j], camera_params);
    Eigen::Affine3f marker_pose = convertMarkerPoseToEigen(markers[j].Rvec, markers[j].Tvec);
    marker_pose = cam_pose*marker_pose;
    stringstream ss;
    ss << "m" << markers[j].id;
    //visualizer.viewReferenceFrame(marker_pose, ss.str());
  }
  
  //3D vizualization
  /*
  //visualizer.addReferenceFrame(cam_pose, "origin");
  
  if(key_frame == true){
    visualizer.addQuantizedPointCloud(curr_cloud, 0.05, cam_pose);
    visualizer.viewReferenceFrame(cam_pose);
    visualizer.viewPointCloud(curr_cloud, cam_pose);
    visualizer.viewQuantizedPointCloud(curr_cloud, 0.02, cam_pose);
  }
  visualizer.spinOnce();
  */
  depth = depth/5;
  cv::imshow("OPENCV_WINDOW_DEPTH", rgb);
  cv::imshow("OPENCV_WINDOW", depth);
  cv::waitKey(1);

  *prev_cloud = *curr_cloud;

  i++;
}
