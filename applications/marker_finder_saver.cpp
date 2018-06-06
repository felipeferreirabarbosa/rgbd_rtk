#include <cstdio>
#include <cstdlib>
#include <fstream>
///ROS
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include <cv_bridge/cv_bridge.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/time_synchronizer.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
///Opencv
#include <opencv2/highgui/highgui.hpp>
//Aruco
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>



using namespace std;
using namespace cv;
using namespace aruco;


//aruco
MarkerDetector marker_detector;
CameraParameters camera_params;
vector<Marker> markers;
float marker_size;


int i=0;

//where markers id and poses will be saved
struct markerFound{
  int id;
  double x_pose;
  double y_pose;
  double z_pose;
};

string listen_id;
int listen_id_to_int;
markerFound all_markers[255];

void imageCallback(const sensor_msgs::ImageConstPtr& msg); //subscribe to rgb image
void markerFinder(cv::Mat rgb); //marker finder
void listenKeyboardSave(const std_msgs::String::ConstPtr& msg); //listening keyboard input for navigation


int main(int argc, char** argv){    
  string rgb_topic;
  rgb_topic = "camera/rgb/image_raw";
  if(argc != 4 && argc !=3){
    fprintf(stderr, "Usage: %s <camera calibration file> <marker size> optional: <rgb_topic> ....bye default : camera/rgb/image_raw \n", argv[0]);
    exit(0);
  }
  if(argc == 3){
    printf(" By defult using camera/rgb/image_raw as ros topic\n");  
  }
   if(argc == 4){
     rgb_topic = argv[3];
  }
  camera_params.readFromXMLFile(argv[1]);    //aruco params 
  marker_size = stof(argv[2]);
  marker_detector.setDictionary("ARUCO_MIP_36h12", 0);

  
  for(int k=0; k<=254; k++){ //initializing markers
    all_markers[k].id = 0;
  }

  ros::init(argc, argv, "marker_finder_ros");    //starting ros
  ros::start();

  ros::NodeHandle n;
  ros::Subscriber sub = n.subscribe("chatter", 1000, listenKeyboardSave);    //subscribing to string msg 

  ros::NodeHandle nh;
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber rgb_sub = it.subscribe(rgb_topic, 1, imageCallback);    //subscribing to rgb image

  ros::spin();  //"while true"

  return 0;
 }

void imageCallback(const sensor_msgs::ImageConstPtr& msgRGB){
  cv_bridge::CvImageConstPtr cv_ptrRGB;
  try{
      cv_ptrRGB = cv_bridge::toCvShare(msgRGB);
  }
  catch (cv_bridge::Exception& e){
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }

  markerFinder(cv_ptrRGB->image); //calling marker finder funcition
}

void markerFinder(cv::Mat rgb ){

  marker_detector.detect(rgb, markers, camera_params, marker_size);   //Detect and view Aruco markers

  for (size_t j = 0; j < markers.size(); j++){
    all_markers[markers[j].id].id = markers[j].id;     //save all markers in a vetor 
    markers[j].draw(rgb, Scalar(0,0,255), 1);   //drawing markers in rgb image
    CvDrawingUtils::draw3dAxis(rgb, markers[j], camera_params);
    stringstream ss;
    ss << "m" << markers[j].id;
  }
   
  cv::imshow("OPENCV_WINDOW", rgb);  //showing rgb image
  cv::waitKey(1);

  i++;
}

void listenKeyboardSave(const std_msgs::String::ConstPtr& msg){
  string listen = msg->data.c_str();

  if(listen.compare("s") == 0){  //validing if string msg wants to save markers
    ofstream arq;
    arq.open("all_markers.txt");
    for(int k=0; k<=254; k++){
      if(all_markers[k].id==0) continue;
        arq<<all_markers[k].id<<endl;   //saving all markers in "all_markers.txt"
    }
  }
  else 
    ROS_INFO("[%s] is not a valid input, use 's' to save all markers", msg->data.c_str());
}
