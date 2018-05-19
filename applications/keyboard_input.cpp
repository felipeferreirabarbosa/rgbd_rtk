#include "ros/ros.h"
#include "std_msgs/String.h"
#include <string>
#include <cstdio>
#include <cstdlib>

#include <sstream>

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "talker");
  ros::NodeHandle n;
  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);
  ros::Rate loop_rate(10);
  string id_marker;
  cout<<"q to quit\n";

  while (ros::ok())
  {
  
    std_msgs::String msg;
    std::stringstream ss;
    cout<<"Insira a ID\n";
    getline(cin, id_marker);
    cout << "You entered: " << id_marker << endl;
    ss << id_marker;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

 
    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();

    if(id_marker.compare("q") == 0)
      break;

  }


  return 0;
}