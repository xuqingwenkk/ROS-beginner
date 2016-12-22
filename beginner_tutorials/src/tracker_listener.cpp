#include "ros/ros.h"
#include "geometry_msgs/PoseStamped.h"


void chatterCallback(const geometry_msgs::PoseStamped::ConstPtr& pos)
{
  //ROS_INFO("Tracker Seq: [%d]", pos->header.seq);
  //ROS_INFO("Tracker Orientation x: [%f], y: [%f], z: [%f], w: [%f]", pos->pose.orientation.x,pos->pose.orientation.y,pos->pose.orientation.z,pos->pose.orientation.w);
  if(pos->pose.orientation.y<-0.66){
     double now=ros::Time::now().toSec();
     ROS_INFO("Tracker time: [%f]",now);
  }
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tracker_listener");

 
  ros::NodeHandle n;


  ros::Subscriber sub = n.subscribe("/vrpn_client_node/xsense/pose", 1000, chatterCallback);


  ros::spin();

  return 0;
}
