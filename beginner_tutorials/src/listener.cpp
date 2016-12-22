#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include "sensor_msgs/CameraInfo.h"
#include "sensor_msgs/CompressedImage.h"

int count =0;
//sensor_msgs::CameraInfo s;
sensor_msgs::CompressedImage s;
int flag=0;
void chatterCallback(const sensor_msgs::CompressedImage::ConstPtr& msg)
{
  flag=1;
  count++;
  ROS_INFO("I heard: [%s]", msg->format.c_str());
  s.data=msg->data;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "listener");

  ros::NodeHandle n;

  rosbag::Bag bag;
  bag.open("test2.bag",rosbag::bagmode::Write);

  ros::Subscriber sub = n.subscribe("camera/image/compressed", 1000, chatterCallback);

  while(ros::ok()){
    if(flag==1){
      //std_msgs::Int32 i;
      //i.data = count;
      bag.write("CompressedImage", ros::Time::now(), s);
      //bag.write("numbers", ros::Time::now(), i);
      flag=0;
    }
    ros::spinOnce();
  }
  bag.close();
  return 0;
}
