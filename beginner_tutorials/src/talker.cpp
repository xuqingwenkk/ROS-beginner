#include "ros/ros.h"
#include "std_msgs/String.h"

#include <sstream>

int main(int argc, char **argv)
{

  ros::init(argc, argv, "talker");

  ros::NodeHandle n;

  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  int count = 0;
  while (ros::ok())
  {

    std_msgs::String msg;

    std::stringstream ss;
    ss << "hello world " << count;
    msg.data = ss.str();

    ROS_INFO("%s", msg.data.c_str());

    chatter_pub.publish(msg);

    ros::spinOnce();

    loop_rate.sleep();
    ++count;
  }


  return 0;
}

/*#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Int32.h"
#include "rosbag/bag.h"
#include "rosbag/view.h"
#include <boost/foreach.hpp>
#include <sstream>
#include <iostream>
#include <vector>
#include "sensor_msgs/CompressedImage.h"

int main(int argc, char **argv)
{
  
  ros::init(argc, argv, "listener");


  ros::NodeHandle n;


  ros::Publisher chatter_pub = n.advertise<std_msgs::String>("chatter", 1000);

  ros::Rate loop_rate(10);

  rosbag::Bag bag("test2.bag");
  rosbag::View view(bag,rosbag::TopicQuery("CompressedImage"));
  BOOST_FOREACH(rosbag::MessageInstance const m,view){
     sensor_msgs::CompressedImage::ConstPtr i=m.instantiate<sensor_msgs::CompressedImage>();    
     if(i!=NULL)
         std::cout<<*i<<std::endl;
  } 

  //rosbag::Bag bag("test1.bag");
  //rosbag::View view(bag,rosbag::TopicQuery("numbers"));
  //BOOST_FOREACH(rosbag::MessageInstance const m,view){
  //   std_msgs::Int32::ConstPtr i=m.instantiate<std_msgs::Int32>();    
  //   if(i!=NULL)
  //      std::cout<<i->data<<std::endl;
  //}  

  //rosbag::View view2(bag,rosbag::TopicQuery("chatter"));
  //BOOST_FOREACH(rosbag::MessageInstance const m,view2){
  //   std_msgs::String::ConstPtr str=m.instantiate<std_msgs::String>();
  //   if(str!=NULL)
  //      std::cout<<str->data<<std::endl;
  //}  

  
//  rosbag::Bag bag;
//  bag.open("test.bag", rosbag::bagmode::Write);

//  int count = 0;
//  while (ros::ok())
//  {

//    std_msgs::String msg;

//    std::stringstream ss;
//    ss << "hello world " << count;
//    msg.data = ss.str();

//    std_msgs::Int32 i;
//   i.data = count;
//    bag.write("chatter", ros::Time::now(), msg);
//    bag.write("numbers", ros::Time::now(), i);
//    ROS_INFO("%s", msg.data.c_str());

//    chatter_pub.publish(msg);

//    ros::spinOnce();

//    loop_rate.sleep();
//    ++count;
//  }
  bag.close();

  return 0;
}
*/
