//-----------------------------------
// Unwrapping Omnidirectional Images
//-----------------------------------

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include "multiview.h"
using namespace std;
using namespace cv;

 /// Global variables
image_transport::Publisher pub_up;
image_transport::Publisher pub_down;
image_transport::Publisher pub_center;
image_transport::Publisher pub_left;
image_transport::Publisher pub_right;

void init_setting(SETTING & set, float angle, int axis);

SETTING set_up;
SETTING set_down;
SETTING set_center;
SETTING set_left;
SETTING set_right;
  
void imgCallback(const sensor_msgs::Image::ConstPtr &img)
{
  IMAGE persp;
  persp.width = 500;
  persp.height = 400;
  persp.theta = 30;
  persp.rgb = Mat(persp.height,persp.width,CV_8UC3,Scalar(0,0,0));
  
  FISHIMAGE fishimage;
  fishimage.rgb = cv_bridge::toCvShare(img, "bgr8") -> image;
  fishimage.width = fishimage.rgb.cols;
  fishimage.height = fishimage.rgb.rows;
  fishimage.theta = 190;
  fishimage.theta *= DTOR;
  fishimage.cx = 983;
  fishimage.cy = 529;
  fishimage.radius = 650;
  
  ROS_INFO("Image receive");
  
  pixel_map(persp,fishimage,set_up);
  ROS_INFO("Calc done");
  sensor_msgs::ImagePtr msg_up = cv_bridge::CvImage(std_msgs::Header(), "bgr8", persp.rgb).toImageMsg();
  ROS_INFO("To image done");
  pub_up.publish(msg_up);
  ROS_INFO("Publish done");
  
//   pixel_map(persp,fishimage,set_down);
//   sensor_msgs::ImagePtr msg_down = cv_bridge::CvImage(std_msgs::Header(), "bgr8", persp.rgb).toImageMsg();
//   pub_down.publish(msg_down);
//   
//   pixel_map(persp,fishimage,set_center);
//   sensor_msgs::ImagePtr msg_center = cv_bridge::CvImage(std_msgs::Header(), "bgr8", persp.rgb).toImageMsg();
//   pub_center.publish(msg_center);
//   
//   pixel_map(persp,fishimage,set_left);
//   sensor_msgs::ImagePtr msg_left = cv_bridge::CvImage(std_msgs::Header(), "bgr8", persp.rgb).toImageMsg();
//   pub_left.publish(msg_left);
//   
//   pixel_map(persp,fishimage,set_right);
//   sensor_msgs::ImagePtr msg_right = cv_bridge::CvImage(std_msgs::Header(), "bgr8", persp.rgb).toImageMsg();
//   pub_right.publish(msg_right);

//   ROS_INFO("Image Pub");

  
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "pseudo_unwrapper");
  ros::NodeHandle n;
  
  init_setting(set_up, -40.0, 0);
  init_setting(set_down, 40.0, 0);
  init_setting(set_center, 0.0, 0);
  init_setting(set_left, -45.0, 1);
  init_setting(set_right, 45.0, 1);
  
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("image_raw", 1, imgCallback);
  
  pub_up = it.advertise("up_view", 5);
  pub_down = it.advertise("down_view", 5);
  pub_center = it.advertise("center_view", 5);
  pub_left = it.advertise("left_view", 5);
  pub_right = it.advertise("right_view", 5);
  
  ros::spin();
  
  return 0;
}

void init_setting(SETTING & set, float angle, int axis)
{
  vector<TRANSFORM> trans;
  TRANSFORM temp;
  switch(axis)
  {
    case 0: 
      temp.axis = XTILT;
      temp.value = DTOR*angle;
      trans.push_back(temp);
      break;
    case 1:
      temp.axis = YROLL;
      temp.value = DTOR*angle;
      trans.push_back(temp);
      break;
    case 2:
      temp.axis = ZPAN;
      temp.value = DTOR*angle;
      trans.push_back(temp);
      break;
    default:
      break;
  }
  // Precompute transform sin and cosine
  for (int j=0;j<trans.size();j++) {
    trans[j].cvalue = cos(trans[j].value);
    trans[j].svalue = sin(trans[j].value);
  }
  set.trans = trans;
  set.antialias = 1;
}
