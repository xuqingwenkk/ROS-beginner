//-----------------------------------
// Unwrapping Omnidirectional Images
//-----------------------------------

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include "math.h"
using namespace std;
using namespace cv;

#define Cx 1000
#define Cy 560
#define R1 130
#define R2 280
#define PI 3.1415926

 /// Global variables
Mat src, dst;
Mat map_x, map_y;
int Hd = R2 - R1;
int Wd = PI * (R1+R2);
image_transport::Publisher pub;

/// Function Headers
void update_map( void );

void imgCallback(const sensor_msgs::Image::ConstPtr &img)
{
  try
  {
    src = cv_bridge::toCvShare(img, "bgr8") -> image;
    update_map();
    remap( src, dst, map_x, map_y, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );
    
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", dst).toImageMsg();
    pub.publish(msg);
    
    imshow("unwrap", dst);
    waitKey(30);
    imshow("view", src);
    waitKey(30);
 
  }
  catch(cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
  }
}

int main(int argc, char *argv[])
{
  ros::init(argc, argv, "unwrapper");
  ros::NodeHandle n;
  
  namedWindow("view");
  startWindowThread();
  
  dst.create( Hd, Wd, src.type() );
  map_x.create( dst.size(), CV_32FC1 );
  map_y.create( dst.size(), CV_32FC1 );
  
  namedWindow("unwrap");
  startWindowThread();
  
  image_transport::ImageTransport it(n);
  image_transport::Subscriber sub = it.subscribe("raw_image", 1000, imgCallback);
  
  pub = it.advertise("unwrap_image", 1);
  
  ros::spin();
  
  destroyWindow("unwrap");
  destroyWindow("view");
  
  return 0;
}

/**
 * @function update_map
 * @brief Fill the map_x and map_y matrices with 4 types of mappings
*/
void update_map( void )
{
  float r=0.0, theta=0.0, Xs=0.0, Ys=0.0;
  for( int j=0;j<dst.rows;j++)
  {
    for(int i=0;i<dst.cols;i++)
    {
      r=(float)j / (float)Hd *(R2 - R1) + R1;
      theta = (float)i / (float)Wd * 2 * PI;
      Xs = Cx + r * sin(theta);
      Ys = Cy + r * cos(theta);
      map_x.at<float>(j,i) = Xs;
      map_y.at<float>(j,i) = Ys;
    }
  }
}