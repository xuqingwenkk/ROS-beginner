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

#define Cx 960
#define Cy 540
#define R1 180
#define R2 270
#define PI 3.1415926

using namespace std;
using namespace cv;



/// Global variables
Mat src, dst;
Mat map_x, map_y;
float r=0.0, theta=0.0, Xd=0.0, Yd=0.0, Xs=0.0, Ys=0.0;

// size of image before unwrapping
int Hs = src.rows;
int Ws = src.cols;
// size of image after unwrapping
int Hd = R2 - R1;
int Wd = (R1 + R2) * PI;

int ind = 0 ;
void update_map( void )
 {
   for( int j = 0; j < src.rows; j++ )
   { for( int i = 0; i < src.cols; i++ )
       {
	 if ( (pow(i-Cx,2)+pow(j-Cy,2) > pow(R1,2)) && (pow(i-Cx,2)+pow(j-Cy,2) < pow(R2,2)) )
	 {
	    int r_sin = i - Cx ;
	    int r_cos = j - Cy ;
	    r = sqrt( pow(r_sin, 2) + pow(r_cos, 2) );
	    theta = atan( (float)r_cos / (float)r_sin ) ;
	    map_x.at<float>(j,i) = r - R1 ;
	    map_y.at<float>(j,i) = theta * Wd / (2.0 * PI) ;
	 }
	 else {
	   map_x.at<float>(j,i) = i ;
	   map_y.at<float>(j,i) = j ;
	 }
	 /*
	 int r_sin = i - Cx ;
	 int r_cos = j - Cy ;
	 r = sqrt( pow(r_sin, 2) + pow(r_cos, 2) );
	 theta = atan( (float)r_cos / (float)r_sin ) ;
	 map_x.at<float>(j,i) = r - R1 ;
         map_y.at<float>(j,i) = theta * Wd / (2.0 * PI) ;
	 */
       }
    }
}

void imgCallback(const sensor_msgs::Image::ConstPtr &img)
{
  try
  {
    src = cv_bridge::toCvShare(img, "bgr8") -> image;
    // initalize of the size of dst, map_x, map_y
    /*
    dst.create( Hd, Wd, src.type() );
    map_x.create( Hd, Wd, CV_32FC1 );
    map_y.create( Hd, Wd, CV_32FC1 ); 
    */
    
    dst.create( Hd, Wd, src.type() );
    map_x.create( dst.size(), CV_32FC1 );
    map_y.create( dst.size(), CV_32FC1 );
    
    //build the map
    
    
    for(int y = 0; y < Hd; y++)
    {
      for(int x = 0; x < Wd; x++)
      {
	
	r = (float) y / (float) Hd * (R2 - R1) + R1;
	theta = (float) x / (float) Wd * 2 * PI;
	Xs = Cx + r * sin(theta);
	Ys = Cy + r * cos(theta);
	map_x.at<float>(y,x) = Xs;
	map_y.at<float>(y,x) = Ys;
	//dst.at<float>(y,x) = src.at<float>(Xs, Ys);
	//map_x.at<int>(y,x) = x;
	//map_y.at<int>(y,x) = y;
      }
    }
    
    //update_map();
    //use remap to unwrap
    //remap( src, dst, map_x, map_y, INTER_LINEAR, BORDER_CONSTANT, Scalar(0, 0, 0) );

    cv::imshow("unwrap", dst);
    cv::imshow("view", src);
    cv::waitKey(1000);
    cv::waitKey(1000);
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

  cv::namedWindow("view");
  cv::startWindowThread();

  cv::namedWindow("unwrap", WINDOW_AUTOSIZE);
  cv::startWindowThread();
  
  ros::Subscriber sub = n.subscribe("image", 1000, imgCallback);
  
  ros::spin();
  
  cv::destroyWindow("unwrap");
  cv::destroyWindow("view");
  
  return 0;
}