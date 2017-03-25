#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <vector>
#include <algorithm>
#include "math.h"
#include <std_msgs/Float32MultiArray.h>
using namespace std;
using namespace cv;

#define Cx 1000
#define Cy 560
#define R1 130
#define R2 280
#define PI 3.1415926

 /// Global variables
image_transport::Publisher pub;

namespace nodelet_omni_image_cut
{
  class OmniImgCut : public nodelet::Nodelet
  {
  public:
    OmniImgCut()
    {
    }
  private:
    virtual void onInit();
    void zoomcallback(const std_msgs::Float32MultiArray::ConstPtr& param);
    void imgCallback(const sensor_msgs::Image::ConstPtr &img);
    Mat cutCal(Mat img, double centerX, double centerY, double radius);
    void update_map( void );
    
    ros::Subscriber zoomsub;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;
    Mat src, dst;
    Mat map_x, map_y;
    int Hd = R2 - R1;
    int Wd = PI * (R1+R2);
    std_msgs::Float32MultiArray zoom;
    bool flag = false;
  };
}