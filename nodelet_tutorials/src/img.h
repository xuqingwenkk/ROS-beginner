#include <nodelet/nodelet.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
using namespace std;
using namespace cv;

#define fishCx 983
#define fishCy 529
#define fishR 650
#define XTILT 0 
#define YROLL 1 
#define ZPAN  2 

#ifndef TRUE
#define TRUE  1
#define FALSE 0
#endif

#define DTOR 0.0174532925f
#define RTOD 57.2957795f

#define ABS(x) (x < 0 ? -(x) : (x))
#define SIGN(x) (x < 0 ? (-1) : 1)
#define MODULUS(p) (sqrt(p.x*p.x + p.y*p.y + p.z*p.z))

typedef struct {
   float x,y,z;
} XYZ;

typedef struct {
   int r,g,b;
} RGB;

typedef struct {
   int axis;
   float value;
   float cvalue,svalue;
} TRANSFORM;

typedef struct {
  Mat rgb;
  int width;
  int height;
  float theta;
} IMAGE;

typedef struct {
  Mat rgb;
  int width;
  int height;
  float theta;
  int cx,cy;
  int radius;
} FISHIMAGE;

typedef struct {
  vector<TRANSFORM> trans;
  int antialias;
  float persp_angle;
} SETTING;

namespace nodelet_image_cut
{
  class ImgCut : public nodelet::Nodelet
  {
  public:
    ImgCut()
    {
    }
  private:
    virtual void onInit();
    void zoomcallback(const std_msgs::Float32MultiArray::ConstPtr& param);
    void imgcallback(const sensor_msgs::Image::ConstPtr& img);
    void init_setting(SETTING & set,  double centerX, double centerY, double persp_angle);
    void CameraRay(float x,float y,XYZ *p,IMAGE & img);
    XYZ VectorSum(float,XYZ,float,XYZ,float,XYZ,float,XYZ);
    //XYZ operator-(XYZ p1, XYZ p2);
    void pixel_map(IMAGE & persp, FISHIMAGE & fishimage, SETTING & set);
    
    ros::Subscriber zoomsub;
    image_transport::Subscriber sub;
    image_transport::Publisher pub;
    SETTING detail_set;
    std_msgs::Float32MultiArray zoom;
    bool flag = false;
  };
}

