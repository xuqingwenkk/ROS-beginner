#include <pluginlib/class_list_macros.h>
#include "omniImg.h"

PLUGINLIB_DECLARE_CLASS(nodelet_omni_image_cut, OmniImgCut, nodelet_omni_image_cut::OmniImgCut, nodelet::Nodelet);

double lessNum(double a, double b)
{
  if(a < b) return a;
  else return b;
}

double moreNum(double a, double b)
{
  if(a > b) return a;
  else return b;
}

namespace nodelet_omni_image_cut
{
  void OmniImgCut::onInit()
  {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    zoomsub = private_nh.subscribe("zoom", 10, &OmniImgCut::zoomcallback, this);
    image_transport::ImageTransport it(private_nh);
    pub = it.advertise("detail_view", 16);
    sub = it.subscribe("image_raw", 1, &OmniImgCut::imgCallback,this);
    dst.create( Hd, Wd, CV_8UC3 );
    map_x.create( dst.size(), CV_32FC1 );
    map_y.create( dst.size(), CV_32FC1 );
    update_map();
  }
  void OmniImgCut::zoomcallback(const std_msgs::Float32MultiArray::ConstPtr& param)
  {
    zoom = *param;
    flag = true;
  }
  void OmniImgCut::imgCallback(const sensor_msgs::Image::ConstPtr &img)
  {
      try
      {
	src = cv_bridge::toCvShare(img, "bgr8") -> image;
	float r=0.0, theta=0.0, Xs=0.0, Ys=0.0;
	remap( src, dst, map_x, map_y, CV_INTER_LINEAR, BORDER_CONSTANT, Scalar(0,0, 0) );
	Mat cutImage;
	if(flag)
	  cutImage = cutCal(dst, zoom.data[0], zoom.data[1], zoom.data[2]);
	else
	  cutImage = cutCal(dst, 500, 50, 256);
	sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", cutImage).toImageMsg();
	pub.publish(msg);
      }
      catch(cv_bridge::Exception& e)
      {
	ROS_ERROR("Could not convert from '%s' to 'bgr8'.", img->encoding.c_str());
      }
  }
  Mat OmniImgCut::cutCal(Mat img, double centerX, double centerY, double radius)
  {
    cout << "Hd is " << Hd << " Wd is " << Wd << endl;
    Mat result;
    double width = radius;
    double height = 0.75 * radius;
    
    if(((centerX - width) < 0)&&(centerX > 0))
    {
      cout << "1...." << endl;
      Mat img1 = img(Range(moreNum(0.0, centerY - height), lessNum(centerY + height, Hd)), Range(0, centerX + width));
      Mat flip_img1;
      flip(img1, flip_img1, 1);
      Mat img2 = img(Range(moreNum(0.0, centerY - height), lessNum(centerY + height, Hd)), Range(Wd + (centerX - width), Wd));
      Mat flip_img2;
      flip(img2, flip_img2, 1);
      result.create(img1.rows, img1.cols + img2.cols, CV_8UC3);
      flip_img1.copyTo(result(Rect(0,0,flip_img1.cols,flip_img1.rows)));
      flip_img2.copyTo(result(Rect(flip_img1.cols,0,flip_img2.cols,flip_img2.rows)));
    }
    else if(((centerX + width) > Wd)&&(centerX < Wd))
    {
      cout << "2...." << endl;
      Mat img1 = img(Range(moreNum(0.0, centerY - height), lessNum(centerY + height, Hd)), Range(centerX - width, Wd));
      Mat img2 = img(Range(moreNum(0.0, centerY - height), lessNum(centerY + height, Hd)), Range(0, centerX + width - Wd));
      result.create(img1.rows, img1.cols + img2.cols, CV_8UC3);
      img1.copyTo(result(Rect(0,0,img1.cols,img1.rows)));
      img2.copyTo(result(Rect(img1.cols, 0, img2.cols, img2.rows)));
    }
    else
    {
      cout << "3...." << endl;
      result = img(Range(moreNum(0.0, centerY - height), lessNum(centerY + height, Hd)), Range(centerX - width, centerX + width));
    }
    cout << result.size() << endl;
    return result;
  }
  void OmniImgCut::update_map( void )
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
}

