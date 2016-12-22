#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

static const std::string OPENCV_WINDOW = "Image Window"; //lose ";"

class ImageConverter
{
	ros::NodeHandle nh_;
	image_transport::ImageTransport it_;
	image_transport::Subscriber image_sub_;
	image_transport::Publisher image_pub_;

public:
ImageConverter()
: it_(nh_)
{
	// Subscrive to Input Video Feed and Publish Output Video Feed
	image_sub_ = it_.subscribe("/camera/image_raw", 1, &ImageConverter::imageCb, this);
	image_pub_ = it_.advertise("/image_converter/output_video",1);

	cv::namedWindow(OPENCV_WINDOW); //replace nameWindow with namedWindow

}

~ImageConverter()
{
	cv::destroyWindow(OPENCV_WINDOW);
}

void imageCb(const sensor_msgs::ImageConstPtr& msg)
{
	cv_bridge::CvImagePtr cv_ptr;// CvImagePtr not CVImagePtr
	try
	{
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
	}
	catch(cv_bridge::Exception& e)
	{
		ROS_ERROR("cv_bridge exception: %s", e.what());
		return;
	}

	// Draw an example circle on the video stream
	if (cv_ptr->image.rows > 60 && cv_ptr->image.cols >60)  //rows, not row
		cv::circle(cv_ptr->image, cv::Point(50, 50), 10, CV_RGB(255, 0, 0));
	// Update GUI Window
	cv::imshow(OPENCV_WINDOW, cv_ptr->image);
	cv::waitKey(3);

	//Output modified video stream
	image_pub_.publish(cv_ptr->toImageMsg());// toImageMsg() , lack ()

}

};

int main (int argc, char** argv)
{
	ros::init(argc, argv, "image_converter");
	ImageConverter ic;
	ros::spin();
	return 0;
}


