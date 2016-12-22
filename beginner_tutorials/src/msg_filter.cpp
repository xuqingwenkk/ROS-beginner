#include <ros/ros.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <geometry_msgs/PoseStamped.h>
#include <iostream>
#include <beginner_tutorials/myNum.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;
using namespace geometry_msgs;

// global variable
ros::Publisher pub1,pub2;
int flag=0;
int cnt=0;

typedef message_filters::sync_policies::ApproximateTime<CameraInfo, PoseStamped> sync_policy_classification;

void callback(const CameraInfoConstPtr& cam_info, const PoseStampedConstPtr& pos)
{
  // Solve all of perception here...
  geometry_msgs::PoseStamped pose_filter;
  pose_filter.pose = pos->pose;
  pub1.publish(pose_filter);
  flag=1;
  cnt++;
  cout << "I should record the pose: " << cnt << endl;
}

void imageCallback(const ImageConstPtr& image)
{
  sensor_msgs::Image img;
  if(flag==1)
  {
    flag=0;
    cout << "I should record the image "<< cnt << endl;
    img = *image;
    pub2.publish(img);
  }
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "msg_filter_node");

  ros::NodeHandle nh;
  
  pub1 = nh.advertise<geometry_msgs::PoseStamped>("/raw_pose_filter", 100);
  pub2 = nh.advertise<sensor_msgs::Image>("raw_image",100);
  
  ros::Subscriber sub1=nh.subscribe("/camera/image_raw_drop", 100, imageCallback);

  message_filters::Subscriber<CameraInfo> info_sub(nh, "/camera/camera_info_drop", 1);
  message_filters::Subscriber<PoseStamped> pose_sub(nh, "/vrpn_client_node/robot/pose", 1);
  message_filters::Synchronizer<sync_policy_classification> sync(sync_policy_classification(100), info_sub, pose_sub);
  //TimeSynchronizer<CameraInfo, PoseStamped> sync(info_sub, pose_sub, 10);
  sync.registerCallback(boost::bind(&callback, _1, _2));

  ros::spin();

  return 0;
}