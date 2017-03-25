#include <pluginlib/class_list_macros.h>
#include "img.h"

PLUGINLIB_DECLARE_CLASS(nodelet_image_cut, ImgCut, nodelet_image_cut::ImgCut, nodelet::Nodelet);

XYZ operator-(XYZ p1, XYZ p2)
{
  XYZ result;
  result.x = p1.x - p2.x;
  result.y = p1.y - p2.y;
  result.z = p1.z - p2.z;
  return result;
}

namespace nodelet_image_cut
{
  void ImgCut::onInit()
  {
    ros::NodeHandle& private_nh = getPrivateNodeHandle();
    //private_nh.getParam("pointX", pointX_);
    //private_nh.getParam("pointY", pointY_);
    //private_nh.getParam("persp_angle", persp_angle_);
    zoomsub = private_nh.subscribe("zoom", 10, &ImgCut::zoomcallback, this);
    image_transport::ImageTransport it(private_nh);
    pub = it.advertise("detail_view", 10);
    sub = it.subscribe("image_raw", 1, &ImgCut::imgcallback,this);
  }
  void ImgCut::zoomcallback(const std_msgs::Float32MultiArray::ConstPtr& param)
  {
    zoom = *param;
    flag = true;
  }
  void ImgCut::imgcallback(const sensor_msgs::Image::ConstPtr& img)
  {
    if(flag)
      init_setting(detail_set, zoom.data[0], zoom.data[1],zoom.data[2]);
    else
      init_setting(detail_set, 500, 300, 30);
    IMAGE persp;
    persp.width = 500;
    persp.height = 400;
    persp.theta = detail_set.persp_angle;
    persp.rgb = Mat(persp.height,persp.width,CV_8UC3,Scalar(0,0,0));
      
    FISHIMAGE fishimage;
    fishimage.rgb = cv_bridge::toCvCopy(img, "bgr8") -> image;
    fishimage.width = fishimage.rgb.cols;
    fishimage.height = fishimage.rgb.rows;
    fishimage.theta = 190;
    fishimage.theta *= DTOR;
    fishimage.cx = fishCx;
    fishimage.cy = fishCy;
    fishimage.radius = fishR;
      
    pixel_map(persp,fishimage,detail_set);
    sensor_msgs::ImagePtr msg_up = cv_bridge::CvImage(std_msgs::Header(), "bgr8", persp.rgb).toImageMsg();
    pub.publish(msg_up);
  }
    
  void ImgCut::init_setting(SETTING & set,  double centerX, double centerY, double persp_angle)
  {
    vector<TRANSFORM> trans;
    TRANSFORM temp;
    temp.axis = XTILT;
    temp.value = DTOR*((centerX-fishCx)/fishR*90);
    trans.push_back(temp);
    
    temp.axis = YROLL;
    temp.value = DTOR*((centerY-fishCy)/fishR*90);
    trans.push_back(temp);
    // Precompute transform sin and cosine
    for (int j=0;j<trans.size();j++) {
      trans[j].cvalue = cos(trans[j].value);
      trans[j].svalue = sin(trans[j].value);
    }
    set.trans = trans;
    set.antialias = 1;
    set.persp_angle = persp_angle;
  }
  
  void ImgCut::CameraRay(float x,float y,XYZ *p,IMAGE & img)
  {
    float h,v;
    float dh,dv;
    XYZ vp = {0,0,0},vd = {0,1,0}, vu = {0,0,1}; /* Camera view position, direction, and up */
    XYZ right = {1,0,0};
    static XYZ p1,p2,p3,p4; /* Corners of the view plane */
    static int first = TRUE;
    static XYZ deltah,deltav;
    static float inversew,inverseh;
    // Precompute what we can just once
    if (first) {
      dh = tan(img.theta * DTOR / 2);
      dv = img.height * dh / img.width;
      p1 = VectorSum(1.0,vp,1.0,vd,-dh,right, dv,vu);
      p2 = VectorSum(1.0,vp,1.0,vd,-dh,right,-dv,vu);
      p3 = VectorSum(1.0,vp,1.0,vd, dh,right,-dv,vu);
      p4 = VectorSum(1.0,vp,1.0,vd, dh,right, dv,vu);
      deltah = p4 - p1;
      deltav = p2 - p1;
      inversew = 1.0 / img.width;
      inverseh = 1.0 / img.height;
      first = FALSE;
    }
    h = x * inversew;
    v = (img.height - 1 - y) * inverseh;
    p->x = p1.x + h * deltah.x + v * deltav.x;
    p->y = p1.y + h * deltah.y + v * deltav.y;
    p->z = p1.z + h * deltah.z + v * deltav.z;
  }

  void ImgCut::pixel_map(IMAGE & persp, FISHIMAGE & fishimage, SETTING & set)
  {
    int i,j,k,ai,aj;
    int u,v;
    float x,y,r,phi,theta;
    XYZ p,q;
    float inversealias = 1.0 / set.antialias;
    float aliasscale = inversealias * inversealias;
    float rscale = fishimage.radius / (0.5 * fishimage.theta);
    int cR=0,cG=0,cB=0;
    for (j=0;j<persp.height;j++) {
      for (i=0;i<persp.width;i++) {
	cR = 0;
	cG = 0;
	cB = 0;
	// Antialiasing loops, sub-pixel sampling
	for (ai=0;ai<set.antialias;ai++) {
	  x = i + ai * inversealias;
	  for (aj=0;aj<set.antialias;aj++) {
	    y = j + aj * inversealias;
	    // Calculate vector to each pixel in the perspective image 
	    CameraRay(x,y,&p,persp);
	    // Apply rotations
	    for (k=0;k<set.trans.size();k++) {
	      switch(set.trans[k].axis) {
		case XTILT:
		  q.x =  p.x;
		  q.y =  p.y * set.trans[k].cvalue + p.z * set.trans[k].svalue;
		  q.z = -p.y * set.trans[k].svalue + p.z * set.trans[k].cvalue;
		  break;
		case YROLL:
		  q.x =  p.x * set.trans[k].cvalue + p.z * set.trans[k].svalue;
		  q.y =  p.y;
		  q.z = -p.x * set.trans[k].svalue + p.z * set.trans[k].cvalue;
		  break;
		case ZPAN:
		  q.x =  p.x * set.trans[k].cvalue + p.y * set.trans[k].svalue;
		  q.y = -p.x * set.trans[k].svalue + p.y * set.trans[k].cvalue;
		  q.z =  p.z;
		  break;
	      }
	      p = q;
	    }
	    // Convert to fisheye image coordinates 
	    theta = atan2(sqrt(p.x * p.x + p.z * p.z),p.y);
	    phi = atan2(p.z,p.x);
	    r = rscale * theta; 
	    // Convert to fisheye texture coordinates 
	    u = fishimage.cx + r * cos(phi);
	    if (u < 0 || u >= fishimage.width)
	      continue;
	    v = fishimage.cy + r * sin(phi);
	    if (v < 0 || v >= fishimage.height)
	      continue;
	  
	    // Add up antialias contribution
	    cR += int(fishimage.rgb.at<Vec3b>(v,u)[0]);
	    cG += int(fishimage.rgb.at<Vec3b>(v,u)[1]);
	    cB += int(fishimage.rgb.at<Vec3b>(v,u)[2]);
	  
	  }
	}
      
	// Set the pixel 
	persp.rgb.at<Vec3b>(j,i)[0] = cR * aliasscale;
	persp.rgb.at<Vec3b>(j,i)[1] = cG * aliasscale;
	persp.rgb.at<Vec3b>(j,i)[2] = cB * aliasscale;
      
      }
    }
  }

/* 
   Sum 4 vectors each with a scaling factor
   Only used 4 times for the first pixel
*/ 
XYZ ImgCut::VectorSum(float d1,XYZ p1,float d2,XYZ p2,float d3,XYZ p3,float d4,XYZ p4)
{  
   XYZ sum;
   
   sum.x = d1 * p1.x + d2 * p2.x + d3 * p3.x + d4 * p4.x;
   sum.y = d1 * p1.y + d2 * p2.y + d3 * p3.y + d4 * p4.y;
   sum.z = d1 * p1.z + d2 * p2.z + d3 * p3.z + d4 * p4.z;
   
   return(sum); 
}  
}