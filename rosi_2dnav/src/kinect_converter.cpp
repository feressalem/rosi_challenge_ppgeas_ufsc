#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp> 

namespace enc = sensor_msgs::image_encodings;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  ros::Publisher image_info_pub_;

public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/sensor/kinect_rgb", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/camera/rgb/image_color", 1);
    image_info_pub_ = nh_.advertise<sensor_msgs::CameraInfo>("/camera/rgb/camera_info", 1);

  }

  ~ImageConverter()
  {

  }

void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImageConstPtr cv_ptr;
    	try
    	{
   	   cv_ptr = cv_bridge::toCvCopy(msg, enc::BGR8);
   	   //cv_ptr = cv_bridge::toCvCopy(img, enc::MONO8);
  	  }
  	  catch (cv_bridge::Exception& e)
  	  {
   	   ROS_ERROR("cv_bridge exception: %s", e.what());
   	   return;
  	  }
  	  cv::flip(cv_ptr->image, cv_ptr->image, +1);

 	sensor_msgs::CameraInfo info;
  	  info.header = msg->header;
 	  info.header.frame_id = "camera_rgb_optical_frame";
  	  info.height = msg->height;
  	  info.width = msg->width;
  	  info.distortion_model = "plumb_bob";
  	  info.D = {0.0, 0.0, 0.0, 0.0, 0.0};
  	  info.K = {525.0, 0.0, 319.5, 0.0, 525.0, 239.5, 0.0, 0.0, 1.0};
  	  info.R = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
  	  info.P = {525.0, 0.0, 319.5, 0.0, 0.0, 525.0, 339.5, 0.0, 0.0, 0.0, 1.0, 0.0};
  	  info.binning_x = 0;
  	  info.binning_y = 0;
  	  info.roi.x_offset = 0;
  	  info.roi.y_offset = 0;
  	  info.roi.height = 0;
  	  info.roi.width = 0;
  	  info.roi.do_rectify = 0;

  	  // Output modified video stream
  	  image_pub_.publish(cv_ptr->toImageMsg());
   	  image_info_pub_.publish(info);
	
   }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "kinect_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}


