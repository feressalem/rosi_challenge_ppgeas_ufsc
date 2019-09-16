#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp> 
#include <iostream>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>

using namespace std;
using namespace sensor_msgs;
using namespace message_filters;

image_transport::Publisher image_pub_;
ros::Publisher image_info_pub_;

void callback(  const ImageConstPtr& image_msg,
		const CameraInfoConstPtr& info_msg)   {
    cv_bridge::CvImagePtr cv_ptr;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(image_msg);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    // Flip in horizontal axis
    cv::flip(cv_ptr->image, cv_ptr->image, +1);

 	sensor_msgs::CameraInfo info;
  	  info.header = info_msg->header;
 	  info.header.frame_id = "camera_rgb_optical_frame";
  	  info.height = info_msg->height;
  	  info.width = info_msg->width;
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

int main(int argc, char** argv) {
	ros::init(argc, argv, "kinect_converter");

	ros::NodeHandle nh;
        image_transport::ImageTransport it(nh);

	image_transport::SubscriberFilter image_color_sub(it,"/sensor/kinect_rgb", 1);
	message_filters::Subscriber<CameraInfo> info_color_sub(nh,"/sensor/kinect_info", 1);

	image_pub_ = it.advertise("/camera/rgb/image_raw", 1);
        image_info_pub_ = nh.advertise<CameraInfo>("/camera/rgb/camera_info", 1);

	typedef sync_policies::ApproximateTime<Image, CameraInfo> MySyncPolicy;
	Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_color_sub, info_color_sub);

	sync.registerCallback(boost::bind(&callback, _1, _2));

	ros::spin();

	return 0;
}
