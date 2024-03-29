#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
//#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Image.h>
//#include <sensor_msgs/CameraInfo.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/core.hpp> 
//#include <limits>

//#define INF ((unsigned) ~0)
namespace enc = sensor_msgs::image_encodings;
//uint16_t inf = std::numeric_limits<T>::infinity();
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
    image_sub_ = it_.subscribe("/sensor/kinect_depth", 1,
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/camera/depth_registered/image_raw", 1);

  }

  ~ImageConverter()
  {

  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;

    	try
    	{
   	   cv_ptr = cv_bridge::toCvCopy(msg, enc::TYPE_16UC1);
   	   //cv_ptr = cv_bridge::toCvCopy(img, enc::MONO8);
  	  }
  	  catch (cv_bridge::Exception& e)
  	  {
   	   ROS_ERROR("cv_bridge exception: %s", e.what());
   	   return;
  	  }
  	  cv::flip(cv_ptr->image, cv_ptr->image, +1);

      //uint16_t dist = cv_ptr->image.at<uint16_t>(480,0);
      //std::cout << dist << std::endl;

      //for (int i=0; i<cv_ptr->image.rows; i++)
      
      //  for (int j=0; j<cv_ptr->image.cols; j++)
      //  {
      //    uint16_t dist = cv_ptr->image.at<uint16_t>(i,j);
          //std::cout << dist << std::endl;
      //    if (dist = 3500) 
      //    {
            //std::cout << "Pixel =0 :" << i << "," << j << "," << dist << std::endl;
            //cv_ptr->image.at<uint16_t>(i,j)= 0;
      //    }
      //  }
      //}



      // Output modified video stream
      image_pub_.publish(cv_ptr->toImageMsg());

	
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "kinect_converter_depth");
  ImageConverter ic;
  ros::spin();
  return 0;
}
