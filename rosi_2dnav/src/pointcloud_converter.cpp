#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

ros::Publisher pub;

void 
cloud_cb (const sensor_msgs::PointCloudConstPtr& msg)
{
  // Create a container for the data.
  sensor_msgs::PointCloud2 output;
  sensor_msgs::PointCloud input;
  input = *msg;

  // Do data processing here...
  sensor_msgs::convertPointCloudToPointCloud2(input, output);
  //output = *input;

  // Publish the data.
  pub.publish (output);
}

int
main (int argc, char** argv)
{
  // Initialize ROS
  ros::init (argc, argv, "pcl_conv");
  ros::NodeHandle nh;

  // Create a ROS subscriber for the input point cloud
  ros::Subscriber sub = nh.subscribe ("sensor/velodyne", 1, cloud_cb);

  // Create a ROS publisher for the output point cloud
  pub = nh.advertise<sensor_msgs::PointCloud2> ("sensor/velodyne2", 1);

  // Spin
  ros::spin ();
}
