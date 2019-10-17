#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <pcl/point_types.h>
#include <tf2_ros/transform_listener.h>
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

ros::Publisher tf_pub;
tf::TransformListener *tf_listener; 

void callback(const PointCloud::ConstPtr& pcl_in)
{

  PointCloud pcl_out;

  //pcl_ros::transformPointCloud("base_link", *pcl_in, pcl_out, *tf_listener);
  pcl_ros::transformPointCloud("velodyne_new", *pcl_in, pcl_out, *tf_listener);
  pcl_conversions::toPCL(ros::Time::now(), pcl_out.header.stamp);
  tf_pub.publish(pcl_out);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "pcl_tf");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("/sensor/velodyne", 1, callback);
  //tf_pub = nh.advertise<PointCloud> ("/sensor/velodyne_bl", 1);
  tf_pub = nh.advertise<PointCloud> ("/sensor/velodyne_new", 1);

  tf_listener    = new tf::TransformListener();

  ros::spin();
  //delete tf_listener; 
  //return 0; 
}
