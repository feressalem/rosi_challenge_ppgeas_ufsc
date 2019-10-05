#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <rtabmap_ros/ResetPose.h>

int main(int argc, char** argv){
  ros::init(argc, argv, "set_odom_pose_node");

  ros::NodeHandle node;

  ros::service::waitForService("/rtabmap/reset_odom_to_pose");
  ros::ServiceClient spawner = node.serviceClient<rtabmap_ros::ResetPose>("/rtabmap/reset_odom_to_pose");
  rtabmap_ros::ResetPose pose;

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  int count = 0;
  ros::Rate rate(10.0);
  while (node.ok()){
    //ros::service::waitForService("reset_odom_to_pose");
    geometry_msgs::TransformStamped transformStamped;

    if (count == 0){

      try{
        transformStamped = tfBuffer.lookupTransform("map", "static_rosiInitialPose",
                                 ros::Time(0));
      }
      catch (tf2::TransformException &ex) {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
        continue;
      }
      tf::Quaternion q(transformStamped.transform.rotation.x, transformStamped.transform.rotation.y, transformStamped.transform.rotation.z, transformStamped.transform.rotation.w);
      tf::Matrix3x3 m(q);
      double roll, pitch, yaw;
      m.getRPY(roll, pitch, yaw);
      ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);

      pose.request.x = transformStamped.transform.translation.x;
      pose.request.y = transformStamped.transform.translation.y;
      pose.request.z = 0;
      pose.request.roll = 0;
      pose.request.pitch = 0;
      pose.request.yaw = yaw;

      if (spawner.call(pose))
      {
          ROS_INFO("Sucesso");
          count++;
      } else {
          ROS_ERROR("Failed to call service /rtabmap/reset_odom_to_pose");
          return 1;
      }

    } else {

      ros::shutdown();

    }

    rate.sleep();
  }
  return 0;
};

 