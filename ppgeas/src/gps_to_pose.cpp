#include <ros/ros.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

  nav_msgs::Odometry gpsPose_;

void chatterCallback_gps(const sensor_msgs::NavSatFix::ConstPtr& msg)
{  

  ::gpsPose_.header = msg->header;
  //::gpsPose_.header.stamp = ros::Time::now();
  ::gpsPose_.header.frame_id = "odom";
  ::gpsPose_.child_frame_id = "base_link";
  ::gpsPose_.pose.pose.position.x = -msg->latitude;
  ::gpsPose_.pose.pose.position.y = -msg->longitude;
  ::gpsPose_.pose.pose.position.z = msg->altitude;
}


void chatterCallback_imu(const sensor_msgs::Imu::ConstPtr& msg)
{
  ::gpsPose_.pose.pose.orientation.x = msg->orientation.x;
  ::gpsPose_.pose.pose.orientation.y = msg->orientation.y;
  ::gpsPose_.pose.pose.orientation.z = msg->orientation.z;
  ::gpsPose_.pose.pose.orientation.w = msg->orientation.w;
}


int main(int argc, char** argv){
  ros::init(argc, argv, "odometry_node");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  
  ros::Subscriber sub_gps = n.subscribe<sensor_msgs::NavSatFix>("/sensor/gps", 1, chatterCallback_gps);
  ros::Subscriber sub_imu = n.subscribe<sensor_msgs::Imu>("/sensor/imu", 1, chatterCallback_imu);
  
  ros::Publisher gpsPosePub_ = n.advertise<nav_msgs::Odometry>("/rtabmap/gps_pose", 1, true);

  static tf2_ros::TransformBroadcaster br;
  geometry_msgs::TransformStamped transformStamped;

  while(ros::ok()){

    gpsPosePub_.publish(::gpsPose_);

    transformStamped.header = ::gpsPose_.header;
    transformStamped.header.frame_id = "odom";
    transformStamped.child_frame_id = base_link;
    transformStamped.transform.translation.x = ::gpsPose_.pose.pose.position.x;
    transformStamped.transform.translation.y = ::gpsPose_.pose.pose.position.y;
    transformStamped.transform.translation.z = ::gpsPose_.pose.pose.position.z;
    transformStamped.transform.rotation.x = ::gpsPose_.pose.pose.orientation.x;
    transformStamped.transform.rotation.y = ::gpsPose_.pose.pose.orientation.y;
    transformStamped.transform.rotation.z = ::gpsPose_.pose.pose.orientation.z;
    transformStamped.transform.rotation.w = ::gpsPose_.pose.pose.orientation.w;

    br.sendTransform(transformStamped);

    ros::spinOnce();
    loop_rate.sleep();
  }



  //move_base_msgs::MoveBaseGoal goal;

  //we'll send a goal to the robot to move 1 meter forward
  //goal.target_pose.header.frame_id = "base_link";
  //goal.target_pose.header.stamp = ros::Time::now();

 // goal.target_pose.pose.position.x = 1.0;
 // goal.target_pose.pose.orientation.w = 1.0;

  //ROS_INFO("Sending goal");
 // ac.sendGoal(goal);

 // ac.waitForResult();

 // if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
 //   ROS_INFO("Hooray, the base moved 1 meter forward");
 // else
 //   ROS_INFO("The base failed to move forward 1 meter for some reason");

  return 0;
}