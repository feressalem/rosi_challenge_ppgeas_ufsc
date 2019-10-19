#include <ros/ros.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

#include <moveit_visual_tools/moveit_visual_tools.h>

#include <rosi_defy/ManipulatorJoints.h>
#include <sensor_msgs/JointState.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>

#include <tf/transform_datatypes.h>
#include <tf2/LinearMath/Quaternion.h>

#include <iostream>

#include <math.h>
#include <cmath>
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<rosi_defy::ManipulatorJoints>("/ur5/jointsPosTargetCommand", 1);

    //Topic you want to subscribe

    sub2_ = n_.subscribe("/odom", 1, &SubscribeAndPublish::callback2, this);


  }


  void callback2(const nav_msgs::Odometry::ConstPtr& msg)
  {
    tf::Quaternion q(msg->pose.pose.orientation.x , msg->pose.pose.orientation.y , msg->pose.pose.orientation.z , msg->pose.pose.orientation.w );
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    ROS_INFO("roll, pitch, yaw=%1.2f  %1.2f  %1.2f", roll, pitch, yaw);
    if (yaw <0 ) {
      yaw_sp = M_PI/2 + (-M_PI -yaw);
    } else {
      yaw_sp = M_PI/2 + (M_PI -yaw);
    }
    //pub_.publish(sim_time);
  }

  void orientationControll(){

    static const std::string PLANNING_GROUP = "manipulator";

    // The :move_group_interface:`MoveGroupInterface` class can be easily
    // setup using just the name of the planning group you would like to control and plan for.
    moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  
    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to add and remove collision objects in our "virtual world" scene
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Raw pointers are frequently used to refer to the planning group for improved performance.
    const robot_state::JointModelGroup* joint_model_group =
    move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
     moveit::planning_interface::MoveGroupInterface::Plan my_plan;
     //moveit::planning_interface::MoveGroupInterface group("tool_pointer");
     geometry_msgs::Pose current_pose = move_group.getCurrentPose().pose;
     
     //moveit_msgs::OrientationConstraint ocm;
     //ocm.link_name = "tool_pointer";
     //ocm.header.frame_id = "base_link";
     //ocm.orientation.x = current_pose.orientation.x;
     //ocm.orientation.y = current_pose.orientation.y;
     //ocm.orientation.z = current_pose.orientation.z;
     //ocm.orientation.w = current_pose.orientation.w;
     //ocm.absolute_x_axis_tolerance = 0.2;
     //ocm.absolute_y_axis_tolerance = 0.2;
     //ocm.absolute_z_axis_tolerance = 1.57;
     //ocm.weight = 1.0;

     //moveit_msgs::PositionConstraint pcm;
     //pcm.link_name = "tool_pointer";
     //pcm.header.frame_id = "base_link";
     //pcm.target_point_offset.x = current_pose.position.x;
     //pcm.target_point_offset.y = current_pose.position.y;
     //pcm.target_point_offset.z = current_pose.position.z;
     //pcm.constraint_region.x = 0.1;
     //pcm.constraint_region.y = 0.1;
     //pcm.constraint_region.z = 0.1;
//  ocm.orientation.w = 1.0;
//  ocm.absolute_x_axis_tolerance = 0.1;
//  ocm.absolute_y_axis_tolerance = 0.1;
//  ocm.absolute_z_axis_tolerance = 0.1;
     //pcm.weight = 1.0;

     //moveit_msgs::Constraints test_constraints;
     //test_constraints.orientation_constraints.push_back(ocm);
     //move_group.setPathConstraints(test_constraints);
     tf::Quaternion q_orig(current_pose.orientation.x, current_pose.orientation.y, current_pose.orientation.z, current_pose.orientation.w), q2, q_new;

     //tf2::Quaternion myQuaternion;
     q2.setRPY( 0, 0, yaw_sp );
     //std::cout << q2.x();
     q_new = q2*q_orig;
     q_new.normalize();

     geometry_msgs::Pose target_orientation;
      target_orientation.orientation.x = q_new.x();
      target_orientation.orientation.y = q_new.y();
      target_orientation.orientation.z = q_new.z();
      target_orientation.orientation.w = q_new.w();
      target_orientation.position.x = current_pose.position.x;
      target_orientation.position.y = current_pose.position.y;
      target_orientation.position.z = current_pose.position.z;
      move_group.setPoseTarget(target_orientation);
      move_group.setPlanningTime(5.0);

      bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
      ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

      move_group.move(); 
      //move_group.clearPathConstraints();
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub2_;
  double roll_sp = 0;
  double pitch_sp = 0;
  double yaw_sp;


};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "ur5_interface_node");
  ros::AsyncSpinner spinner(2);
  spinner.start();
  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  while(ros::ok()){
    SAPObject.orientationControll();  

  }



  return 0;
}

