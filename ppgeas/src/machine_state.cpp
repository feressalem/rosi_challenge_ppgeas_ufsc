#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <rosi_defy/RosiMovementArray.h>
#include <rosi_defy/RosiMovement.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <rtabmap_ros/ResetPose.h>
#include <std_srvs/Empty.h>


float array_arm_pos[2] = {};
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int State = 0;

void chatterCallback1(const rosi_defy::RosiMovementArray::ConstPtr& msg)
{
  ::array_arm_pos[0]= msg->movement_array[2].joint_var;
  ::array_arm_pos[1]= msg->movement_array[1].joint_var;
  //ROS_INFO(array_arm_pos[0]);
}

int main(int argc, char** argv){
  ros::init(argc, argv, "machine_state");
  ros::NodeHandle n;
  ros::Rate loop_rate(10);
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ros::Subscriber sub = n.subscribe<rosi_defy::RosiMovementArray>("/rosi/arms_joints_position", 1, chatterCallback1);
  ros::service::waitForService("/rtabmap/reset_odom_to_pose");
  ros::ServiceClient spawner = n.serviceClient<rtabmap_ros::ResetPose>("/rtabmap/reset_odom_to_pose");
  ros::service::waitForService("/rtabmap/reset");
  ros::ServiceClient spawner2 = n.serviceClient<std_srvs::Empty>("/rtabmap/reset");
  ros::service::waitForService("/move_base/clear_costmaps");
  ros::ServiceClient spawner3 = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");  


  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);
  geometry_msgs::TransformStamped transformStamped;
  
  move_base_msgs::MoveBaseGoal goal;
  rtabmap_ros::ResetPose pose;
  std_srvs::Empty c;
  while (ros::ok())
    {
    if (State == 0){
      if (::array_arm_pos[0] <= -2){
        ROS_INFO("mudando estado");
        State++;
      }
    }
    if (State == 1){

        ROS_INFO("Estado 1 : resetando mapa e odometria");
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
            ROS_INFO("Odometria resetada com sucesso");
            
        } else {
            ROS_ERROR("Failed to call service /rtabmap/reset_odom_to_pose");
            return 1;
        }  
        if (spawner2.call(c))
        {
            ROS_INFO("Mapa resetado com sucesso");
            
        } else {
            ROS_ERROR("Failed to call service /rtabmap/reset_odom_to_pose");
            return 1;
        } 
        if (spawner2.call(c))
        {
            ROS_INFO("Mudando estado");
            // State++; //MUDEI de volta
            
        } else {
            ROS_ERROR("Failed to call service /rtabmap/reset_odom_to_pose");
            return 1;
        } 
        if (spawner3.call(c))
        {
            ROS_INFO("Limpando costmaps");
            State++; //MUDEI de volta
            
        } else {
            ROS_ERROR("Failed to call service /rtabmap/reset_odom_to_pose");
            return 1;
        } 
    }      
    if (State == 2){
      ROS_INFO("Estado 2 : navegacao");

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = -1.0;
      goal.target_pose.pose.position.y = 2.0;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 1.0;
      goal.target_pose.pose.orientation.w = 0.0;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved to first goal");
        
      }
      else{
        ROS_INFO("The base failed to move forward 1 meter for some reason");
      }
      State++;
    }
    if (State == 3){
      ROS_INFO("Estado 3 : navegacao 2");

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = -3.0;
      goal.target_pose.pose.position.y = 2.2;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 1.0;
      goal.target_pose.pose.orientation.w = 0.0;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved to second goal");
        State++;
      }
      else{
        ROS_INFO("The base failed to move forward 1 meter for some reason");
      }
    }
    if (State == 4){
      
      ROS_INFO("Estado 4 : navegacao 3");

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = -5.5;
      goal.target_pose.pose.position.y = 1.9;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 1.0;
      goal.target_pose.pose.orientation.w = 0.0;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved to a long goal");
        State++;
      }
      else{
        ROS_INFO("The base failed to move forward 1 meter for some reason");
      }
    }

    if (State == 5){ // Enfiar o Braço até o rolo
      
      ROS_INFO("Estado 5 : Tocar o rolo");

      State++;
    }
    if (State == 6){
      
      ROS_INFO("Estado 6 : navegacao 5");

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = -12.0;
      goal.target_pose.pose.position.y = 2.2;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 1.0;
      goal.target_pose.pose.orientation.w = 0.0;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved to a long goal");
        State++;
      }
      else{
        ROS_INFO("The base failed to move forward 1 meter for some reason");
      }
    }

    if (State == 7){
      
     ROS_INFO("Estado 7 : navegacao 6");

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = -19.0;
      goal.target_pose.pose.position.y = 2.4;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 1.0;
      goal.target_pose.pose.orientation.w = 0.0;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved to a long goal");
        State++;
      }
      else{
        ROS_INFO("The base failed to move forward 1 meter for some reason");
      }

    }
        if (State == 8){
      
     ROS_INFO("Estado 8 : navegacao 6");

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = -26.0;
      goal.target_pose.pose.position.y = 2.6;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;      
      goal.target_pose.pose.orientation.z = 1.0;
      goal.target_pose.pose.orientation.w = 0.0;


      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved to a long goal");
        State++;
      }
      else{
        ROS_INFO("The base failed to move forward 1 meter for some reason");
      }

    }
    if (State == 9){
      
     ROS_INFO("Estado 9 : navegacao 7");

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = -33.0;
      goal.target_pose.pose.position.y = 2.8;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 1.0;
      goal.target_pose.pose.orientation.w = 0.0;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved to the finish goal");
        State++;
      }
      else{
        ROS_INFO("The base failed to move forward 1 meter for some reason");
      }

    }
    if (State == 10){
      
     ROS_INFO("Estado 10 : navegacao 8");

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = -40.0;
      goal.target_pose.pose.position.y = 3.0;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 1.0;
      goal.target_pose.pose.orientation.w = 0.0;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved to the finish goal");
        State++;
      }
      else{
        ROS_INFO("The base failed to move forward 1 meter for some reason");
      }

    }
    if (State == 11){
      
     ROS_INFO("Estado 11 : navegacao 7");

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = -47.0;
      goal.target_pose.pose.position.y = 3.2;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 1.0;
      goal.target_pose.pose.orientation.w = 0.0;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved to the finish goal");
        State++;
      }
      else{
        ROS_INFO("The base failed to move forward 1 meter for some reason");
      }

    }
    if (State == 12){
      
     ROS_INFO("Estado 9 : navegacao 7");

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = -54.0;
      goal.target_pose.pose.position.y = 3.4;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 1.0;
      goal.target_pose.pose.orientation.w = 0.0;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved to the finish goal");
        State++;
      }
      else{
        ROS_INFO("The base failed to move forward 1 meter for some reason");
      }

    }
    if (State == 13){
      
     ROS_INFO("Estado 9 : navegacao 7");

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = -61.0;
      goal.target_pose.pose.position.y = 3.6;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 1.0;
      goal.target_pose.pose.orientation.w = 0.0;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved to the finish goal");
        State++;
      }
      else{
        ROS_INFO("The base failed to move forward 1 meter for some reason");
      }

    }
    if (State == 14){
      
     ROS_INFO("Estado 9 : navegacao 7");

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = -68.0;
      goal.target_pose.pose.position.y = 3.8;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 1.0;
      goal.target_pose.pose.orientation.w = 0.0;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved to the finish goal");
        State++;
      }
      else{
        ROS_INFO("The base failed to move forward 1 meter for some reason");
      }

    }
    if (State == 15){
      
     ROS_INFO("Estado 9 : navegacao 7");

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = -75.0;
      goal.target_pose.pose.position.y = 4.0;
      goal.target_pose.pose.position.z = 0.0;
      goal.target_pose.pose.orientation.x = 0.0;
      goal.target_pose.pose.orientation.y = 0.0;
      goal.target_pose.pose.orientation.z = 1.0;
      goal.target_pose.pose.orientation.w = 0.0;

      ROS_INFO("Sending goal");
      ac.sendGoal(goal);

      ac.waitForResult();

      if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
        ROS_INFO("Hooray, the base moved to the finish goal");
        State=20;
      }
      else{
        ROS_INFO("The base failed to move forward 1 meter for some reason");
      }

    }

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