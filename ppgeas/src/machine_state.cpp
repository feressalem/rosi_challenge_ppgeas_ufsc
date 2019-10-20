#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <rosi_defy/RosiMovementArray.h>
#include <rosi_defy/RosiMovement.h>
#include <tf2_ros/transform_listener.h>
#include <tf/transform_datatypes.h>
#include <geometry_msgs/TransformStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ppgeas/DetectConveyorBelt.h>

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
#include <ppgeas/ArmsSetPoint.h>
#include <ppgeas/Touch.h>

#include <math.h>
#include <cmath>
#ifndef M_PI
#define M_PI (3.14159265358979323846)
#endif

#include <dynamic_reconfigure/Reconfigure.h>
#include <dynamic_reconfigure/BoolParameter.h>
#include <dynamic_reconfigure/IntParameter.h>
#include <dynamic_reconfigure/Config.h>


float array_arm_pos[4] = {};
typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
int State = 0;  //inicial state

void chatterCallback1(const rosi_defy::RosiMovementArray::ConstPtr& msg)
{
  // ::array_arm_pos[0]= msg->movement_array[2].joint_var;
  // ::array_arm_pos[1]= msg->movement_array[1].joint_var;
  //ROS_INFO(array_arm_pos[0]);

  	// saving_arms_positions (feedback error)
	if (msg->movement_array[0].joint_var < 0){
		::array_arm_pos[0] = 2 * M_PI + msg->movement_array[0].joint_var;
	} else {
		::array_arm_pos[0] = msg->movement_array[0].joint_var;
	}
	if (msg->movement_array[2].joint_var > 0){
		::array_arm_pos[2] = -2*M_PI + msg->movement_array[2].joint_var;
  	} else {
		::array_arm_pos[2] = msg->movement_array[2].joint_var;
	}
	if (msg->movement_array[1].joint_var > 0){
		::array_arm_pos[1] = -2*M_PI + msg->movement_array[1].joint_var;
	} else {
		::array_arm_pos[1] = msg->movement_array[1].joint_var;
	}
	if (msg->movement_array[3].joint_var < 0){
		::array_arm_pos[3] =  2*M_PI + msg->movement_array[3].joint_var;
	} else {
		::array_arm_pos[3] = msg->movement_array[3].joint_var;
	}
}

int main(int argc, char** argv){
  ros::init(argc, argv, "machine_state");
  ros::NodeHandle n;
  ros::AsyncSpinner spinner(1);
  spinner.start();
  //ros::Rate loop_rate(10);
  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  //wait for the action server to come up
  while(!ac.waitForServer(ros::Duration(5.0))){
    ROS_INFO("Waiting for the move_base action server to come up");
  }
  ros::Subscriber sub = n.subscribe<rosi_defy::RosiMovementArray>("/rosi/arms_joints_position", 1, chatterCallback1);


  ros::service::waitForService("/move_base/clear_costmaps");
  ros::ServiceClient spawner3 = n.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");

  ros::Publisher pub_Arm_sp = n.advertise<ppgeas::ArmsSetPoint>("arm_sp", 1, true);
  
  // ######################################################################################### Renan
  ros::service::waitForService("detect_conveyorbelt");
  ros::ServiceClient cb_detection = n.serviceClient<ppgeas::DetectConveyorBelt>("detect_conveyorbelt");
  ppgeas::DetectConveyorBelt srv_detection;
  // ######################################################################################### Renan

  // ######################################################################################### Renan
  ros::service::waitForService("touch");
  ros::ServiceClient arm = n.serviceClient<ppgeas::Touch>("touch");
  ppgeas::Touch srv_planning;
  // ######################################################################################### Renan

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);


  geometry_msgs::TransformStamped transformStamped;

  move_base_msgs::MoveBaseGoal goal;

  ppgeas::ArmsSetPoint arm_sp_msg;

  std_srvs::Empty c;
  while (ros::ok())
    {
	try{
    transformStamped = tfBuffer.lookupTransform("odom", "base_link2",
                             ros::Time(0));
    }
    catch (tf2::TransformException &ex) {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
      continue;
    }
    if (State == 0){

      if (abs(::array_arm_pos[0]) > 2.5){
        ROS_INFO("mudando estado");
        State++;
      }
    }
    if (State == 1){

        ROS_INFO("Estado 1 : resetando mapa e odometria");

        if (spawner3.call(c))
        {
            ROS_INFO("Limpando costmaps");
            // State = 4; //MUDEI de volta

        } else {
            ROS_ERROR("Failed to call costmap service");
            return 1;
        }
        srv_planning.request.messages = "giro1";
        if (arm.call(srv_planning)){
        	ROS_INFO("Girando o braço");
            State = 12; //MUDEI de volta
        } else {
            ROS_ERROR("Failed to call costmap service");
            return 1;
        }
    }

	if (State == 2){
		ROS_INFO("Estado 2 : navegacao ate esquerda");

		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -5.3;
		goal.target_pose.pose.position.y = -2.07;
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

	if (State == 3){

		ROS_INFO("Estado 3 : navegacao ate embaixo da escada da esquerda");

		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -41.0;
		goal.target_pose.pose.position.y = -2.02;
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
		State++; // Só vai para o próximo estado que fica chamando o serviço de detectar fogo
		}
		else{
		ROS_INFO("The base failed to move forward 1 meter for some reason");
		}
	}

    if (State == 4){
     	ROS_INFO("Estado 4 : Inicio da subida da escada da esquerda");
     	
		arm_sp_msg.front_dir = 0;
		arm_sp_msg.rear_dir = 0;
		arm_sp_msg.front_arms_sp = 3.5;
		arm_sp_msg.rear_arms_sp = 0.1;
		
		pub_Arm_sp.publish(arm_sp_msg);
		printf("%.2f\n", (abs(::array_arm_pos[0])));
		if ((abs(::array_arm_pos[0]) > 3.0)){
			State++;
		}
    }

    if (State == 5){
    	ROS_INFO("Estado 5: Limpando costmap");
    	dynamic_reconfigure::ReconfigureRequest srv_req;
     	dynamic_reconfigure::ReconfigureResponse srv_resp;
     	// dynamic_reconfigure::IntParameter orientation_mode_param;
     	dynamic_reconfigure::BoolParameter obstacle_param;
     	// dynamic_reconfigure::BoolParameter ow_orientation_param;
     	dynamic_reconfigure::Config conf;

     	obstacle_param.name = "enabled";
     	obstacle_param.value = false;
     	conf.bools.push_back(obstacle_param);
     	srv_req.config = conf;

     	if(ros::service::call("/move_base/local_costmap/obstacle_layer/set_parameters", srv_req, srv_resp)){
     		ROS_INFO("SEM COSTMAP!!!");
     	} else {
     		ROS_INFO("ACHOU ERRADO!!!");
     	}
     	// ow_orientation_param.name = "global_plan_overwrite_orientation";
     	// ow_orientation_param.value = false;
     	// conf.bools.push_back(ow_orientation_param);
     	// srv_req.config = conf;

     	// if(ros::service::call("/move_base/TebLocalPlannerROS/set_parameters", srv_req, srv_resp)){
     	// 	ROS_INFO("SEM ORIENTATION OW!!!");
     	// } else {
     	// 	ROS_INFO("ACHOU ERRADO OW!!!");
     	// }

     	// orientation_mode_param.name = "orientation_mode";
     	// orientation_mode_param.value = 2;
     	// conf.ints.push_back(orientation_mode_param);
     	// srv_req.config = conf;

     	// if(ros::service::call("/move_base/GlobalPlanner/set_parameters", srv_req, srv_resp)){
     	// 	ROS_INFO("OREINTATION MODE INTERPOLATE!!!");
     	// } else {
     	// 	ROS_INFO("INTERPOLATE DEU RUIM!!!");
     	// }
    	State++;
    }

    if (State == 6){

 		// ROS_INFO("Estado 8 : navegacao até o final da passarela");

		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -51.25;
		goal.target_pose.pose.position.y = -2.02;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 1.0;
		goal.target_pose.pose.orientation.w = 0.0;

		// ROS_INFO("Sending goal");
		ac.sendGoal(goal);
		// ac.waitForResult();
		// ROS_INFO("oi");

		// printf("%.2f\n", transformStamped.transform.translation.x);
		printf("%.2f\n", transformStamped.transform.translation.x);
        
        if(transformStamped.transform.translation.x < -42.2){
        	State++;
        }
		// if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		// 	ROS_INFO("Hooray, the base moved to the finish goal");
		// 	State++;
		// }
		// else{
		// 	ROS_INFO("The base failed to move forward 1 meter for some reason");
		// }

    }

	if (State == 7){

     	// ROS_INFO("Estado 7 : Mexendo as esteiras pra conseguir subir");
     	
		arm_sp_msg.front_dir = 1;
		arm_sp_msg.rear_dir = 0;
		arm_sp_msg.front_arms_sp = 2.7;
		arm_sp_msg.rear_arms_sp = 5.5;
		
		pub_Arm_sp.publish(arm_sp_msg);
		
		printf("%.2f\n", transformStamped.transform.translation.x);

        if(transformStamped.transform.translation.x < -42.55){
        	State++;
        }
    }

	if (State == 8){

     	// ROS_INFO("Estado 8 : Mexendo as esteiras pra conseguir subir");
     	
		arm_sp_msg.front_dir = 0;
		arm_sp_msg.rear_dir = 0;
		arm_sp_msg.front_arms_sp = 3.7;
		arm_sp_msg.rear_arms_sp = 5.8;
		
		pub_Arm_sp.publish(arm_sp_msg);
		
        printf("%.2f\n", transformStamped.transform.translation.x);

        if(transformStamped.transform.translation.x < -42.7){
        	State++;
        }
    }

	if (State == 9){

     	// ROS_INFO("Estado 9 : Voltando esteiras a posicao de navegacao");
     	
		arm_sp_msg.front_dir = 1;
		arm_sp_msg.rear_dir = 1;
		arm_sp_msg.front_arms_sp = 2.7;
		arm_sp_msg.rear_arms_sp = 0.5;
		
		pub_Arm_sp.publish(arm_sp_msg);
	
	    printf("%.2f\n", transformStamped.transform.translation.x);

        if(transformStamped.transform.translation.x < -43.20){
        	State++;
        }
    }

	if (State == 10){

		// ROS_INFO("Estado 10 : navegacao até o final da passarela");

		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -51.25;
		goal.target_pose.pose.position.y = -2.02;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 1.0;
		goal.target_pose.pose.orientation.w = 0.0;

		// ROS_INFO("Sending goal");
		ac.sendGoal(goal);
		
		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("Chegou no final da passarela da esquerda");
			State++;
		}
		else{
			ROS_INFO("The base failed to move forward 1 meter for some reason");
		}

    }


	if (State == 11){

		// ROS_INFO("Estado 11 : navegacao de re ate o inicio da passarela");

		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -43;
		goal.target_pose.pose.position.y = -2.02;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 1.0;
		goal.target_pose.pose.orientation.w = 0.0;

		// ROS_INFO("Sending goal");
		ac.sendGoal(goal);
		
		ac.waitForResult();

		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("Voltou para o inicio da passarela da esquerda");
			State++;
		}
		else{
			ROS_INFO("The base failed to move forward 1 meter for some reason");
		}
    }


    if (State == 12){

		// ROS_INFO("Estado 12 : navegacao até embaixo da passarela");

		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -41;
		goal.target_pose.pose.position.y = -2.02;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 1.0;
		goal.target_pose.pose.orientation.w = 0.0;

		// ROS_INFO("Sending goal");
		ac.sendGoal(goal);
		State++;
    }

    if (State == 13){ 

     	// ROS_INFO("Estado 13 : Mexendo as esteiras pra conseguir descer");
     	
		arm_sp_msg.front_dir = 0;
		arm_sp_msg.rear_dir = 0;
		arm_sp_msg.front_arms_sp = 3.3;
		arm_sp_msg.rear_arms_sp = 5.8;
		
		pub_Arm_sp.publish(arm_sp_msg);

        printf("%.2f\n", transformStamped.transform.translation.x);

        if(transformStamped.transform.translation.x > -41.5){
        	State++;
        }
    }

	if (State == 14){

     	// ROS_INFO("Estado 14 : Voltando as esteiras para posicao de navegacao");
     	
		arm_sp_msg.front_dir = 1;
		arm_sp_msg.rear_dir = 1;
		arm_sp_msg.front_arms_sp = 2.7;
		arm_sp_msg.rear_arms_sp = 0.5;
		
		pub_Arm_sp.publish(arm_sp_msg);
	
        printf("%.2f\n", transformStamped.transform.translation.x);

        if(transformStamped.transform.translation.x > -41){
        	State=500;
        }
    }



    if (State == 42){

     	ROS_INFO("Estado 42 : Subindo escada 2");
     	
		arm_sp_msg.front_dir = 1;
		arm_sp_msg.rear_dir = 1;
		arm_sp_msg.front_arms_sp = 0.1;
		arm_sp_msg.rear_arms_sp = 3.5;
		
		pub_Arm_sp.publish(arm_sp_msg);
		printf("%.2f\n", (abs(::array_arm_pos[0])));
		if ((abs(::array_arm_pos[1]) > 3.0)){
			State++;
		}
    }

    if (State == 43){
    	ROS_INFO("Estado 31: Limpando costmap");
    	dynamic_reconfigure::ReconfigureRequest srv_req;
     	dynamic_reconfigure::ReconfigureResponse srv_resp;
     	dynamic_reconfigure::IntParameter orientation_mode_param;
     	dynamic_reconfigure::BoolParameter obstacle_param;
     	dynamic_reconfigure::BoolParameter ow_orientation_param;
     	dynamic_reconfigure::Config conf;

     	obstacle_param.name = "enabled";
     	obstacle_param.value = false;
     	conf.bools.push_back(obstacle_param);
     	srv_req.config = conf;

     	if(ros::service::call("/move_base/local_costmap/obstacle_layer/set_parameters", srv_req, srv_resp)){
     		ROS_INFO("SEM COSTMAP!!!");
     	} else {
     		ROS_INFO("ACHOU ERRADO!!!");
     	}
     	ow_orientation_param.name = "global_plan_overwrite_orientation";
     	ow_orientation_param.value = false;
     	conf.bools.push_back(ow_orientation_param);
     	srv_req.config = conf;

     	if(ros::service::call("/move_base/TebLocalPlannerROS/set_parameters", srv_req, srv_resp)){
     		ROS_INFO("SEM ORIENTATION OW!!!");
     	} else {
     		ROS_INFO("ACHOU ERRADO OW!!!");
     	}

     	orientation_mode_param.name = "orientation_mode";
     	orientation_mode_param.value = 2;
     	conf.ints.push_back(orientation_mode_param);
     	srv_req.config = conf;

     	if(ros::service::call("/move_base/GlobalPlanner/set_parameters", srv_req, srv_resp)){
     		ROS_INFO("OREINTATION MODE INTERPOLATE!!!");
     	} else {
     		ROS_INFO("INTERPOLATE DEU RUIM!!!");
     	}
    	State++;
    }

    if (State == 44){

		// ROS_INFO("Estado 32 : navegacao até o final da passarela");

		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -51.25;
		goal.target_pose.pose.position.y = -2.02;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 0.0;
		goal.target_pose.pose.orientation.w = 1.0;

		// ROS_INFO("Sending goal");
		ac.sendGoal(goal);
		// ac.waitForResult();
		// ROS_INFO("oi");

		// printf("%.2f\n", transformStamped.transform.translation.x);
		try{
        	transformStamped = tfBuffer.lookupTransform("odom", "base_link",
                                ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			//continue;
        }

        printf("%.2f\n", transformStamped.transform.translation.x);
        
        if(transformStamped.transform.translation.x < -42.0){
        	State++;
        }
		// if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
		// 	ROS_INFO("Hooray, the base moved to the finish goal");
		// 	State++;
		// }
		// else{
		// 	ROS_INFO("The base failed to move forward 1 meter for some reason");
		// }

    }

	if (State == 45){

     	// ROS_INFO("Estado 33 : Mexendo as esteiras pra conseguir subir");
     	
		arm_sp_msg.front_dir = 0;
		arm_sp_msg.rear_dir = 1;
		arm_sp_msg.front_arms_sp = 1.5;
		arm_sp_msg.rear_arms_sp = 3.5;
		
		pub_Arm_sp.publish(arm_sp_msg);
		try{
        	transformStamped = tfBuffer.lookupTransform("odom", "base_link",
                                ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
        }

        printf("%.2f\n", transformStamped.transform.translation.x);

        if(transformStamped.transform.translation.x < -42.2){
        	State++;
        }
    }

    if (State == 46){

     	// ROS_INFO("Estado 33 : Mexendo as esteiras pra conseguir subir");
     	
		arm_sp_msg.front_dir = 1;
		arm_sp_msg.rear_dir = 0;
		arm_sp_msg.front_arms_sp = 5.7;
		arm_sp_msg.rear_arms_sp = 3;
		
		pub_Arm_sp.publish(arm_sp_msg);
		try{
        	transformStamped = tfBuffer.lookupTransform("odom", "base_link",
                                ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
        }

        printf("%.2f\n", transformStamped.transform.translation.x);

        if(transformStamped.transform.translation.x < -42.63){
        	State++;
        }
    }

    if (State == 47){

     	// ROS_INFO("Estado 34 : Mexendo as esteiras pra conseguir subir");
     	
		arm_sp_msg.front_dir = 0;
		arm_sp_msg.rear_dir = 1;
		arm_sp_msg.front_arms_sp = 5.8;
		arm_sp_msg.rear_arms_sp = 3.7;
		
		pub_Arm_sp.publish(arm_sp_msg);
		
		try{
        	transformStamped = tfBuffer.lookupTransform("odom", "base_link",
                                ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
        }

        printf("%.2f\n", transformStamped.transform.translation.x);

        if(transformStamped.transform.translation.x < -42.70){
        	State++;
        }
    }
	if (State == 48){

     	// ROS_INFO("Estado 34 : Mexendo as esteiras pra conseguir subir");
     	
		arm_sp_msg.front_dir = 0;
		arm_sp_msg.rear_dir = 0;
		arm_sp_msg.front_arms_sp = 2.7;
		arm_sp_msg.rear_arms_sp = 0.5;
		
		pub_Arm_sp.publish(arm_sp_msg);
		try{
        	transformStamped = tfBuffer.lookupTransform("odom", "base_link",
                                ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
        }

        printf("%.2f\n", transformStamped.transform.translation.x);

        if(transformStamped.transform.translation.x < -43.20){
        	State++;
        }
    }
			// try{
	  //       	transformStamped = tfBuffer.lookupTransform("odom", "base_link",
	  //                               ros::Time(0));
	  //       }
	  //       catch (tf2::TransformException &ex) {
			// 	ROS_WARN("%s",ex.what());
			// 	ros::Duration(1.0).sleep();
			// 	continue;
	  //       }

	  //       if(transformStamped.transform.translation.x < -42.35){
	  //       	State=34;
	  //       }


	if (State == 49){

		// ROS_INFO("Estado 32 : navegacao até o final da passarela");

		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -51.25;
		goal.target_pose.pose.position.y = -2.02;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 0.0;
		goal.target_pose.pose.orientation.w = 1.0;

		// ROS_INFO("Sending goal");
		ac.sendGoal(goal);
		
		ac.waitForResult();
		// ROS_INFO("oi");

		// printf("%.2f\n", transformStamped.transform.translation.x);
		// try{
  //       	transformStamped = tfBuffer.lookupTransform("odom", "base_link",
  //                               ros::Time(0));
  //       }
  //       catch (tf2::TransformException &ex) {
		// 	ROS_WARN("%s",ex.what());
		// 	ros::Duration(1.0).sleep();
		// 	//continue;
  //       }

  //       printf("%.2f\n", transformStamped.transform.translation.x);
        
  //       if(transformStamped.transform.translation.x < -42.25){
  //       	State++;
  //       }
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("Hooray, the base moved to the finish goal");
			State++;
		}
		else{
			ROS_INFO("The base failed to move forward 1 meter for some reason");
		}

    }

    if (State == 50){

		// ROS_INFO("Estado 32 : navegacao até o final da passarela");

		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -43;
		goal.target_pose.pose.position.y = -2.02;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 0.0;
		goal.target_pose.pose.orientation.w = 1.0;

		// ROS_INFO("Sending goal");
		ac.sendGoal(goal);
		
		ac.waitForResult();
		// ROS_INFO("oi");

		// printf("%.2f\n", transformStamped.transform.translation.x);
		// try{
  //       	transformStamped = tfBuffer.lookupTransform("odom", "base_link",
  //                               ros::Time(0));
  //       }
  //       catch (tf2::TransformException &ex) {
		// 	ROS_WARN("%s",ex.what());
		// 	ros::Duration(1.0).sleep();
		// 	//continue;
  //       }

  //       printf("%.2f\n", transformStamped.transform.translation.x);
        
  //       if(transformStamped.transform.translation.x < -42.25){
  //       	State++;
  //       }
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("Hooray, the base moved to the finish goal");
			State++;
		}
		else{
			ROS_INFO("The base failed to move forward 1 meter for some reason");
		}

    }

	if (State == 51){ // Começa a descida

     	// ROS_INFO("Estado 34 : Mexendo as esteiras pra conseguir subir");
     	
		arm_sp_msg.front_dir = 1;
		arm_sp_msg.rear_dir = 0;
		arm_sp_msg.front_arms_sp = 5.7;
		arm_sp_msg.rear_arms_sp = 5.7;
		
		pub_Arm_sp.publish(arm_sp_msg);
		
		try{
        	transformStamped = tfBuffer.lookupTransform("odom", "base_link",
                                ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
			ROS_WARN("%s",ex.what());
			ros::Duration(1.0).sleep();
			continue;
        }

        // printf("%.2f\n", transformStamped.transform.translation.x);

        // if(transformStamped.transform.translation.x > -42.25){
        printf("%.2f\n", (abs(::array_arm_pos[0])));
		if ((abs(::array_arm_pos[0]) > 3.0)){	
        	State++;
        }
    }

    if (State == 52){

		// ROS_INFO("Estado 32 : navegacao até o final da passarela");

		goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -41;
		goal.target_pose.pose.position.y = -2.02;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 0.0;
		goal.target_pose.pose.orientation.w = 1.0;

		// ROS_INFO("Sending goal");
		ac.sendGoal(goal);
		
		// ac.waitForResult();
		// ROS_INFO("oi");

        printf("%.2f\n", transformStamped.transform.translation.x);
        
        if(transformStamped.transform.translation.x > -42.25){
        	State++;
        }
    }

    if (State == 53){ // Começa a descida

     	// ROS_INFO("Estado 34 : Mexendo as esteiras pra conseguir subir");
     	
		arm_sp_msg.front_dir = 0;
		arm_sp_msg.rear_dir = 1;
		arm_sp_msg.front_arms_sp = 0.5;
		arm_sp_msg.rear_arms_sp = 1;
		
		pub_Arm_sp.publish(arm_sp_msg);

        printf("%.2f\n", transformStamped.transform.translation.x);

        if(transformStamped.transform.translation.x > -41.5){
        	State++;
        }
    }

    if (State == 54){ // Começa a descida

     	// ROS_INFO("Estado 34 : Mexendo as esteiras pra conseguir subir");
     	
		arm_sp_msg.front_dir = 0;
		arm_sp_msg.rear_dir = 0;
		arm_sp_msg.front_arms_sp = 2.7;
		arm_sp_msg.rear_arms_sp = 0.5;
		
		pub_Arm_sp.publish(arm_sp_msg);

        printf("%.2f\n", transformStamped.transform.translation.x);

        if(transformStamped.transform.translation.x > -41){
        	State=500;
        }
    }




   // ros::spinOnce();

    //loop_rate.sleep();
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
