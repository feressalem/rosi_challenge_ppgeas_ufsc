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

  ros::Publisher initPosePub_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 2, true);

  ros::Publisher pub_Arm_sp = n.advertise<ppgeas::ArmsSetPoint>("arm_sp", 1, true);

  // ######################################################################################### Renan
  ros::service::waitForService("detect_conveyorbelt");
  ros::ServiceClient cb_detection = n.serviceClient<ppgeas::DetectConveyorBelt>("detect_conveyorbelt");
  ppgeas::DetectConveyorBelt srv_detection;

  ros::service::waitForService("detect_conveyorbelt");
  ros::ServiceClient client_touch = n.serviceClient<ppgeas::Touch>("arm_touch");
  ppgeas::Touch srv_touch;
  
  // ######################################################################################### Renan

  // ######################################################################################### Danilo



  // ######################################################################################### Renan

  tf2_ros::Buffer tfBuffer;
  tf2_ros::TransformListener tfListener(tfBuffer);


  geometry_msgs::TransformStamped transformStamped;
  geometry_msgs::PoseWithCovarianceStamped initPose_;

  move_base_msgs::MoveBaseGoal goal;

  ppgeas::ArmsSetPoint arm_sp_msg;

  std_srvs::Empty c;
  while (ros::ok())
    {
    if (State == 0){
        try{
        transformStamped = tfBuffer.lookupTransform("map", "static_rosiInitialPose",
                                 ros::Time(0));
        }
        catch (tf2::TransformException &ex) {
          ROS_WARN("%s",ex.what());
          ros::Duration(1.0).sleep();
          continue;
        }
        initPose_.header.stamp = ros::Time::now();
        initPose_.header.frame_id = "map";
      //position
        initPose_.pose.pose.position.x = transformStamped.transform.translation.x;
        initPose_.pose.pose.position.y = transformStamped.transform.translation.y;
      //angle
        initPose_.pose.pose.orientation.x = transformStamped.transform.rotation.x;
        initPose_.pose.pose.orientation.y = transformStamped.transform.rotation.y;
        initPose_.pose.pose.orientation.z = transformStamped.transform.rotation.z;
        initPose_.pose.pose.orientation.w = transformStamped.transform.rotation.w;
      //publish msg
        initPosePub_.publish(initPose_);
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
            State++; // RESETAR PRA State++ QUANDO TUDO ESTIVER PRONTO

        } else {
            ROS_ERROR("Failed to call costmap service");
            return 1;
        }
    }

    if (State == 2000){
    	ROS_INFO("Estado 2 : Posicionando o end_effector");

        if (client_touch.call(c))
        {
            ROS_INFO("Posicionando o end_effector");
            State++; // RESETAR PRA State++ QUANDO TUDO ESTIVER PRONTO

        } else {
            ROS_ERROR("Failed to call costmap service");
            return 1;
        }
    }

    if (State == 2){
      ROS_INFO("Estado 2 : navegacao ate a posicao de tocar o primeiro rolo");

      goal.target_pose.header.frame_id = "map";
      goal.target_pose.header.stamp = ros::Time::now();

      goal.target_pose.pose.position.x = -5.1;
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
      ROS_INFO("Estado 3 : Tocar o rolo");

    //   static const std::string PLANNING_GROUP = "manipulator";

    // // The :move_group_interface:`MoveGroupInterface` class can be easily
    // // setup using just the name of the planning group you would like to control and plan for.
    //   moveit::planning_interface::MoveGroupInterface move_group(PLANNING_GROUP);
  
    // // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // // class to add and remove collision objects in our "virtual world" scene
    //   moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    //  // Raw pointers are frequently used to refer to the planning group for improved performance.
    //   //const robot_state::JointModelGroup* joint_model_group =
    //  // move_group.getCurrentState()->getJointModelGroup(PLANNING_GROUP);
    //   moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    //   double cx;
    //   double cy;
    //   int windows_size_x = 640;
    //   int windows_size_y = 480;

    //     try{
    //     transformStamped = tfBuffer.lookupTransform("base_link", "tool_pointer",
    //                              ros::Time(0));
    //     }
    //     catch (tf2::TransformException &ex) {
    //       ROS_WARN("%s",ex.what());
    //       ros::Duration(1.0).sleep();
    //       continue;
    //     }
      
    //   //call conveyor belt service
    //     if (cb_detection.call(srv_detection))
    //     {
    //         ROS_INFO("centroid: %.4f, %.4f", srv_detection.response.ctdx, srv_detection.response.ctdy );
    //         State = 3; //MUDEI de volta
    //         cx = srv_detection.response.ctdx;
    //         cy = srv_detection.response.ctdy;
    //     } else {
    //         ROS_ERROR("Failed to call service detect_conveyorbelt");
    //         return 1;
    //     }
    //     if (abs(cy - windows_size_y/2) < 5)
    //     {
    //       if(cy - windows_size_y/2 > 0)
    //       {
    //       geometry_msgs::Pose target_orientation;
    //       target_orientation.orientation.x = transformStamped.transform.rotation.x;
    //       target_orientation.orientation.y = transformStamped.transform.rotation.y;
    //       target_orientation.orientation.z = transformStamped.transform.rotation.z;
    //       target_orientation.orientation.w = transformStamped.transform.rotation.w;
    //       target_orientation.position.x = transformStamped.transform.translation.x;
    //       target_orientation.position.y = transformStamped.transform.translation.y;
    //       target_orientation.position.z = transformStamped.transform.translation.z - 0.2;
    //       move_group.setPoseTarget(target_orientation);
    //       move_group.setPlanningTime(5.0);

    //   bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //   ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

    //   move_group.move(); 
    //     } else {
    //       geometry_msgs::Pose target_orientation;
    //       target_orientation.orientation.x = transformStamped.transform.rotation.x;
    //       target_orientation.orientation.y = transformStamped.transform.rotation.y;
    //       target_orientation.orientation.z = transformStamped.transform.rotation.z;
    //       target_orientation.orientation.w = transformStamped.transform.rotation.w;
    //       target_orientation.position.x = transformStamped.transform.translation.x;
    //       target_orientation.position.y = transformStamped.transform.translation.y;
    //       target_orientation.position.z = transformStamped.transform.translation.z  - 0.2;
    //       move_group.setPoseTarget(target_orientation);
    //       move_group.setPlanningTime(5.0);

    //   bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    //   ROS_INFO_NAMED("tutorial", "Visualizing plan 3 (constraints) %s", success ? "" : "FAILED");

    //   move_group.move(); 
    //     }
    //     }

    //   //while(abs(srv_detection.response.cx - windows_size_x/2) < 5 && abs(srv_detection.response.cy - window_size_y/2) < 5){
    //   //  if(cb_detection.call(srv_detection)){
    //   //    ROS_INFO("Centroide x = %f", srv_detection.response.cx);
    //   //    ROS_INFO("Centroide x = %f", srv_detection.response.cy);
    //   //    cx = srv_detection.response.cx;
    //   //    cy = srv_detection.response.cy;
    //   //  } else{
    //   //    ROS_ERROR("Failed to call service cb_detection")
    //   //  }
    //   //  if(arm_center.call(srv_planning)){
    //   //    //juntas = srv_planning.response;
    //   //  }else{
    //   //    ROS_ERROR("Failed to call service arm_center")
    //   //  }
    //   //}
      State++;
    }

    if (State == 4){

      ROS_INFO("Estado 4 : navegacao ate embaixo da escada da esquerda");

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

    if (State == 5){
    	ROS_INFO("Estado 5: Detectar fogo do lado direito em baixo");
    	State++;
    }

    if (State == 6){

     	ROS_INFO("Estado 6 : Inicio da subida da escada da esquerda");
     	
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

    if (State == 7){
    	ROS_INFO("Estado 7: Limpando costmap");
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

    if (State == 8){

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
        
        if(transformStamped.transform.translation.x < -42.25){
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

	if (State == 9){

     	// ROS_INFO("Estado 9 : Mexendo as esteiras pra conseguir subir");
     	
		arm_sp_msg.front_dir = 1;
		arm_sp_msg.rear_dir = 0;
		arm_sp_msg.front_arms_sp = 3;
		arm_sp_msg.rear_arms_sp = 5.8;
		
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

        if(transformStamped.transform.translation.x < -42.55){
        	State++;
        }
    }

    if (State == 10){

     	// ROS_INFO("Estado 10 : Mexendo as esteiras pra conseguir subir");
     	
		arm_sp_msg.front_dir = 0;
		arm_sp_msg.rear_dir = 0;
		arm_sp_msg.front_arms_sp = 3.7;
		arm_sp_msg.rear_arms_sp = 5.8;
		
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
	if (State == 11){

     	// ROS_INFO("Estado 11 : Voltando esteiras a posicao de navegacao");
     	
		arm_sp_msg.front_dir = 1;
		arm_sp_msg.rear_dir = 1;
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


	if (State == 12){

		// ROS_INFO("Estado 12 : navegacao até o final da passarela");

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
			ROS_INFO("Chegou no final da passarela da direita");
			State++;
		}
		else{
			ROS_INFO("The base failed to move forward 1 meter for some reason");
		}

    }

    if (State == 13){
    	ROS_INFO("Estado 13: Detectar fogo do lado direito sobre a passarela");
    	State++;
    }

    if (State == 14){

		// ROS_INFO("Estado 14 : navegacao de re ate o inicio da passarela");

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
			ROS_INFO("Voltou para o inicio da passarela");
			State++;
		}
		else{
			ROS_INFO("The base failed to move forward 1 meter for some reason");
		}

    }

    if (State == 15){

		// ROS_INFO("Estado 15 : navegacao até embaixo da passarela");

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
		
		// ac.waitForResult();
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
		// if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			// ROS_INFO("Hooray, the base moved to the finish goal");
			State++;
		// }
		// else{
			// ROS_INFO("The base failed to move forward 1 meter for some reason");
		// }
    }

    if (State == 15){ // Começa a descida

     	// ROS_INFO("Estado 15 : Mexendo as esteiras pra comecar a descida");
     	
		arm_sp_msg.front_dir = 0;
		arm_sp_msg.rear_dir = 0;
		arm_sp_msg.front_arms_sp = 3.7;
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

        printf("%.2f\n", transformStamped.transform.translation.x);

        if(transformStamped.transform.translation.x > -42.65){
        	State++;
        }
    }

    if (State == 16){ // Começa a descida

     	// ROS_INFO("Estado 16 : Mexendo as esteiras pra conseguir descer");
     	
		arm_sp_msg.front_dir = 0;
		arm_sp_msg.rear_dir = 1;
		arm_sp_msg.front_arms_sp = 3.3;
		arm_sp_msg.rear_arms_sp = 1;
		
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

        if(transformStamped.transform.translation.x > -41.5){
        	State++;
        }
    }

    if (State == 17){ // Começa a descida

     	// ROS_INFO("Estado 17 : Voltando as esteiras para posicao de navegacao");
     	
		arm_sp_msg.front_dir = 1;
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

        if(transformStamped.transform.translation.x > -41){
        	State++;
        }
    }

    if (State == 18){
    	ROS_INFO("State 18 : Vai ate o final do mapa, pela direita");
    	goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -54;
		goal.target_pose.pose.position.y = 4;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = 1.0;
		goal.target_pose.pose.orientation.w = 0.0;

		// ROS_INFO("Sending goal");
		ac.sendGoal(goal);
		
		// ac.waitForResult();
		// ROS_INFO("oi");

		printf("%.2f\n", transformStamped.transform.translation.x);
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
        
        if(transformStamped.transform.translation.x < -52){
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

    if (State == 19){
    	goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -54;
		goal.target_pose.pose.position.y = 3.5;
		goal.target_pose.pose.position.z = 0.0;
		goal.target_pose.pose.orientation.x = 0.0;
		goal.target_pose.pose.orientation.y = 0.0;
		goal.target_pose.pose.orientation.z = -0.71;
		goal.target_pose.pose.orientation.w = 0.71;

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
        
  //       if(transformStamped.transform.translation.x > -42.25){
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

    if (State == 20){
    	goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -54;
		goal.target_pose.pose.position.y = -3.5;
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
        
  //       if(transformStamped.transform.translation.x > -42.25){
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

    if (State == 21){
    	goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -45;
		goal.target_pose.pose.position.y = -4;
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
        
  //       if(transformStamped.transform.translation.x > -42.25){
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


    if (State == 22){
    	goal.target_pose.header.frame_id = "map";
		goal.target_pose.header.stamp = ros::Time::now();

		goal.target_pose.pose.position.x = -40;
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
        
  //       if(transformStamped.transform.translation.x > -42.25){
  //       	State++;
  //       }
		if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			ROS_INFO("Hooray, the base moved to the finish goal");
			State=200;
		}
		else{
			ROS_INFO("The base failed to move forward 1 meter for some reason");
		}
    }
/*----------------------------------------------------------------------------------------------------------------*/
    // Subida da escada do lado esquerdo

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
        
        if(transformStamped.transform.translation.x > -42.25){
        	State++;
        }
		// if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED){
			// ROS_INFO("Hooray, the base moved to the finish goal");
			// State++;
		// }
		// else{
			// ROS_INFO("The base failed to move forward 1 meter for some reason");
		// }
    }

    if (State == 53){ // Começa a descida

     	// ROS_INFO("Estado 34 : Mexendo as esteiras pra conseguir subir");
     	
		arm_sp_msg.front_dir = 0;
		arm_sp_msg.rear_dir = 1;
		arm_sp_msg.front_arms_sp = 0.5;
		arm_sp_msg.rear_arms_sp = 1;
		
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

        if(transformStamped.transform.translation.x > -41){
        	State++;
        }
    }






    if (State == 100){
    	ROS_INFO("Estado 36: Voltando o costmap");
    	dynamic_reconfigure::ReconfigureRequest srv_req;
     	dynamic_reconfigure::ReconfigureResponse srv_resp;
     	dynamic_reconfigure::BoolParameter enable_param;
     	dynamic_reconfigure::Config conf;

     	enable_param.name = "enabled";
     	enable_param.value = true;
     	conf.bools.push_back(enable_param);
     	srv_req.config = conf;

     	if(ros::service::call("/move_base/local_costmap/obstacle_layer/set_parameters", srv_req, srv_resp)){
     		ROS_INFO("COM COSTMAP!!!");
     	} else {
     		ROS_INFO("ACHOU ERRADO!!!");
     	}
    	State=100;
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
