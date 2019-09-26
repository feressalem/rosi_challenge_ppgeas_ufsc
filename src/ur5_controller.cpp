
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <rosi_challenge_ppgeas_ufsc/ManipulatorJoints.h>

#include <ros/ros.h>

double cmd[6];
double pos[6];
double vel[6];
double eff[6];
hardware_interface::JointStateInterface jnt_state_interface;
hardware_interface::PositionJointInterface jnt_pos_interface;


class MyRobot : public hardware_interface::RobotHW
{
public:
  MyRobot() 
 { 
   // connect and register the joint state interface
   hardware_interface::JointStateHandle state_handle_a("shoulder_pan_joint", &pos[0], &vel[0], &eff[0]);
   jnt_state_interface.registerHandle(state_handle_a);

   hardware_interface::JointStateHandle state_handle_b("shoulder_lift_joint", &pos[1], &vel[1], &eff[1]);
   jnt_state_interface.registerHandle(state_handle_b);

   hardware_interface::JointStateHandle state_handle_c("elbow_joint", &pos[2], &vel[2], &eff[2]);
   jnt_state_interface.registerHandle(state_handle_c);

   hardware_interface::JointStateHandle state_handle_d("wrist_1_joint", &pos[3], &vel[3], &eff[3]);
   jnt_state_interface.registerHandle(state_handle_d);

   hardware_interface::JointStateHandle state_handle_e("wrist_2_joint", &pos[4], &vel[4], &eff[4]);
   jnt_state_interface.registerHandle(state_handle_e);

   hardware_interface::JointStateHandle state_handle_f("wrist_3_joint", &pos[5], &vel[5], &eff[5]);
   jnt_state_interface.registerHandle(state_handle_f);

   registerInterface(&jnt_state_interface);

   // connect and register the joint position interface
   hardware_interface::JointHandle pos_handle_a(jnt_state_interface.getHandle("shoulder_pan_joint"), &cmd[0]);
   jnt_pos_interface.registerHandle(pos_handle_a);

   hardware_interface::JointHandle pos_handle_b(jnt_state_interface.getHandle("shoulder_lift_joint"), &cmd[1]);
   jnt_pos_interface.registerHandle(pos_handle_b);

   hardware_interface::JointHandle pos_handle_c(jnt_state_interface.getHandle("elbow_joint"), &cmd[2]);
   jnt_pos_interface.registerHandle(pos_handle_c);

   hardware_interface::JointHandle pos_handle_d(jnt_state_interface.getHandle("wrist_1_joint"), &cmd[3]);
   jnt_pos_interface.registerHandle(pos_handle_d);

   hardware_interface::JointHandle pos_handle_e(jnt_state_interface.getHandle("wrist_2_joint"), &cmd[4]);
   jnt_pos_interface.registerHandle(pos_handle_e);

   hardware_interface::JointHandle pos_handle_f(jnt_state_interface.getHandle("wrist_3_joint"), &cmd[5]);
   jnt_pos_interface.registerHandle(pos_handle_f);

   registerInterface(&jnt_pos_interface);
  }

private:

};

void chatterCallback(const rosi_challenge_ppgeas_ufsc::ManipulatorJoints::ConstPtr& msg)
{
  pos[0] = msg->joint_variable[0];
  pos[1] = msg->joint_variable[1];
  pos[2] = msg->joint_variable[2];
  pos[3] = msg->joint_variable[3];
  pos[4] = msg->joint_variable[4];
  pos[5] = msg->joint_variable[5];
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "ur5_controller_node");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe<rosi_challenge_ppgeas_ufsc::ManipulatorJoints>("/ur5/jointsPositionCurrentState",1, chatterCallback);
  ros::Publisher pub = n.advertise<rosi_challenge_ppgeas_ufsc::ManipulatorJoints>("/ur5/jointsPosTargetCommand", 1);
  MyRobot robot;
  //controller_manager::ControllerManager cm(&robot, n);

  ros::Rate loop_rate(3);

  rosi_challenge_ppgeas_ufsc::ManipulatorJoints command;

  while (ros::ok())
  {

   // cm.update(robot.get_time(), robot.get_period());

    command.joint_variable[0] = cmd[0];
    command.joint_variable[1] = cmd[1];
    command.joint_variable[2] = cmd[2];
    command.joint_variable[3] = cmd[3];
    command.joint_variable[4] = cmd[4];
    command.joint_variable[5] = cmd[5];

    
    pub.publish(command);
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}