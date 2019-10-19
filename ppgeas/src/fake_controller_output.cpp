#include <ros/ros.h>
#include <rosi_defy/ManipulatorJoints.h>
#include <sensor_msgs/JointState.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<rosi_defy::ManipulatorJoints>("/ur5/jointsPosTargetCommand", 100);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/move_group/fake_controller_joint_states", 100, &SubscribeAndPublish::callback, this);
  }

  void callback(const sensor_msgs::JointState::ConstPtr& msg)
  {
    rosi_defy::ManipulatorJoints output;
    output.header = msg->header;
    output.joint_variable.push_back(msg->position[0]);
    output.joint_variable.push_back(msg->position[1]);
    output.joint_variable.push_back(msg->position[2]);
    output.joint_variable.push_back(msg->position[3]);
    output.joint_variable.push_back(msg->position[4]);
    output.joint_variable.push_back(msg->position[5]);
    printf("%.5f \n", output.joint_variable[5]);
    pub_.publish(output);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;


};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "fake_controller_output_node");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}