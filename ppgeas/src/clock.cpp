#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <rosgraph_msgs/Clock.h>

class SubscribeAndPublish
{
public:
  SubscribeAndPublish()
  {
    //Topic you want to publish
    pub_ = n_.advertise<rosgraph_msgs::Clock>("/clock", 1);

    //Topic you want to subscribe
    sub_ = n_.subscribe("/simulation/time", 1, &SubscribeAndPublish::callback, this);
  }

  void callback(const std_msgs::Float32::ConstPtr& msg)
  {
    rosgraph_msgs::Clock sim_time;
    //.... do something with the input and generate the output...
    sim_time.clock = ros::Time(msg->data);
    pub_.publish(sim_time);
  }

private:
  ros::NodeHandle n_; 
  ros::Publisher pub_;
  ros::Subscriber sub_;

};//End of class SubscribeAndPublish

int main(int argc, char **argv)
{
  //Initiate ROS
  ros::init(argc, argv, "odometry_node");

  //Create an object of class SubscribeAndPublish that will take care of everything
  SubscribeAndPublish SAPObject;

  ros::spin();

  return 0;
}

