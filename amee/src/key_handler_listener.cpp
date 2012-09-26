#include "ros/ros.h"
#include "std_msgs/String.h"
#include "amee/Velocity2.h"

void keyHandlerCallback(const amee::Velocity2& v)
{
  ROS_INFO("[%0.2f, %0.2f]", v.linear, v.angular);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "KeyHandlerListener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("key_commands", 1000, keyHandlerCallback);

  ros::spin();

  return 0;
}
