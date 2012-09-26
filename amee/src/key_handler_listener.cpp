#include "ros/ros.h"
#include "std_msgs/String.h"
#include "amee/KeyboardCommand.h"

void keyHandlerCallback(const amee::KeyboardCommand & v)
{
  ROS_INFO("[%0.2f, %0.2f]", v.linear, v.angular);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "KeyHandlerListener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/KeyboardControl/KeyboardCommand", 1000, keyHandlerCallback);

  ros::spin();

  return 0;
}
