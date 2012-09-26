#include "ros/ros.h"
#include "std_msgs/String.h"
#include "amee/Velocity.h"

void keyHandlerCallback(const amee::Velocity & v)
{
  ROS_INFO("[%0.2f, %0.2f]", v.left, v.right);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "KeyHandlerListener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("wheel_velocities", 1000, keyHandlerCallback);

  ros::spin();

  return 0;
}
