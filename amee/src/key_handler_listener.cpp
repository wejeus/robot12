#include "ros/ros.h"
#include "std_msgs/String.h"
#include "amee/Velocity.h"
#include "amee/Motor.h"

using namespace amee;

void keyHandlerCallback(const Motor::ConstPtr &motor)
{
  ROS_INFO("[%0.2f, %0.2f]", motor->left, motor->right);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "KeyHandlerListener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/serial/motor_speed", 1, keyHandlerCallback);

  ros::spin();

  return 0;
}
