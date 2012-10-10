#include "ros/ros.h"
#include "std_msgs/String.h"
#include "amee/KeyboardCommand.h"
#include "amee/Encoder.h"

void keyHandlerCallback(const amee::Encoder & v)
{
  ROS_INFO("%f:[%li, %li]", v.timestamp, v.left, v.right);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "KeyHandlerListener");

  ros::NodeHandle n;

  ros::Subscriber sub = n.subscribe("/serial/encoder", 1000, keyHandlerCallback);

  ros::spin();

  return 0;
}
