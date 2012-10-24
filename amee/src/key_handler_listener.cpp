#include "key_handler_listener.h"

using namespace amee;

namespace amee{

	void KeyHandlerListener::keyHandlerCallback(const Motor::ConstPtr &motor){
		ROS_INFO("[%0.2f, %0.2f]", motor->left, motor->right);
	}

}//namespace amee


int main(int argc, char **argv){

	KeyHandlerListener khListener;

	ros::init(argc, argv, "KeyHandlerListener");

	ros::NodeHandle n;

	ros::Subscriber sub = n.subscribe("/serial/motor_speed", 1, &KeyHandlerListener::keyHandlerCallback, &khListener);

	ros::spin();

	return 0;
}
