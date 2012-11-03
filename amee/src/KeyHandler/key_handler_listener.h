#ifndef KEY_HANDLER_LISTENER_H
#define KEY_HANDLER_LISTENER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "amee/Velocity.h"
#include "amee/Motor.h"

using namespace amee;

namespace amee{

	class KeyHandlerListener{
		public:
			void keyHandlerCallback(const Motor::ConstPtr &);
	};

};//namespace amee

#endif
