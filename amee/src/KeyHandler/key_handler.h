#ifndef KEY_HANDLER_H
#define KEY_HANDLER_H

#include "ros/ros.h"
#include "std_msgs/String.h"
#include "amee/Velocity.h"
#include "roboard_drivers/Motor.h"
//#include <QWSKeyboardHandler>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

using namespace amee;
using namespace roboard_drivers;

namespace amee{
#define DEF_VELO_RATE_L -0.5
#define DEF_VELO_RATE_R -0.5

	class KeyHandler{

	/* Hopefully this will work for all types of keyboards with arrows */
	enum {LEFT_YAW = 108, RIGHT_YAW = 114, KEYCODE_SPACE = 32, KEYCODE_U = 65, KEYCODE_D, KEYCODE_R, KEYCODE_L};//TODO Add page up and page down for rotating when 1 wheel is locked


	private:
		bool sendToMotorControl;
		float VELO_RATE_L;
		float VELO_RATE_R;
		ros::Publisher keyCom_pub;
	
		inline void setWheels(Motor &, const float L, const float R) const;
		inline void setVel(Velocity &, const float L, const float R) const;

	public:
		void setVeloRate(float left, float right);
		void initDefVeloRate();
		void setKeyComPublisher(ros::Publisher &);
		void handleKeyStroke() const;
		inline int kbhit(bool) const;
		inline const float & getVeloRateL() const;
		inline const float & getVeloRateR() const;
		inline const bool & isMotorControlPub() const;
		void setMotorControlPub(bool);

	};



};//namespace amee

#endif
