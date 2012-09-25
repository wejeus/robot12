#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <std_msgs/Int32.h>
#include "robo/Encoder.h"
#include "robo/Motor.h"
#include "amee/Velocity.h"

class MotorControl {
private:
	amee::Velocity mVelocity;
	robo::Encoder mPrevEncoder;
	robo::Encoder mCurrentEncoder;
	robo::Encoder mMeasurementAccumulator;
	robo::Motor mMotor;
	int mInit;
	int mMeasurementCounter; 
	ros::Publisher	mot_pub;

	public:
		void receive_encoder(const robo::Encoder::ConstPtr &msg);
		void calcWheelPWMVelocities(amee::Velocity& velocity);
		void linearVelocityControl(float linearVelocity);
		void init();
		bool isInitialized();
		void setMotorPublisher(ros::Publisher pub);

};
#endif
