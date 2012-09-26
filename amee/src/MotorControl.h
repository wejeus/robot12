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
	int mMeasurementValidCounter;
	int mMeasurementCounter; 
	ros::Publisher	mot_pub;

	public:
		void receive_encoder(const robo::Encoder::ConstPtr &msg);
		void receive_speed(const amee::Velocity::ConstPtr &v);
		void setSpeed(float vLeft, float vRight);
		void drive();
		void rotate(float degrees);
		void stop();
		void init();
		bool measurementsValid();
		void setMotorPublisher(ros::Publisher pub);
	protected:
		void calcWheelPWMVelocities(amee::Velocity& velocity);
};
#endif
