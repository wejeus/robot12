#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <std_msgs/Int32.h>
#include "robo/Encoder.h"
#include "robo/Motor.h"
#include "amee/Velocity.h"
#include "amee/Odometry.h"

class MotorControl {
private:
	amee::Velocity mVelocity;
	amee::Odometry mOdometry;
	float mAngle;
	robo::Encoder mPrevEncoder;
	robo::Encoder mCurrentEncoder;
	robo::Encoder mMeasurementAccumulator;
	robo::Motor mMotor;
	int mMeasurementValidCounter;
	int mMeasurementCounter; 
	ros::Publisher mot_pub;
	ros::Publisher odo_pub;

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
		void setOdometryPublisher(ros::Publisher pub);
	protected:
		void calcWheelPWMVelocities(amee::Velocity& velocity);
	private:
		void publishOdometry();
};
#endif
