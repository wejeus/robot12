#ifndef MOTOR_CONTROL_H
#define MOTOR_CONTROL_H

#include <std_msgs/Int32.h>
#include "roboard_drivers/Encoder.h"
#include "roboard_drivers/Motor.h"
#include "amee/Velocity.h"
#include "amee/Odometry.h"

class MotorControl {
private:
	amee::Velocity mVelocity;
	amee::Odometry mOdometry;
	float mAngle;
	roboard_drivers::Encoder mPrevEncoder;
	roboard_drivers::Encoder mCurrentEncoder;
	roboard_drivers::Encoder mMeasurementAccumulator;
	roboard_drivers::Motor mMotor;
	int mMeasurementValidCounter;
	int mMeasurementCounter; 
	ros::Publisher mot_pub;
	ros::Publisher odo_pub;

	public:
		void receive_encoder(const roboard_drivers::Encoder::ConstPtr &msg);
		void receive_speed(const amee::Velocity::ConstPtr &v);
		// Sets the given speed ( in m/s ) to both motors.
		void setSpeed(float vLeft, float vRight);
		void drive(); // call this frequently to make the robot drive
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
