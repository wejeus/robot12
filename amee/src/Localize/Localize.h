#ifndef LOCALIZE_H
#define LOCALIZE_H

#include <std_msgs/Int32.h>
#include "amee/Velocity.h"
#include "amee/Odometry.h"
#include "amee/Pose.h"
#include "roboard_drivers/Encoder.h"
#include "roboard_drivers/Motor.h"

class Localize {

#define X 0.0f

private:

public:

	amee::Velocity speed;

	roboard_drivers::Encoder encoderMeasurement;
	roboard_drivers::Encoder lastEncoderMeasurement;

	//amee::IMU imuMeasurement;

	//roboard_drivers::Encoder measurement;
	amee::Pose mMeasurement1;
	amee::Pose mMeasurement2;

	amee::Velocity controlSignal;
	
	amee::Pose pose; // set to zero as default
 	amee::Pose lastPose;
	amee::Pose newPose; 


	void init();

	void receiveEncoder(const roboard_drivers::Encoder::ConstPtr &msg);
	void receiveMotorSpeed(const roboard_drivers::Motor::ConstPtr &v);
	void receiveControlSignal(const amee::Velocity::ConstPtr &msg);
	
	void publishPose(ros::Publisher pose_pub);

protected:
	
		
};
#endif