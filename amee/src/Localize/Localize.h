#ifndef LOCALIZE_H
#define LOCALIZE_H

#include <std_msgs/Int32.h>
#include "amee/Velocity.h"
#include "amee/Odometry.h"
#include "amee/Pose.h"
#include "Eigen/Eigen"

class Localize {

#define X 0.0f

private:

public:

	amee::Velocity speed;

	amee::Odometry encoderMeasurement;
	amee::Odometry lastEncoderMeasurement;

	//amee::IMU imuMeasurement;

	amee::Odometry measurement;

	amee::Velocity controlSignal;
	
	amee::Pose pose; // set to zero as default
 	amee::Pose lastPose;
	amee::Pose newPose; 

	void init();

	void receiveEncoder(const amee::Odometry::ConstPtr &msg);
	void receiveMotorSpeed(const amee::Velocity::ConstPtr &v);
	void receiveControlSignal(const amee::Velocity::ConstPtr &msg);
	
	void publishPose(ros::Publisher pose_pub);

protected:
	
		
};
#endif