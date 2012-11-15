#include "ros/ros.h"
#include <string.h>
//#include "MotorControl.h"
#include <iostream>
#include <cmath>
#include "../Amee.h"
#include "Localize.h"
#include "roboard_drivers/Encoder.h"
#include "EKF.h"
#include "Eigen/Eigen"
//#include "roboard_drivers/Encoder"

void Localize::init() 
{
	controlSignal.left    = 0;
	controlSignal.right   = 0;
	controlSignal.linear  = 0;
	controlSignal.angular = 0;

	encoderMeasurement.left  = 0;
	encoderMeasurement.right = 0;	

	speed.left    = 0;
	speed.right   = 0;
	speed.linear  = 0;
	speed.angular = 0;
}

void Localize::receiveMotorSpeed(const roboard_drivers::Motor::ConstPtr &v) {
	speed.left  = v->left;
	speed.right = v->right;
	speed.linear = (v->left + v->right)/2;
	speed.angular = (v->left - v->right)/WHEEL_BASE;
}

void Localize::receiveEncoder(const roboard_drivers::Encoder::ConstPtr &msg) {
	encoderMeasurement.left = msg->left;
	encoderMeasurement.right = msg->right;
}

void Localize::receiveControlSignal(const amee::Velocity::ConstPtr &ctrl) {
	controlSignal.left  = ctrl->left;
	controlSignal.right = ctrl->right;
	controlSignal.linear = (ctrl->left + ctrl->right)/2;
	controlSignal.angular = (ctrl->left - ctrl->right)/WHEEL_BASE;
}

void Localize::publishPose(ros::Publisher pose_pub) {
	pose_pub.publish(pose);
}



int main(int argc, char **argv) 
{
	// -- Initiate ROS node
	ros::init(argc, argv, "Localize"); //Creates a node named "Localize"
	ros::NodeHandle n;

	// -- Initialize localizer
	Localize localize;
	localize.init();		

	// -- Subscribe & Publish topics
	ros::Subscriber enc_sub;
	ros::Subscriber motorSpeed_sub;
	ros::Subscriber controlSignal_sub;
	ros::Publisher  pose_pub;

	enc_sub           = n.subscribe("/serial/encoder",     100, &Localize::receiveEncoder, &localize);       // Subscribe encoder
	motorSpeed_sub    = n.subscribe("/serial/motor_speed", 100, &Localize::receiveMotorSpeed, &localize);    // Subscribe motorSpeed
	controlSignal_sub = n.subscribe("/amee/motor_control/set_wheel_velocities", 100, &Localize::receiveControlSignal, &localize); // Subscribe contrlSignal

	pose_pub = n.advertise<amee::Pose>("/amee/pose", 100); // Publish pose
	
	ros::Rate loop_rate(40);

	// Initiate Kalman filter
	EKF ekf;
	ekf.init();


	// -- Estimate pose
	while(ros::ok())
	{
		localize.lastEncoderMeasurement = localize.encoderMeasurement; // Save last before updating with spinOnce

		ros::spinOnce(); // call all callbacks

		// Calc relative measurements in tics
		localize.measurement.left  = (localize.encoderMeasurement.left  - localize.lastEncoderMeasurement.left) * TICS_PER_REVOLUTION / (2.0f * M_PI * WHEEL_RADIUS);;
		localize.measurement.right = (localize.encoderMeasurement.right - localize.lastEncoderMeasurement.right) * TICS_PER_REVOLUTION / (2.0f * M_PI * WHEEL_RADIUS);;
		//localize.measurement.IMUtheta = ;...
		
		// Estimate pose w kalman filter
		ekf.estimate(localize.controlSignal, localize.measurement);
		
		// Get pose from kalman filter
		localize.pose = ekf.getPose();
		
		// -- Publish pose
		localize.publishPose(pose_pub);

		// Sleep for a while
		loop_rate.sleep(); 

	}

	return 0;
}