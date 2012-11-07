#include "ros/ros.h"
#include <string.h>
//#include "MotorControl.h"
#include <iostream>
#include <cmath>
#include "../Amee.h"
#include "Localize.h"
#include "EKF.h"
#include "Eigen/Eigen"
//#include "roboard_drivers/Encoder"

void Localize::init() 
{


}

void Localize::receiveMotorSpeed(const roboard_drivers::Motor::ConstPtr &v) {
	speed.left  = v->left;
	speed.right = v->right;
	speed.linear = (v->left + v->right)/2;
	speed.angular = (v->left - v->right)/WHEEL_BASE;
}

void Localize::receiveEncoder(const amee::Odometry::ConstPtr &msg) {
	encoderMeasurement.leftWheelDistance = msg->leftWheelDistance;
	encoderMeasurement.rightWheelDistance = msg->rightWheelDistance;
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

	pose_pub = n.advertise<amee::Pose>("/pose", 100); // Publish pose
	
	ros::Rate loop_rate(40);

	// Create filter
	EKF ekf;
	ekf.init();


	// -- Estimate pose
	while(ros::ok())
	{
		localize.lastEncoderMeasurement = localize.encoderMeasurement; // Save last before spinOnce

		ros::spinOnce();   // call all callbacks

		localize.measurement.leftWheelDistance  = localize.encoderMeasurement.leftWheelDistance  - localize.lastEncoderMeasurement.leftWheelDistance;
		localize.measurement.rightWheelDistance = localize.encoderMeasurement.rightWheelDistance - localize.lastEncoderMeasurement.rightWheelDistance;
		//localize.measurement.IMUtheta = ;...
		
		ekf.estimate(localize.controlSignal, localize.measurement); // have pose in the init function instead!
		
		// -- Publish pose
		localize.publishPose(pose_pub);

		loop_rate.sleep(); // go to sleep for a short while

	}



	return 0;
}
