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
#include "../MovementControl/MoveFollowWall.h"
//#include "roboard_drivers/Encoder"


void Localize::init() 
{
	ros::spinOnce();
	mPose.x = mOdometry.x;
	mPose.y = mOdometry.y;
	mPose.theta = mOdometry.angle;

}

void Localize::receiveOdometry(const amee::Odometry::ConstPtr &msg) {
	mOdometry.x = msg->x;
	mOdometry.y = msg->y;
	mOdometry.angle = msg->angle;
}

void Localize::receiveFollowWallState(const amee::FollowWallStates::ConstPtr &msg) {
	followWallState.state = msg->state;
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
	ros::Subscriber odo_sub;
	ros::Subscriber followWallState_sub;
	ros::Publisher  pose_pub;

	odo_sub            = n.subscribe("/amee/motor_control/odometry", 100, &Localize::receiveOdometry, &localize);  // Subscribe encoder
	followWallState_sub= n.subscribe("/amee/followWallStates", 100, &Localize::receiveFollowWallState, &localize); // Subscribe motorSpeed

	pose_pub = n.advertise<amee::Pose>("/amee/pose", 100); // Publish pose
	
	ros::Rate loop_rate(40);

	// -- Estimate pose
	while(ros::ok())
	{
		localize.mLastOdometry = localize.mOdometry; // save old odometry values
		ros::spinOnce(); // call all callbacks

		// dont manipulate x & y.
		localize.mPose.x = localize.mOdometry.x;
		localize.mPose.y = localize.mOdometry.y;
		localize.mPose.theta += localize.mOdometry.angle - localize.mLastOdometry.angle;

		// Correct angle if aligned to wall
		if(localize.followWallState.state == amee::MoveFollowWall::ALIGNED_TO_WALL) {
			localize.mPose.theta -= localize.mPose.theta - floor(localize.mPose.theta / 90 + 0.5) * 90.0f;
		}

		// -- Publish pose
		pose_pub.publish(localize.mPose);

		// Sleep for a while
		loop_rate.sleep(); 

	}

	return 0;
}