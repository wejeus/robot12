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
	mLastOdometry = mOdometry;
	std::cout << mLastOdometry.timestamp << " " << mOdometry.timestamp << std::endl;
	mOdometry.x = msg->x;
	mOdometry.y = msg->y;
	mOdometry.angle = msg->angle;
	mOdometry.timestamp = msg->timestamp;
	mOdometry.leftWheelDistance = msg->leftWheelDistance;
	mOdometry.rightWheelDistance = msg->rightWheelDistance;
	//std::cout << "Odometry " << mOdometry.x << " " << mOdometry.y << " " << mOdometry.angle << std::endl;
	mPose.theta += mOdometry.angle - mLastOdometry.angle;

	// Calc x & y with corrected theta
	float leftDistance = mOdometry.leftWheelDistance - mLastOdometry.leftWheelDistance;
	float rightDistance = mOdometry.rightWheelDistance - mLastOdometry.rightWheelDistance;
	float distance = (leftDistance + rightDistance) / 2.0f;

	//std::cout << "distance: " << leftDistance << " " << rightDistance << std::endl;

	mPose.x += cos(mPose.theta / 180.0f * M_PI) * distance;
	mPose.y += sin(mPose.theta / 180.0f * M_PI) * distance;
	mPose.theta = mPose.theta - floor(mPose.theta / 360) * 360.0f;
}

void Localize::receiveFollowWallState(const amee::FollowWallStates::ConstPtr &msg) {
	followWallState.state = msg->state;
	if(followWallState.state == amee::MoveFollowWall::ALIGNED_TO_WALL) {
		mPose.theta -= mPose.theta - floor(mPose.theta / 90.0f + 0.5f) * 90.0f;
	}
	mPose.theta = mPose.theta - floor(mPose.theta / 360) * 360.0f;
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
	followWallState_sub= n.subscribe("/amee/follow_wall_states", 100, &Localize::receiveFollowWallState, &localize); // Subscribe motorSpeed

	pose_pub = n.advertise<amee::Pose>("/amee/pose", 100); // Publish pose
	
	ros::Rate loop_rate(40);

	// -- Estimate pose
	while(ros::ok())
	{
		ros::spinOnce(); // call all callbacks

		// -- Publish pose
		pose_pub.publish(localize.mPose);
		// Sleep for a while
		loop_rate.sleep(); 

	}

	return 0;
}