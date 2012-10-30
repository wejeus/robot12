#include "ros/ros.h"
#include "MovementControl.h"
#include "amee/Velocity.h"
#include <iostream>
#include <cmath>
#include "MoveRotate.h"
#include "MoveStraight.h"
#include "MoveStop.h"
#include "MoveFollowWall.h"

using namespace amee;


MovementControl::MovementControl(ros::Publisher pub) {
	mRotationState = new MoveRotate(pub);
	mStraightState = new MoveStraight(pub);
	mStopState = new MoveStop(pub);
	mFollowWallState = new MoveFollowWall(pub);
	mCurrentState = mStopState;
}

MovementControl::~MovementControl() {
	delete mRotationState;
	delete mStraightState;
	delete mStopState;
	delete mFollowWallState;
}

void MovementControl::setSpeedPublisher(ros::Publisher& pub) {
	speed_pub = pub;
}

// Callback for IR distances
void MovementControl::receive_distances(const IRDistances::ConstPtr &msg)
{
	mSensorData.irdistances.rightFront = msg->rightFront;
	mSensorData.irdistances.rightBack = msg->rightBack;
	mSensorData.irdistances.frontShortRange = msg->frontShortRange;
	mSensorData.irdistances.wheelRight = msg->wheelRight;
}

void MovementControl::receive_odometry(const Odometry::ConstPtr &msg) {
	mSensorData.odometry.angle = msg->angle;
	mSensorData.odometry.distance = msg->distance;
	// TODO others
}

void MovementControl::doControl() {
	if (mCurrentState->isRunning()) {
		mCurrentState->doControl(mSensorData);
	}
}

void MovementControl::receive_command(const amee::MovementCommand::ConstPtr &msg) {
	int type = msg->type;
	float angle = msg->angle;
	float distance = msg->distance;
	switch(type) {
		case TYPE_MOVE_STRAIGHT:
			std::cout << "MOVE STRAIGHT COMMAND RECEIVED" << std::endl;
			mCurrentState = mStraightState;
			mStraightState->init(mSensorData, distance);
		break;
		case TYPE_MOVE_ROTATE:		
			std::cout << "ROTATE COMMAND RECEIVED" << std::endl;
			mCurrentState = mRotationState;
			mRotationState->init(mSensorData, angle);
		break;
		case TYPE_MOVE_COORDINATE:
		 	std::cout << "MOVE TO COORDINATE" << std::endl;
		 	//TODO initialize MoveCoonrdinate here
		 	// and change state
		break;
		case TYPE_FOLLOW_WALL:
		 	std::cout << "FOLLOW WALL" << std::endl;
		 	mFollowWallState->init(mSensorData);
		 	mCurrentState = mFollowWallState;
		break;
		case TYPE_STOP:
		 	std::cout << "STOP COMMAND RECEIVED" << std::endl;
		 	mCurrentState = mStopState;
		break;
		default: std::cout << "GAY COMMAND RECEIVED" << std::endl;
	}
}

void MovementControl::init() {
	mRotationState->init(mSensorData);
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "MovementControlNode");//Creates a node named "MotorControl"
	ros::NodeHandle n;

	ros::Publisher	vel_pub;
	vel_pub = n.advertise<Velocity>("/amee/motor_control/set_wheel_velocities", 100);

	// create the controller and initialize it
	MovementControl control(vel_pub);
	

	ros::Subscriber dist_sub;

	// create subscriber for distances
	dist_sub = n.subscribe("/amee/sensors/irdistances", 100, &MovementControl::receive_distances, &control);
	ros::Subscriber odo_sub = n.subscribe("/amee/motor_control/odometry", 100, &MovementControl::receive_odometry, &control);

	ros::Subscriber command_sub = n.subscribe("/MovementControl/MovementCommand",10,&MovementControl::receive_command, &control);

	ros::Rate loop_rate(20);
	while(vel_pub.getNumSubscribers() == 0 && ros::ok()) {
		loop_rate.sleep();
	} 

	control.init();
	
	while(ros::ok()){
		
		// go to sleep for a short while
		loop_rate.sleep();

		// call all callbacks
		ros::spinOnce();
		
		// drive!
		control.doControl();
	}

	return 0;
}
