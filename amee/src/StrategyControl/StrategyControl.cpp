#include "ros/ros.h"
#include "StrategyControl.h"
#include <iostream>
#include <cmath>
#include <std_msgs/Int32.h>

using namespace amee;


StrategyControl::StrategyControl(ros::Publisher& pub, ros::Publisher& statesPub) {
	/*
	mRotationState = new MoveRotate(pub);
	mStraightState = new MoveStraight(pub);
	mStopState = new MoveStop(pub);
	mFollowWallState = new MoveFollowWall(pub, statesPub);
	mAlignWallState = new MoveAlignWall(pub);
	mCurrentState = mStopState;
	*/
}

StrategyControl::~StrategyControl() {
	/*
	delete mRotationState;
	delete mStraightState;
	delete mStopState;
	delete mFollowWallState;
	delete mAlignWallState;
	*/
}

void StrategyControl::setSpeedPublisher(ros::Publisher& pub) {
	speed_pub = pub;
}

// Callback for IR distances
void StrategyControl::receive_distances(const IRDistances::ConstPtr &msg)
{
	/*
	mSensorData.irdistances.rightFront = msg->rightFront;
	mSensorData.irdistances.rightBack = msg->rightBack;
	// mSensorData.irdistances.frontShortRange = msg->frontShortRange;
	mSensorData.irdistances.obstacleInFront = msg->obstacleInFront;
	mSensorData.irdistances.wheelRight = msg->wheelRight;
	mSensorData.irdistances.leftBack = msg->leftBack;
	mSensorData.irdistances.leftFront = msg->leftFront;
	mSensorData.irdistances.wheelLeft = msg->wheelLeft;
	*/
}

void StrategyControl::receive_odometry(const Odometry::ConstPtr &msg) {
	/*
	mSensorData.odometry.angle = msg->angle;
	mSensorData.odometry.distance = msg->distance;
	mSensorData.odometry.x = msg->x;
	mSensorData.odometry.y = msg->y;
	// TODO others
	*/
}

void StrategyControl::doControl() {
	/*
	if (mCurrentState->isRunning()) {
		mCurrentState->doControl(mSensorData);
	}
	*/
}

void StrategyControl::receive_sonar(const roboard_drivers::sonar::ConstPtr &msg) {
	mSensorData.sonarDistance = msg->distance;
	// std::cout << "Sonar received: " << mSensorData.sonarDistance << std::endl;
}

void StrategyControl::receive_command(const amee::StrategyCommand::ConstPtr &msg) {
	int type = msg->type;
	float angle = msg->angle;
	float distance = msg->distance;
	switch(type) {
		case TYPE_MOVE_STRAIGHT:
			// std::cout << "MOVE STRAIGHT COMMAND RECEIVED" << std::endl;
			mCurrentState = mStraightState;
			mStraightState->init(mSensorData, distance);
		break;
		case TYPE_MOVE_ROTATE:		
			// std::cout << "ROTATE COMMAND RECEIVED" << std::endl;
			mCurrentState = mRotationState;
			mRotationState->init(mSensorData, angle);
		break;
		case TYPE_MOVE_COORDINATE:
		 	// std::cout << "MOVE TO COORDINATE" << std::endl;
		 	//TODO initialize MoveCoonrdinate here
		 	// and change state
		break;
		case TYPE_FOLLOW_WALL:
		 	// std::cout << "FOLLOW WALL" << std::endl;
		 	mFollowWallState->init(mSensorData);
		 	mCurrentState = mFollowWallState;
		break;
		case TYPE_STOP:
		 	// std::cout << "STOP COMMAND RECEIVED" << std::endl;
		 	mCurrentState = mStopState;
		break;
		case TYPE_ALIGN_TO_WALL:
			// std::cout << "ALIGN TO WALL COMMAND RECEIVED" << std::endl;
			mAlignWallState->init(mSensorData);
			mCurrentState = mAlignWallState;
		break;
		// default: std::cout << "GAY COMMAND RECEIVED" << std::endl;
	}
}

void StrategyControl::init() {
	mRotationState->init(mSensorData);
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "StrategyControlNode");//Creates a node named "StrategyControl"
	ros::NodeHandle n;

	// create the controller and initialize it
	StrategyControl control(vel_pub, wall_pub);
	

	ros::Subscriber dist_sub;

	// create subscriber for distances
	dist_sub = n.subscribe("/amee/sensors/irdistances", 100, &StrategyControl::receive_distances, &control);
	ros::Subscriber odo_sub = n.subscribe("/amee/motor_control/odometry", 100, &StrategyControl::receive_odometry, &control);
	ros::Subscriber sonar_sub = n.subscribe("/roboard/sonar",1, &StrategyControl::receive_sonar, &control);

	ros::Subscriber command_sub = n.subscribe("/StrategyControl/StrategyCommand",10,&StrategyControl::receive_command, &control);

	ros::Rate loop_rate(20);

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
