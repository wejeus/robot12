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
	//float angle = msg->angle;
	//float distance = msg->distance;
	switch(type) {
		case TYPE_STRATEGY_EXPLORE:
			std::cout << "STRATEGY_EXPLORE" << std::endl;
			//TODO: implement me
			//mCurrentState = mExploreState;
			//mExploreState->init(mSensorData);
		break;
		case TYPE_STRATEGY_GO2TAG:		
			std::cout << "STRATEGY GO2TAG" << std::endl;
			//TODO: implement me
			//mCurrentState = mGo2TagState;
			//mGo2TagState->init(mSensorData);
		break;
		case TYPE_STRATEGY_CLASSIFY:
		 	std::cout << "STRATEGY CLASSIFY" << std::endl;
		 	//TODO implement me
			//mCurrentState = mClassifyState;
			//mClassifyState->init(mSensorData);
		break;
		case TYPE_STRATEGY_GET_OUT:
		 	std::cout << "STRATEGY GET_OUT" << std::endl;
			//TODO: implement me
		 	//mCurrentState = mGetOutState;
		 	//mGetOutState->init(mSensorData);

		break;
		// default: std::cout << "GAY COMMAND RECEIVED" << std::endl;
	}
}

void StrategyControl::init() {
	//mRotationState->init(mSensorData);
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "StrategyControlNode");//Creates a node named "StrategyControl"
	ros::NodeHandle n;

	// create the controller and initialize it
	//StrategyControl control(vel_pub, wall_pub);
	StrategyControl control();
	

	ros::Subscriber dist_sub;

	// create subscriber for distances
	/*
	dist_sub = n.subscribe("/amee/sensors/irdistances", 100, &StrategyControl::receive_distances, &control);
	ros::Subscriber odo_sub = n.subscribe("/amee/motor_control/odometry", 100, &StrategyControl::receive_odometry, &control);
	ros::Subscriber sonar_sub = n.subscribe("/roboard/sonar",1, &StrategyControl::receive_sonar, &control);

	ros::Subscriber command_sub = n.subscribe("/StrategyControl/StrategyCommand",10,&StrategyControl::receive_command, &control);
	*/

	ros::Rate loop_rate(20);

	//control.init();
	
	while(ros::ok()){
		
		// go to sleep for a short while
		loop_rate.sleep();

		// call all callbacks
		ros::spinOnce();
		
		// drive!
		//control.doControl();
	}

	return 0;
}
