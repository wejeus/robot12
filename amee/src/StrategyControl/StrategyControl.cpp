#include "ros/ros.h"
#include "StrategyControl.h"
#include "StrategyExplore.h"
#include "StrategyClassify.h"
#include "StrategyGetOut.h"
#include "StrategyGo2Tag.h"
#include <iostream>
#include <cmath>
#include "amee/StrategyCommand.h"
#include "amee/MovementCommand.h"
#include <std_msgs/Int32.h>

using namespace amee;


StrategyControl::StrategyControl(ros::Publisher& pub) {
	mClassifyState = new StrategyClassify(pub);
	mExploreState = new StrategyExplore(pub);
	mGo2TagState = new StrategyGo2Tag(pub);
	mGetOutState = new StrategyGetOut(pub);

	mCurrentState = NULL;
}

StrategyControl::~StrategyControl() {
	delete mClassifyState;
	delete mExploreState;
	delete mGo2TagState;
	delete mGetOutState;
}

void StrategyControl::doControl() {
	if (mCurrentState != NULL && mCurrentState->isRunning()) {
		mCurrentState->doControl(mStrategyData);
	}
}

void StrategyControl::receive_pose(const amee::Pose::ConstPtr &msg){
	//receive pose
}

void StrategyControl::receive_command(const amee::StrategyCommand::ConstPtr &msg) {
	int type = msg->type;
	//float angle = msg->angle;
	//float distance = msg->distance;
	switch(type) {
		case TYPE_STRATEGY_EXPLORE:
			std::cout << "STRATEGY_EXPLORE" << std::endl;
			mCurrentState = mExploreState;
			mExploreState->init(mStrategyData);
		break;
		case TYPE_STRATEGY_GO2TAG:		
			std::cout << "STRATEGY GO2TAG" << std::endl;
			mCurrentState = mGo2TagState;
			mGo2TagState->init(mStrategyData);
		break;
		case TYPE_STRATEGY_CLASSIFY:
		 	std::cout << "STRATEGY CLASSIFY" << std::endl;
			mCurrentState = mClassifyState;
			mClassifyState->init(mStrategyData);
		break;
		case TYPE_STRATEGY_GET_OUT:
		 	std::cout << "STRATEGY GET_OUT" << std::endl;
		 	mCurrentState = mGetOutState;
		 	mGetOutState->init(mStrategyData);

		break;
		// default: std::cout << "GAY COMMAND RECEIVED" << std::endl;
	}
}

void StrategyControl::init() {
	// mCurrentState->init(mStrategyData);
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "StrategyControlNode");//Creates a node named "StrategyControl"
	ros::NodeHandle n;

	// create the controller and initialize it
	ros::Publisher pub = n.advertise<amee::MovementCommand>("/MovementControl/MovementCommand", 1);
	//StrategyControl control(vel_pub, wall_pub);
	StrategyControl control(pub);
	

	ros::Subscriber dist_sub;

	// create subscriber for distances
	/*
	dist_sub = n.subscribe("/amee/sensors/irdistances", 100, &StrategyControl::receive_distances, &control);
	ros::Subscriber odo_sub = n.subscribe("/amee/motor_control/odometry", 100, &StrategyControl::receive_odometry, &control);
	ros::Subscriber sonar_sub = n.subscribe("/roboard/sonar",1, &StrategyControl::receive_sonar, &control);
	*/
	ros::Subscriber command_sub = n.subscribe("/StrategyControl/StrategyCommand", 10, &StrategyControl::receive_command, &control);
	ros::Subscriber pos_sub = n.subscribe("/amee/pose", 100, &StrategyControl::receive_pose, &control);
	

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
