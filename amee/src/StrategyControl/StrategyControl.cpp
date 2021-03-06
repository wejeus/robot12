#include "ros/ros.h"

#include "StrategyControl.h"
// #include "StrategyExplore.h"
// #include "StrategyClassify.h"
#include "StrategyGoTo.h"
#include "amee/Path.h"

#include <iostream>
#include <cmath>

#include "amee/StrategyCommand.h"
#include "amee/MovementCommand.h"
#include "amee/NodeMsg.h"

#include <std_msgs/Int32.h>

using namespace amee;


StrategyControl::StrategyControl(ros::Publisher& pub, ros::Publisher &phaseInfo, ros::Publisher& pathPub) {
	// mClassifyState = new StrategyClassify(pub);
	// mExploreState = new StrategyExplore(pub);
	mGoToState = new StrategyGoTo(pub, phaseInfo, pathPub);
	mCurrentState = mGoToState;
}

StrategyControl::~StrategyControl() {
	// delete mClassifyState;
	// delete mExploreState;
	delete mGoToState;
}

// void StrategyControl::doControl() {
// 	if (mCurrentState != NULL && mCurrentState->isRunning()) {
// 		mCurrentState->doControl(mStrategyData);
// 	}
// }

void StrategyControl::receive_pose(const amee::Pose::ConstPtr &msg) {
	mCurrentState->receive_pose(msg);
	mPose = (*msg);
}

void StrategyControl::receive_graph(const amee::GraphMsg::ConstPtr &msg) {
	mCurrentState->receive_graph(msg);
	mGraphMsg = msg;
}

void StrategyControl::receive_command(const amee::StrategyCommand::ConstPtr &msg) {
	int type = msg->type;
	// float x = msg->x;
	// float y = msg->y;
	unsigned int id = msg->nodeId;	
	switch(type) {
		case TYPE_STRATEGY_EXPLORE:
			std::cout << "STRATEGY_EXPLORE" << std::endl;
			// mCurrentState = mExploreState;
			// mExploreState->init(mStrategyData);
		break;
		case TYPE_STRATEGY_GO_TO:
			std::cout << "STRATEGY GO TO" << std::endl;
			mCurrentState = mGoToState;
			mGoToState->init(mPose, mGraphMsg, id);//TODO change this to add the end position, also add the init(mStrategyData, something else) to the StrategyGoTo class
		break;
		case TYPE_STRATEGY_CLASSIFY:
		 // 	std::cout << "STRATEGY CLASSIFY" << std::endl;
			// mCurrentState = mClassifyState;
			// mClassifyState->init(mStrategyData);
		break;
		case TYPE_STRATEGY_GET_OUT:
			// if(!mMapInitialized){
			// 	std::cout << "Map is not initialized, can not find exit!" << std::endl; return;
			// }
		 // 	std::cout << "STRATEGY GET_OUT" << std::endl;
			// mCurrentState = mGoToState;
			// mGoToState->init(mStrategyData, mGraphMsg);

		break;
		// default: std::cout << "GAY COMMAND RECEIVED" << std::endl;
	}
}

void StrategyControl::receive_mapper_event(const amee::MapperEvent::ConstPtr &msg) {
	mCurrentState->receive_mapper_event(msg);
}

void StrategyControl::receive_movement_event(const amee::MovementEvent::ConstPtr &msg) {
	mCurrentState->receive_movement_event(msg);
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
	ros::Publisher phaseInfo = n.advertise<std_msgs::Int32>("/StrategyControl/PhaseInfo", 100);
	ros::Publisher pathPub = n.advertise<amee::Path>("/amee/graph/path", 10);

	//StrategyControl control(vel_pub, wall_pub);
	StrategyControl control(pub, phaseInfo, pathPub);
	

	ros::Subscriber dist_sub;

	// create subscriber for strategy command, pose and graph
	ros::Subscriber command_sub = n.subscribe("/StrategyControl/StrategyCommand", 10, &StrategyControl::receive_command, &control);
	ros::Subscriber pos_sub = n.subscribe("/amee/pose", 100, &StrategyControl::receive_pose, &control);
	ros::Subscriber graph_sub = n.subscribe("/amee/map/graph", 1, &StrategyControl::receive_graph, &control);
	ros::Subscriber mapper_events_sub = n.subscribe("/amee/map/mapper_events", 2, &StrategyControl::receive_mapper_event, &control);
	ros::Subscriber move_events_sub = n.subscribe("/amee/movement_events", 2, &StrategyControl::receive_movement_event, &control);
	

	// ros::Rate loop_rate(6);

	control.init();
	
	ros::spin();

	// while(ros::ok()){
		
	// 	// go to sleep for a short while
	// 	loop_rate.sleep();

	// 	// call all callbacks
	// 	ros::spinOnce();
		
	// 	// drive!
	// 	control.doControl();
	// }

	return 0;
}
