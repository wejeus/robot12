#include <iostream>
#include "StrategyExplore.h"
#include "../Mapper/Mapper.h"
#include "../MovementControl/MovementControl.h"
#include "amee/MovementCommand.h"
#include "amee/Path.h"
#include <cmath>

using namespace amee;

StrategyExplore::StrategyExplore(ros::Publisher &pub, ros::Publisher &phaseInfo, ros::Publisher &pathPub) {
	mCommandPub = pub;
	mPhaseInfo = phaseInfo;
	mPathPub = pathPub;
	mRunning = false;
	// mRestartFollowingWall = true;
	// mFollowWallPossible = false;

	mStrategyGoTo = new StrategyGoTo(pub, phaseInfo, pathPub);
}

StrategyExplore::~StrategyExplore() {
	delete mStrategyGoTo;
}

void StrategyExplore::init(const amee::Pose &pose, const amee::GraphMsg::ConstPtr& graphMsg) {
	mGraph = graphMsg;
	//mFollowingWall = false;

	std::cout << "StrategyExplore init, explore maze" << std::endl;

	MovementCommand mc;
 	mc.type = 4; // MoveFollowWall
 	mCommandPub.publish(mc);

	mRunning = true;
}

bool StrategyExplore::isRunning() const {
	return mRunning;
}


void StrategyExplore::stop() {
		MovementCommand mc;
	 	mc.type = 5; // MoveStop
	 	mCommandPub.publish(mc);
}

void StrategyExplore::receive_pose(const amee::Pose::ConstPtr &msg){ 
	mPose = (*msg);
	mStrategyGoTo->receive_pose(msg);
}

void StrategyExplore::receive_graph(const amee::GraphMsg::ConstPtr &msg){}

void StrategyExplore::receive_mapper_event(const amee::MapperEvent::ConstPtr &msg){
	if (msg->type == Mapper::NodeReached) {
		// TODO: find closest unvisited grid
		// TODO: goto found grid... mStrategyGoTo->init(mPose, )
	}
}

void StrategyExplore::receive_movement_event(const amee::MovementEvent::ConstPtr &msg){
	if (msg->type == MovementControl::MOVEMENT_EVENT_TYPE_OBSTICLE_IN_FRONT) {
		// TODO: followWall();
	}	
}

void StrategyExplore::receive_timerP1(const ros::TimerEvent &event) {
	unsigned int i = 0;
	//mStrategyGoTo->init(mPose, graphMsg, i);
}

void StrategyExplore::receive_timerP2(const ros::TimerEvent &event) {}

