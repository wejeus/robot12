#include <iostream>
#include "StrategyRescue.h"
#include "../Mapper/Mapper.h"
#include "../MovementControl/MovementControl.h"
#include "amee/MovementCommand.h"
#include "amee/Path.h"
#include <cmath>

using namespace amee;

StrategyRescue::StrategyRescue(ros::Publisher &pub, ros::Publisher &phaseInfo, ros::Publisher &pathPub) {
	mCommandPub = pub;
	mPhaseInfo = phaseInfo;
	mPathPub = pathPub;
	mRunning = false;
	// mRestartFollowingWall = true;
	// mFollowWallPossible = false;

	mStrategyGoTo = new StrategyGoTo(pub, phaseInfo, pathPub);
}

StrategyRescue::~StrategyRescue() {
	delete mStrategyGoTo;
}

void StrategyRescue::init(const amee::Pose &pose, const amee::GraphMsg::ConstPtr& graphMsg) {
	mGraph = graphMsg;
	mFollowingWall = false;

	// TODO: filter out tag ID:s
	//unsigned int[] mTagNodeID = StrategyRescue::filterTagNodes(mGraph); 
	unsigned int mTagNodeID[2];
	mStrategyGoTo->init(pose, graphMsg, mTagNodeID[0]);
	std::cout << "StrategyRescue init, rescue tag" << mTagNodeID[0] << std::endl;

	mRunning = true;

}

bool StrategyRescue::isRunning() const {
	return mRunning;
}


void StrategyRescue::stop() {
		MovementCommand mc;
	 	mc.type = 5; // MoveStop
	 	mCommandPub.publish(mc);
}

inline float StrategyRescue::EuclidDist(const Pose& p, const float& x, const float& y) const {
	return sqrt(pow(p.x - x, 2) + pow(p.y - y, 2));
}

void StrategyRescue::receive_pose(const amee::Pose::ConstPtr &msg){ 
	mPose = (*msg);
	mStrategyGoTo->receive_pose(msg);
}

void StrategyRescue::receive_graph(const amee::GraphMsg::ConstPtr &msg){}

void StrategyRescue::receive_mapper_event(const amee::MapperEvent::ConstPtr &msg){
	mStrategyGoTo->receive_mapper_event(msg);

}

void StrategyRescue::receive_movement_event(const amee::MovementEvent::ConstPtr &msg){
	mStrategyGoTo->receive_movement_event(msg);	
}

void StrategyRescue::receive_timerP1(const ros::TimerEvent &event) {}

void StrategyRescue::receive_timerP2(const ros::TimerEvent &event) {
	// TODO: empty node queue and move to exit
}

