#include <iostream>
#include "StrategyGetOut.h"
#include "amee/MovementCommand.h"
#include <cmath>

using namespace amee;

StrategyGetOut::StrategyGetOut(ros::Publisher& pub) {
	mPub = pub;
	mRunning = false;
	mGoingOut = false;
}

StrategyGetOut::~StrategyGetOut() {
}

void StrategyGetOut::init(const StrategyData& data) {
	//get the current graph
	//find the path to a specific position
	mRunning = true;
}

void StrategyGetOut::init(const StrategyData &data, amee::GraphMsg::ConstPtr& graphMsg) {
	mStrategyData = data;
	mGraph = graphMsg;

	PathFinderAlgo pfa;
	std::vector<Pose> v = pfa.findShortestPath(mGraph, mStrategyData.x, mStrategyData.y, 0);

	mPath.empty();
	for(std::vector<Pose>::iterator it = v.begin(); it != v.end(); ++it) {
		mPath.push(*it);
	}

	mRunning = true;
}

bool StrategyGetOut::isRunning() const {
	return mRunning;
}

void StrategyGetOut::doControl(const StrategyData& data) {
	mStrategyData = data;

	//if position not reached 
	if(mPath.empty())
		mRunning = false;
	else{

		Pose curP = mPath.front();

		//if current position is reach pop the one in the front
		if(EuclidDist(curP, mStrategyData.x, mStrategyData.y) < EUCLIDEAN_POSITION_DISTANCE)
			mPath.pop();
		
		MovementCommand mc;
        mc.type = 3; mc.x = curP.x; mc.y = curP.y;

        mPub.publish(mc);
	}

}

inline float StrategyGetOut::EuclidDist(const Pose& p, const float& x, const float& y) const {
	return sqrt(pow(p.x - x, 2) + pow(p.y - y, 2));
}

