#include <iostream>
#include "StrategyGoTo.h"
#include "amee/MovementCommand.h"
#include <cmath>

using namespace amee;

StrategyGoTo::StrategyGoTo(ros::Publisher& pub, ros::Publisher &phaseInfo) {
	mPub = pub;
	mPhaseInfo = phaseInfo;
	mRunning = false;
}

StrategyGoTo::~StrategyGoTo() {
}

void StrategyGoTo::init(const StrategyData& data) {
	//get the current graph
	//find the path to a specific position
	mRunning = true;
}

/**
 * Initialize this function to go back to the starting position of the maze.
 */
void StrategyGoTo::init(const StrategyData &data, const amee::GraphMsg::ConstPtr& graphMsg) {
	std::cout << "initializing GO OUT.." << std::endl;
	mStrategyData = data;
	mGraph = graphMsg;

	PathFinderAlgo pfa;
	std::vector<Pose> v = pfa.findShortestPath(mGraph, mStrategyData.x, mStrategyData.y, 0);
	if(v.empty())
		std::cout << "PathFinderAlgo returnd empty path list!" << std::endl;

	mPath.empty();
	for(std::vector<Pose>::iterator it = v.begin(); it != v.end(); ++it) {
		mPath.push(*it);
	}

	mRunning = true;
	std::cout << "initializing done!" << std::endl;
}

/**
 * Initialize this function to go to a specific position.
 */
void StrategyGoTo::init(const StrategyData &data, const amee::GraphMsg::ConstPtr& graphMsg, const float& to_x, const float& to_y) {
	mStrategyData = data;
	mGraph = graphMsg;


	PathFinderAlgo pfa;
	std::vector<Pose> v = pfa.findShortestPath(mGraph, mStrategyData.x, mStrategyData.y, 0);
	if(v.empty())
		std::cout << "PathFinderAlgo returnd empty path list!" << std::endl;

	mPath.empty();
	for(std::vector<Pose>::iterator it = v.begin(); it != v.end(); ++it) {
		mPath.push(*it);
	}

	mRunning = true;
}

bool StrategyGoTo::isRunning() const {
	return mRunning;
}

void StrategyGoTo::doControl(const StrategyData& data) {
	mStrategyData = data;

	//if there are still nodes in our path
	if(mPath.empty()){
		std::cout << "we have reached our final destination." << std::endl;
		// mPhaseInfo.publish();
		mRunning = false;
	}else{

		Pose curP = mPath.front();

		//if current position is reach pop the one in the front
		if(EuclidDist(curP, mStrategyData.x, mStrategyData.y) < EUCLIDEAN_POSITION_DISTANCE) {
			mPath.pop();
			std::cout << "Going to the next node in the path..." << std::endl;
		}
		
		MovementCommand mc;
        mc.type = 3; mc.x = curP.x; mc.y = curP.y;

        mPub.publish(mc);
	}

}

inline float StrategyGoTo::EuclidDist(const Pose& p, const float& x, const float& y) const {
	return sqrt(pow(p.x - x, 2) + pow(p.y - y, 2));
}

