#include <iostream>
#include "StrategyGoTo.h"
#include "../Mapper/Mapper.h"
#include "../MovementControl/MovementControl.h"
#include "amee/MovementCommand.h"
#include "amee/Path.h"
#include <cmath>

using namespace amee;

StrategyGoTo::StrategyGoTo(ros::Publisher& pub, ros::Publisher &phaseInfo, ros::Publisher &pathPub) {
	mCommandPub = pub;
	mPhaseInfo = phaseInfo;
	mPathPub = pathPub;
	mRunning = false;
	// mRestartFollowingWall = true;
	// mFollowWallPossible = false;
}

StrategyGoTo::~StrategyGoTo() {
}

// void StrategyGoTo::init(const StrategyData& data) {
// 	//get the current graph
// 	//find the path to a specific position
// 	mRunning = true;
// }

/**
 * Initialize this function to go back to the starting position of the maze.
 */
// void StrategyGoTo::init(const StrategyData &data, const amee::GraphMsg::ConstPtr& graphMsg) {
// 	std::cout << "initializing GO OUT.." << std::endl;
// 	mStrategyData = data;
// 	mGraph = graphMsg;

// 	PathFinderAlgo pfa;
// 	std::vector<NodeMsg> v = pfa.findShortestPath(mGraph, mStrategyData.x, mStrategyData.y, 0);
// 	if(v.empty())
// 		std::cout << "PathFinderAlgo returnd empty path list!" << std::endl;

// 	mPath.empty();
// 	for(std::vector<NodeMsg>::iterator it = v.begin(); it != v.end(); ++it) {
// 		mPath.push(*it);
// 	}

// 	mRunning = true;
// 	std::cout << "initializing done!" << std::endl;
// }

/**
 * Initialize this function to go to a specific position.
 */
// void StrategyGoTo::init(const StrategyData &data, const amee::GraphMsg::ConstPtr& graphMsg, const float& to_x, const float& to_y) {
// 	mStrategyData = data;
// 	mGraph = graphMsg;

// 	PathFinderAlgo pfa;
// 	std::vector<NodeMsg> v = pfa.findShortestPath(mGraph, mStrategyData.x, mStrategyData.y, 0);
// 	if(v.empty())
// 		std::cout << "PathFinderAlgo returned empty path list!" << std::endl;

// 	mPath.empty();
// 	for(std::vector<NodeMsg>::iterator it = v.begin(); it != v.end(); ++it) {
// 		mPath.push(*it);
// 	}

// 	mRunning = true;
// }

void StrategyGoTo::init(const amee::Pose &pose, const amee::GraphMsg::ConstPtr& graphMsg, const unsigned int& id) {
	std::cout << "StrategyGoTo init, move to node " << id << std::endl;
	mGraph = graphMsg;
	mFollowingWall = false;

	int currentNodeId = mGraph.getIDFromPose(pose.x, pose.y, pose.theta);

	if (currentNodeId == -1) {
		std::cout << "ERROR: We are not close to a node! Can't plan path." << std::endl;
		mRunning = false;
		return;
	} // TODO else move to closest node

	PathFinderAlgo pfa;
	std::vector<NodeMsg> v = pfa.findShortestPath(mGraph, currentNodeId, id);
	if(v.empty())
		std::cout << "PathFinderAlgo returnd empty path list!" << std::endl;

	// Path pathMsg;
	// pathMsg.path = v;
	// mPathPub.publish(pathMsg);

	mPath.empty();
	int i = 0;
	std::cout << "Path: " << std::endl;
	for(std::vector<NodeMsg>::iterator it = v.begin(); it != v.end(); ++it) {
		mPath.push(*it);
		std::cout << i << ": " << (*it).pose.x << " " << (*it).pose.y << " id: " << (*it).nodeID << std::endl;
	}

	moveToNextWaypoint();

	mRunning = true;

}

bool StrategyGoTo::isRunning() const {
	return mRunning;
}

// void StrategyGoTo::doControl(const StrategyData& data) {
	// // std::cout << "doControl" << std::endl;
	// mStrategyData = data;

	// //if there are still nodes in our path
	// if(mPath.empty()){
	// 	std::cout << "we have reached our final destination." << std::endl;
	// 	std_msgs::Int32 msg;
	// 	msg.data = 1;
	// 	mPhaseInfo.publish(msg);
	// 	mRunning = false;
	// 	MovementCommand mc;
	// 	mc.type = 5; // MoveStop
	// 	mCommandPub.publish(mc);
	// }else{

	// 	NodeMsg waypoint = mPath.front();
		
	// 	//if waypoint is reached pop the one in the front
	// 	if(EuclidDist(waypoint.pose, mStrategyData.x, mStrategyData.y) < EUCLIDEAN_POSITION_DISTANCE) {
	// 		unsigned int mLastNodeID = waypoint.nodeID;
	// 		mPath.pop();
	// 		waypoint = mPath.front();
	// 		MovementCommand mc;
	// 		if (waypoint.nodeID - mLastNodeID == 1) { // checks if we can follow a wall to get to the next

	// 			mc.type = 4; // moveFollowWall
	// 			std::cout << "Do MoveFollowWall to get to next waypoint" << std::endl;
	// 		}  else {

	// 			mc.type = 3; mc.x = waypoint.pose.x - data.x; mc.y = waypoint.pose.y - data.y; //moveCoordinate 
	// 			float tx = cos(data.theta) * mc.x + sin(data.theta) * mc.y;	
	// 			float ty = -sin(data.theta) * mc.x + sin(data.theta) * mc.y;
	// 			mc.x = tx;
	// 			mc.y = ty;
	// 			std::cout << "Do MoveCoordinate to get to next waypoint" << std::endl;
	// 		}
	// 		mCommandPub.publish(mc);
	// 		std::cout << "Going to the next node in the path..." << std::endl;
	// 	}
	// }

// }

void StrategyGoTo::moveToNextWaypoint() {
	//if there are still nodes in our path
	if(mPath.empty()){
	 	std::cout << "we have reached our final destination." << std::endl;
	 	std_msgs::Int32 msg;
	 	msg.data = 1;
	 	mPhaseInfo.publish(msg);
	 	mRunning = false;
	 	stop();
	}else{

	 	NodeMsg reachedWaypoint = mPath.front();
		mPath.pop();
		NodeMsg waypoint = mPath.front();
		 	
// 		unsigned int mLastNodeID = waypoint.nodeID;

// 		waypoint = mPath.front();
		MovementCommand mc;
		if (waypoint.nodeID - reachedWaypoint.nodeID == 1) { // checks if we can follow a wall to get to the next

			mc.type = 4; // moveFollowWall
			std::cout << "Do MoveFollowWall to get to next waypoint" << std::endl;
			if (!mFollowingWall) {
				mCommandPub.publish(mc);		
			}
			mFollowingWall = true;
		} else {
			mFollowingWall = false;
			mc.type = 3; 
			mc.x = waypoint.pose.x - mPose.x; 
			mc.y = waypoint.pose.y - mPose.y; //moveCoordinate 
			float tx = cos(mPose.theta) * mc.x + sin(mPose.theta) * mc.y;	
			float ty = -sin(mPose.theta) * mc.x + cos(mPose.theta) * mc.y;
			mc.x = tx;
			mc.y = ty;
			std::cout << "Do MoveCoordinate to get to next waypoint mc: " << mc.x << " " << mc.y << std::endl;
			mCommandPub.publish(mc);
		}
		
		std::cout << "Going to the next node in the path..." << std::endl;

	}
}

void StrategyGoTo::stop() {
		MovementCommand mc;
	 	mc.type = 5; // MoveStop
	 	mCommandPub.publish(mc);
}

inline float StrategyGoTo::EuclidDist(const Pose& p, const float& x, const float& y) const {
	return sqrt(pow(p.x - x, 2) + pow(p.y - y, 2));
}

void StrategyGoTo::receive_pose(const amee::Pose::ConstPtr &msg){ 
	mPose = (*msg);
}

void StrategyGoTo::receive_graph(const amee::GraphMsg::ConstPtr &msg){}

void StrategyGoTo::receive_mapper_event(const amee::MapperEvent::ConstPtr &msg){
	if (msg->type == Mapper::NodeReached) {
		std::cout << "Node reached" << std::endl;
		moveToNextWaypoint();
	} else if (msg->type == Mapper::UnknownNode) {
		std::cout << "HELP!!! UnknownNode" << std::endl;
	}
}

void StrategyGoTo::receive_movement_event(const amee::MovementEvent::ConstPtr &msg){
	if (msg->type == MovementControl::MOVEMENT_EVENT_TYPE_OBSTICLE_IN_FRONT) {
		std::cout << "HELP!!! Obstacle detected!!!" << std::endl;
	} else if (msg->type == MovementControl::MOVEMENT_EVENT_TYPE_DONE_MOVING_COORDINATE) {
		std::cout << "Movement completed" << std::endl;
		moveToNextWaypoint();
	}
}


