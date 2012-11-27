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
	mState = MoveCoordinate;

	int currentNodeId = mGraph.getIDFromPose(pose.x, pose.y, pose.theta);

	if (currentNodeId == -1) {
		std::cout << "ERROR: We are not close to a node! Can't plan path." << std::endl;
		mRunning = false;
		return;
	} 

	PathFinderAlgo pfa;
	std::vector<NodeMsg> v;
	bool pathFound = pfa.findShortestPath(mGraph, v, currentNodeId, id);
	if(v.empty() || !pathFound) {
		std::cout << "PathFinderAlgo returnd empty path list!" << std::endl;
		mRunning = false;
		return;
	}

	Path pathMsg;

	mPath.empty();
	int i = 0;
	std::cout << "Path: " << std::endl;
	NodeMsg startNode = *(mGraph.getNode(currentNodeId));
	mPath.push(startNode); // push the start node twice on the queue, that way we will move there first
	for(std::vector<NodeMsg>::iterator it = v.begin(); it != v.end(); ++it) {
		mPath.push(*it);
		std::cout << i << ": " << (*it).pose.x << " " << (*it).pose.y << " id: " << (*it).nodeID << std::endl;
		++i;
		pathMsg.nodeIDs.push_back((*it).nodeID);
	}

	mPathPub.publish(pathMsg);
	mRunning = true;
	moveToNextWaypoint();

	

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
	if (isRunning()) {


	switch (mState) {
		case Rotate:
			std::cout << "Rotate Done" << std::endl;
			if (mPath.front().type == Graph::NODE_NEXT_TO_WALL) {
				std::cout << "We can align, so align!" << std::endl;
				MovementCommand mc;
				mc.type = MovementControl::TYPE_ALIGN_TO_WALL;
				mCommandPub.publish(mc);
				mState = Align;
			} else {
				std::cout << "Can't align, so start followWall" << std::endl;
				startFollowWall();
			}
			break;
		case Align:
			std::cout << "Align done" << std::endl;
			startFollowWall();
			break;
		case FollowWall: {
				std::cout << "FollowWall node reached" << std::endl;
				NodeMsg reachedWaypoint = mPath.front();
		 		mPath.pop();
				if(mPath.empty()){
		 			std::cout << "we have reached our final destination." << std::endl;
		 			std_msgs::Int32 msg;
		 			msg.data = 1;
		 			mPhaseInfo.publish(msg);
		 			mRunning = false;
		 			stop();
		 		} else {	
		 			std::cout << "switching to move coordinate" << std::endl;
		 			NodeMsg waypoint = mPath.front();
		 			if (waypoint.nodeID - reachedWaypoint.nodeID != 1) { // if we can not follow a wall to get to the next waypoint
		 				startMoveCoordinate(waypoint.pose);
					}
		 		}
		 	}
			break;
		case MoveCoordinate: {
				std::cout << "MoveCoordinate done" << std::endl;
				NodeMsg reachedWaypoint = mPath.front();
	 			mPath.pop();
	 			if(mPath.empty()){
	 				std::cout << "End of path, do final rotation" << std::endl;
	 				mState = FinalRotate;
	 				MovementCommand mc;
					mc.angle = (reachedWaypoint.pose.theta - mPose.theta) * 180.0f / M_PI;
					mc.type = MovementControl::TYPE_MOVE_ROTATE;
					mCommandPub.publish(mc);
				} else {
		 			NodeMsg waypoint = mPath.front();
					if (waypoint.nodeID - reachedWaypoint.nodeID != 1) { // if we can not follow a wall to get to the next waypoint
						std::cout << "continue move coordinate" << std::endl;
						startMoveCoordinate(waypoint.pose);
					} else {
						std::cout << "we can do follow wall, so rotate first and then follow wall" << std::endl;
						mState = Rotate;
						MovementCommand mc;
						mc.type = MovementControl::TYPE_MOVE_ROTATE;
						mc.angle = (waypoint.pose.theta - mPose.theta) * 180.0f / M_PI;
						mCommandPub.publish(mc);
					}
				}
			}
			break;	
		case FinalRotate:
				std::cout << "we have reached our final destination." << std::endl;
	 			std_msgs::Int32 msg;
	 			msg.data = 1;
	 			mPhaseInfo.publish(msg);
	 			mRunning = false;
	 			stop();
	 		break;
		}
	}
	}

void StrategyGoTo::startMoveCoordinate(Pose& pose) {
	MovementCommand mc;
	mc.type = MovementControl::TYPE_MOVE_COORDINATE; 
	mc.x = pose.x - mPose.x; 
	mc.y = pose.y - mPose.y; //moveCoordinate 
	float tx = cos(mPose.theta) * mc.x + sin(mPose.theta) * mc.y;	
	float ty = -sin(mPose.theta) * mc.x + cos(mPose.theta) * mc.y;
	mc.x = tx;
	mc.y = ty;
	std::cout << "Do MoveCoordinate to get to next waypoint mc: " << mc.x << " " << mc.y << std::endl;
	mCommandPub.publish(mc);
	mState = MoveCoordinate;
}

void StrategyGoTo::startFollowWall() {
	MovementCommand mc;
	mc.type = MovementControl::TYPE_FOLLOW_WALL;
	std::cout << "Do MoveFollowWall to get to next waypoint" << std::endl;
	mCommandPub.publish(mc);
	mState = FollowWall;	
}

void StrategyGoTo::stop() {
	MovementCommand mc;
 	mc.type = MovementControl::TYPE_STOP; // MoveStop
 	mCommandPub.publish(mc);
}

inline float StrategyGoTo::EuclidDist(const Pose& p, const float& x, const float& y) const {
	return sqrt(pow(p.x - x, 2) + pow(p.y - y, 2));
}

void StrategyGoTo::receive_pose(const amee::Pose::ConstPtr &msg){ 
	mPose = (*msg);
}

void StrategyGoTo::receive_graph(const amee::GraphMsg::ConstPtr &msg){
	mGraph = msg;
}

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
		std::cout << "MoveCoordinate completed" << std::endl;
		moveToNextWaypoint();
	} else if ((msg->type == MovementControl::MOVEMENT_EVENT_TYPE_DONE_ROTATING) && ((mState == Rotate) ||(mState == FinalRotate))) {
		std::cout << "Rotating completed" << std::endl;
		moveToNextWaypoint();
	} else if ((msg->type == MovementControl::MOVEMENT_EVENT_TYPE_DONE_ALIGNING_WALL) && mState == Align) {
		std::cout << "Aligning completed" << std::endl;
		moveToNextWaypoint();
	} else if ((msg->type == MovementControl::MOVEMENT_EVENT_TYPE_FAILED_ALIGNING_WALL) && mState == Align) {
		std::cout << "DAMN, COULDN't ALIGN. IMPLEMENT HANDLING OF THIS CASE!!!!" << std::endl;
	} else {
		std::cout << "Unknown move completed" << std::endl;
	}
}


