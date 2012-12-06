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

void StrategyGoTo::init(const amee::Pose &pose, const amee::GraphMsg::ConstPtr& graphMsg, const unsigned int& id) {
	std::cout << "StrategyGoTo init, move to node " << id << std::endl;
	mGraph = graphMsg;
	mTargetId = id;
	mPose = pose;

	reinit();
}

void StrategyGoTo::reinit() {
	mState = Start;
	mUnknownNodeCounter = 0;
	int currentNodeId = mGraph.getIDFromPose(mPose.x, mPose.y, mPose.theta);

	if (currentNodeId == -1) {
		std::cout << "ERROR: We are not close to a node! Can't plan path." << std::endl;
		mRunning = false;
		return;
	} 

	PathFinderAlgo pfa;
	std::vector<NodeMsg> v;
	bool pathFound = pfa.findShortestPath(mGraph, v, currentNodeId, mTargetId);
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
	// mPath.push(startNode); // push the start node twice on the queue, that way we will move there first
	pathMsg.nodeIDs.clear();
	int lastId = startNode.nodeID;
	for(std::vector<NodeMsg>::iterator it = v.begin(); it != v.end(); ++it) {
		// int curId = (*it).nodeID;
		// if (curId - lastId != 1) {
		
		mPath.push(*it);
		std::cout << i << ": " << (*it).pose.x << " " << (*it).pose.y << " id: " << (*it).nodeID << std::endl;
		
			
		// } else {
			// std::cout << i << ": continue follow wall." << std::endl;
		// }
		pathMsg.nodeIDs.push_back((*it).nodeID);	
		// lastId = curId;
		++i;
	}
	// NodeMsg last = v[v.size() - 1];
	// if (mPath.back().nodeID != last.nodeID) {
		// mPath.push(last);
		// pathMsg.nodeIDs.push_back(last.nodeID);	
	// }

	mPath.pop(); // we assume we are already at the start position
	mPathPub.publish(pathMsg);
	mRunning = true;
	moveToNextWaypoint();
}

bool StrategyGoTo::isRunning() const {
	return mRunning;
}

void StrategyGoTo::moveToNextWaypoint() {
	switch (mState) {
		case Start:
			startFollowWall();
			break;
		case Rotate:
			std::cout << "Rotate Done" << std::endl;
			// if (mPath.front().type == Graph::NODE_NEXT_TO_WALL) {
			// 	std::cout << "We can align, so align!" << std::endl;
			// 	MovementCommand mc;
			// 	mc.type = MovementControl::TYPE_ALIGN_TO_WALL;
			// 	mCommandPub.publish(mc);
			// 	mState = Align;
			// } else {
			// 	std::cout << "Can't align, so start followWall" << std::endl;
				startFollowWall();
			// }
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
		 			NodeMsg waypoint = mPath.front();
		 			if ((reachedWaypoint.nodeID >= waypoint.nodeID) || (waypoint.nodeID - reachedWaypoint.nodeID != 1)) { // if we can not follow a wall to get to the next waypoint
		 				std::cout << "switching to move coordinate" << std::endl;
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
					mc.angle = getAngleChange(mPose.theta, reachedWaypoint.pose.theta);
					mc.type = MovementControl::TYPE_MOVE_ROTATE;
					mCommandPub.publish(mc);
				} else {
		 			NodeMsg waypoint = mPath.front();
					if ((reachedWaypoint.nodeID >= waypoint.nodeID) || (waypoint.nodeID - reachedWaypoint.nodeID != 1)) { // if we can not follow a wall to get to the next waypoint
						std::cout << "continue move coordinate" << std::endl;
						startMoveCoordinate(waypoint.pose);
					} else {
						std::cout << "we can do follow wall, so rotate first and then follow wall" << std::endl;
						mState = Rotate;
						MovementCommand mc;
						mc.type = MovementControl::TYPE_MOVE_ROTATE;
						mc.angle = getAngleChange(mPose.theta, waypoint.pose.theta);
						mCommandPub.publish(mc);
					}
				}
			}
			break;	
		case FinalRotate: {
				std::cout << "we have reached our final destination." << std::endl;
	 			std_msgs::Int32 msg;
	 			msg.data = 1;
	 			mPhaseInfo.publish(msg);
	 			mRunning = false;
	 			stop();
	 		}
	 		break;
	 	case CollisionRecovery:
	 		// we recovered from the "collision" and now continue the path execution as if nothing happened.
	 		mState = MoveCoordinate;
	 		moveToNextWaypoint();
	 	break;
	}
}

void StrategyGoTo::handleCollision() {
	NodeMsg waypointToReach =  mPath.front();
	// if (EuclidDist(mPose, waypointToReach.pose) <= NODE_REACHED_DISTANCE) {
		mState = CollisionRecovery;
		MovementCommand mc;
		mc.type = MovementControl::TYPE_COLLISION_RECOVERY;
		mCommandPub.publish(mc);
	// } else {
		// Path execution failed!
		// std::cout << "Collision, but we are not close to next waypoint, path execution faild." << std::endl;
	// }

}

float StrategyGoTo::getAngleChange(float from, float to) {
	if (from >= to) {
		float rotateLeft = 2.0f * M_PI - from + to;
		float rotateRight = from - to;
		if (rotateLeft < rotateRight) {
			return rotateLeft * 180.0f / M_PI;
		} 
		return -rotateRight * 180.0f / M_PI;
	} else {
		float rotateLeft = to - from;
		float rotateRight = from + 2.0f * M_PI - to;
		if (rotateLeft < rotateRight) {
			return rotateLeft * 180.0f / M_PI;
		}
		return -rotateRight * 180.0f / M_PI;
	}
	return 0.0f;
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

inline float StrategyGoTo::EuclidDist(const Pose& p1, const Pose& p2) const {
	return sqrt(pow(p1.x - p2.x, 2) + pow(p1.y - p2.y, 2));
}

void StrategyGoTo::receive_pose(const amee::Pose::ConstPtr &msg){ 
	mPose = (*msg);
	// there are no events for tag nodes, so we check here based on our pose if we reached it
	if (isRunning() && !mPath.empty() && (mPath.front().type == Graph::NODE_TAG)) { 
		if (EuclidDist(mPose, mPath.front().pose) <= 0.08f) {
			moveToNextWaypoint();
		}
	}
}

void StrategyGoTo::receive_graph(const amee::GraphMsg::ConstPtr &msg){
	mGraph = msg;
}

void StrategyGoTo::receive_mapper_event(const amee::MapperEvent::ConstPtr &msg){
	if (isRunning()) {
		if (msg->type == Mapper::NodeReached) {
			mUnknownNodeCounter = 0;
			std::cout << "Node " << msg->nodeID << " reached" << std::endl;
			if (mPath.front().nodeID == msg->nodeID) {
				moveToNextWaypoint();	
			} else  {
				// std::cout << "The node has id" << msg->nodeID << " but we want to reach node " << mPath.front().nodeID << std::endl;
				// int currentNode = msg->nodeID;
				// int waypoint = mPath.front().nodeID;
				// if (abs(currentNode - waypoint) >= 2) {
				// 	std::cout << "We are somewhere where we don't want to be, replan the path" << std::endl;
				// 	reinit();
				// }
			}
			
			
		} else if (msg->type == Mapper::UnknownNode) {
			std::cout << "Unknown node" << std::endl;			
			// if (mUnknownNodeCounter >= 3) {
			// 	std::cout << "Third UnknownNode, replan the path" << std::endl;
			// 	stop();
			// 	reinit();	
			// } else {
			// 	++mUnknownNodeCounter;	
			// }			
		}
	}
}

void StrategyGoTo::receive_movement_event(const amee::MovementEvent::ConstPtr &msg){
	if (isRunning()) {
		if (msg->type == MovementControl::MOVEMENT_EVENT_TYPE_OBSTICLE_IN_FRONT) {
			std::cout << "!!!Obstacle detected!!! Try handling it." << std::endl;
			handleCollision();
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
		} else if ((msg->type == MovementControl::MOVEMENT_EVENT_TYPE_RECOVERY_DONE) && mState == CollisionRecovery) {
			std::cout << "Collision recovery done, continue path execution!" << std::endl;
			moveToNextWaypoint();
		} else {
			std::cout << "Unknown move completed" << std::endl;
		}
	}
}


