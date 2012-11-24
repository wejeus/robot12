#include "Mapper.h"
#include <iostream>
#include <cmath>
#include "../MovementControl/MoveFollowWall.h"
#include "WallSegment.h"
#include "HorizontalWallSegment.h"

using namespace amee;

Mapper::Mapper() {
	mInitialized = false;
	mMappingState = PauseMapping;
	mNodeId = 0;
	mLastNodeId = -1;
}

Mapper::~Mapper() {
}

// Callback for IR distances
void Mapper::receive_distances(const IRDistances::ConstPtr &msg)
{
	mDistances.timestamp = msg->timestamp;
	mDistances.rightFront = msg->rightFront;
	mDistances.rightBack = msg->rightBack;
	mDistances.frontShortRange = msg->frontShortRange;
	mDistances.wheelRight = msg->wheelRight;
	mDistances.leftBack = msg->leftBack;
	mDistances.leftFront = msg->leftFront;
	mDistances.wheelLeft = msg->wheelLeft;
}

void Mapper::receive_tag(const amee::Tag::ConstPtr& msg) {
	// Map::Point p;
	// p.x = mPose.x;
	// p.y = mPose.y;
	// mTagPositions.push_back(p);
	if (sqrt((mPose.x - mLastTagPose.x) * (mPose.x - mLastTagPose.x) + (mPose.y - mLastTagPose.y) * (mPose.y - mLastTagPose.y)) > 0.04f) {
		int type = amee::Graph::NODE_TAG;
		addNode(type);
		mLastTagPose = mPose;	
	}
}

void Mapper::receiveOdometry(const amee::Odometry::ConstPtr &msg) {
	Odometry lastOdometry = mOdometry;
	mOdometry.distance = msg->distance;
	mOdometry.x = msg->x;
	mOdometry.y = msg->y;
	mOdometry.angle = msg->angle;
	mOdometry.angle = mOdometry.angle * M_PI / 180.0f; // transform to radians 
	mOdometry.timestamp = msg->timestamp;
	mOdometry.leftWheelDistance = msg->leftWheelDistance;
	mOdometry.rightWheelDistance = msg->rightWheelDistance;
	//std::cout << "Odometry " << mOdometry.x << " " << mOdometry.y << " " << mOdometry.angle << std::endl;


	/******* KALMAN: calculate odometry measurement here *********/

	/******* KALMAN: give it to Kalman filter *********/

	/******* KALMAN: reset mPose with the new pose from the Kalman filter *********/

	/******* KALMAN: delete the following lines / use them for calculate odmetry measurment *********/

	// set pose.theta based on angular change
	mPose.theta += mOdometry.angle - lastOdometry.angle;
	mPose.theta -= floor(mPose.theta / (2.0f * M_PI)) * 2.0f * M_PI; // move theta to [0,2PI]

	// Calc x & y with theta
	float leftDistance = mOdometry.leftWheelDistance - lastOdometry.leftWheelDistance;
	float rightDistance = mOdometry.rightWheelDistance - lastOdometry.rightWheelDistance;
	float distance = (leftDistance + rightDistance) / 2.0f;

	mPose.x += cos(mPose.theta) * distance;
	mPose.y += sin(mPose.theta) * distance;
	mPose.timestamp = mOdometry.timestamp;
}

void Mapper::receive_FollowWallState(const amee::FollowWallStates::ConstPtr &msg) {
	int type = 0;
	if (mMappingState != PauseMapping && mMappingState != Mapping) {
		return;
	}
	switch(msg->state) {
		case amee::MoveFollowWall::INIT:
			mLastNodeId = -1;
			mRotating = false;
			break;
		case amee::MoveFollowWall::ALIGN_TO_WALL_OUT:
			init(); //  only does sth if not initialized
			type = amee::Graph::NODE_NEXT_TO_WALL;
			addNode(type);
			mRotating = false;
			mMappingState = Mapping;
			break;
		case amee::MoveFollowWall::FOLLOW_WALL_OUT:
			if(mInitialized) addNode(amee::Graph::NODE_NEXT_TO_WALL);
			break;
		case amee::MoveFollowWall::ROTATE_LEFT_IN:
			type = amee::Graph::NODE_ROTATE_LEFT;
			mRotating = true;
			if (mInitialized) addNode(type);
			mMappingState = Mapping;
			break;
		case amee::MoveFollowWall::ROTATE_RIGHT_IN:
			mRotating = true;
			mMappingState = Mapping;
			type = amee::Graph::NODE_ROTATE_RIGHT;
			if (mInitialized) addNode(type);
			break;
		case amee::MoveFollowWall::FOLLOW_WALL_IN:
		case amee::MoveFollowWall::HANDLE_EVIL_WALLS_IN:
		case amee::MoveFollowWall::ALIGN_TO_FRONT_WALL_IN:
		case amee::MoveFollowWall::T_INTERSECTION_HANDLING_IN:
		// case amee::MoveFollowWall::MOVE_TAIL_IN:
		// case amee::MoveFollowWall::LOOK_FOR_BEGINNING_OF_WALL_IN:
			mRotating = false;
			mMappingState = Mapping;
			break;
		default:
			mMappingState = PauseMapping;
	}
}

void Mapper::addNode(int type) {
	mNodeId = mGraph.addNode(mPose, type);
	if (mLastNodeId != -1) {
		mGraph.addEdges(mNodeId, mLastNodeId);
	} 
	mLastNodeId = mNodeId;
	mNewNodes.push_back(mNodeId);
}

void Mapper::findEdges() {
	// first connect old nodes with new nodes
	for (std::list<int>::const_iterator i = mOldNodes.begin(), end = mOldNodes.end(); i != end; i++) {
		NodeMsg* startNode = mGraph.getNode(*i);
		Map::Point start(startNode->pose);
		for (std::list<int>::const_iterator j = mNewNodes.begin(), end = mNewNodes.end(); j != end; j++) {
			NodeMsg* endNode = mGraph.getNode(*j);
			Map::Point end(endNode->pose);
			if (mMap.isPathCollisionFree(start,end,0.02f,0.12f)) {
				mGraph.addEdges(*i,*j);
			}
		}
	}

	// now connect new nodes with new nodes
	for (std::list<int>::const_iterator i = mNewNodes.begin(), end = mNewNodes.end(); i != end; i++) {
		NodeMsg* startNode = mGraph.getNode(*i);
		Map::Point start(startNode->pose);
		for (std::list<int>::const_iterator j = i, end = mNewNodes.end(); j != end; j++) {
			NodeMsg* endNode = mGraph.getNode(*j);
			Map::Point end(endNode->pose);
			if (mMap.isPathCollisionFree(start,end,0.02f,0.12f)) {
				mGraph.addEdges(*i,*j);
			}
		}
	}

	// now add the new nodes to the list of old nodes
	mOldNodes.splice(mOldNodes.end(), mNewNodes);

}

void Mapper::init() {
	if (!mInitialized) {
		// mStartAngle = mPose.theta - (mPose.theta - floor(mPose.theta / 90.0f + 0.5f) * 90.0f);
		// mStartPos.x = mPose.x;
		// mStartPos.y = mPose.y;
		mPose.x = 0.0f;
		mPose.y = 0.0f;
		mPose.theta = 0.0f;
		mInitialized = true;
		Map::Measurement base;
		base.valid = false;
		base.pos.x = 0.0f;
		base.pos.y = 0.0f;
		mMeasurements.resize(4,base);
		mMappingState = PauseMapping;
		mVisualizeTimer = 0;
		mCleanTimer = 0;


		/******* KALMAN: initialize here *********/

	}
	
}

void Mapper::calculateMeasurements() {
	// calculate a position for each sensor reading
	float halfBase = IR_BASE_RIGHT / 2.0f;
	float robotR = ROBOT_RADIUS;
	// right back
	if (isValidDistance(mDistances.rightBack)) {
		// right back position of 0
		Map::Point p;
		p.x = -halfBase;
		p.y = -robotR;

		// set sensor position in global coordinates
		mMeasurements[RIGHT_BACK].sensorPos = p;
		mMeasurements[RIGHT_BACK].sensorRelativePos = p;
		mMeasurements[RIGHT_BACK].sensorPos.rotate(mPose.theta);
		mMeasurements[RIGHT_BACK].sensorPos = mMeasurements[RIGHT_BACK].sensorPos + mPose;
		mMeasurements[RIGHT_BACK].dist = mDistances.rightBack;

		// add distance vector (positive y is left of the robot, negative y right)
		p.y += -mDistances.rightBack;

		mMeasurements[RIGHT_BACK].valid = true;
		p.rotate(mPose.theta);
		mMeasurements[RIGHT_BACK].pos = p + mPose;
	} else {
		mMeasurements[RIGHT_BACK].valid = false;
	}

	// right front
	if (isValidDistance(mDistances.rightFront)) {
		// right front position of 0
		Map::Point p;
		p.x = halfBase;
		p.y = -robotR;

		// set sensor position in global coordinates
		mMeasurements[RIGHT_FRONT].sensorRelativePos = p;
		mMeasurements[RIGHT_FRONT].sensorPos = p;
		mMeasurements[RIGHT_FRONT].sensorPos.rotate(mPose.theta);
		mMeasurements[RIGHT_FRONT].sensorPos = mMeasurements[RIGHT_FRONT].sensorPos + mPose;
		mMeasurements[RIGHT_FRONT].dist = mDistances.rightFront;
		// add distance vector (positive y is left of the robot, negative y right)
		p.y += -mDistances.rightFront;

		mMeasurements[RIGHT_FRONT].valid = true;
		p.rotate(mPose.theta);
		mMeasurements[RIGHT_FRONT].pos = p + mPose;
	} else {
		mMeasurements[RIGHT_FRONT].valid = false;
	}
	// left front
	if (isValidDistance(mDistances.leftFront)) {
		// left front position of 0
		Map::Point p;
		p.x = halfBase;
		p.y = robotR;

		// set sensor position in global coordinates
		mMeasurements[LEFT_FRONT].sensorRelativePos = p;
		mMeasurements[LEFT_FRONT].sensorPos = p;
		mMeasurements[LEFT_FRONT].sensorPos.rotate(mPose.theta);
		mMeasurements[LEFT_FRONT].sensorPos = mMeasurements[LEFT_FRONT].sensorPos + mPose;
		mMeasurements[LEFT_FRONT].dist = mDistances.leftFront;
		// add distance vector (positive y is left of the robot, negative y right)
		p.y += mDistances.leftFront;

		mMeasurements[LEFT_FRONT].valid = true;
		p.rotate(mPose.theta);
		mMeasurements[LEFT_FRONT].pos = p + mPose;
	} else {
		mMeasurements[LEFT_FRONT].valid = false;
	}

	// left back
	if (isValidDistance(mDistances.leftBack)) {
		// left back position of 0
		Map::Point p;
		p.x = -halfBase;
		p.y = robotR;

		// set sensor position in global coordinates
		mMeasurements[LEFT_BACK].sensorRelativePos = p;
		mMeasurements[LEFT_BACK].sensorPos = p;
		mMeasurements[LEFT_BACK].sensorPos.rotate(mPose.theta);
		mMeasurements[LEFT_BACK].sensorPos = mMeasurements[LEFT_BACK].sensorPos + mPose;
		mMeasurements[LEFT_BACK].dist = mDistances.leftBack;

		// add distance vector (positive y is left of the robot, negative y right)
		p.y += mDistances.leftBack;

		mMeasurements[LEFT_BACK].valid = true;
		p.rotate(mPose.theta);
		mMeasurements[LEFT_BACK].pos = p + mPose;
	} else {
		mMeasurements[LEFT_BACK].valid = false;
	}
	
}

bool Mapper::isValidDistance(float dist) {
	return dist >= 0.0f && dist <= 0.15f;
}

void Mapper::setVisualizationPublisher(ros::Publisher pub) {
	vis_pub = pub;
}

void Mapper::setGraphPublisher(ros::Publisher pub) {
	graph_pub = pub;
}

void Mapper::setPosePublisher(ros::Publisher pub) {
	pose_pub = pub;
}

void Mapper::visualize() {
	if (mVisualizeTimer == 8) {

		mVisualizeTimer = 0;
	
		mMap.getVisualization(mVis);
		// mVis.robotPose.x = mPose.x;
		// mVis.robotPose.y = mPose.y;
		// mVis.robotPose.theta = mPose.theta;

		// for (unsigned int i = 0; i < mMeasurements.size(); ++i) {
		// 	if (mMeasurements[i].valid) {
		// 		Point p;
		// 		p.x = mMeasurements[i].pos.x;
		// 		p.y = mMeasurements[i].pos.y;
		// 		mVis.currentMeasurements.push_back(p);
		// 	}
		// }

		mVis.tags.resize(mTagPositions.size());

		for (unsigned int i = 0; i < mTagPositions.size(); ++i) {
			// Point p;
			// p.x = mTagPositions[i].x;
			// p.y = mTagPositions[i].y;
			mVis.tags[i].x = mTagPositions[i].x;
			mVis.tags[i].y = mTagPositions[i].y;
			//mVis.tags.push_back(p);
		}

		vis_pub.publish(mVis);

		graph_pub.publish(mGraph.getMessage());

	}
	++mVisualizeTimer;
}

void Mapper::cleanMap() {
	if (mCleanTimer == 5) {
		mCleanTimer = 0;
		Map::Point pos(mPose.x, mPose.y);
		mMap.reduceNumWalls(pos, 1.0f);
	}
	++mCleanTimer;
}

void Mapper::doMapping() {
	if (mInitialized) {

		calculateMeasurements();
		checkIfNextToWall(); 

		switch(mMappingState) {
			case Pause:
				// std::cout << "Mapper state: Pause" << std::endl;
				break;
			case PauseMapping:
				localize();
				break;
			case Mapping:
				// std::cout << "Mapper state: Mapping, left wall:" << mLeftNextToWall << " right wall: " << mRightNextToWall << std::endl;
				localize();
				mapping();
				break;
			case Localizing:
				// std::cout << "Mapper state: Localizing" << std::endl;
				localize();
				break;
		}

		visualize();
	}
}

void Mapper::localize() {
	if (!mRotating) {
		Map::MeasurementSet mset;
		mset.leftBack = mMeasurements[LEFT_BACK];
		mset.leftFront = mMeasurements[LEFT_FRONT];
		mset.rightBack = mMeasurements[RIGHT_BACK];
		mset.rightFront = mMeasurements[RIGHT_FRONT];
		amee::Pose newPose;
		float irDiff = fabs(mset.rightBack.dist - mset.rightFront.dist);
		bool rightOk = mRightNextToWall && (irDiff < 0.015f);
		irDiff = fabs(mset.leftBack.dist - mset.leftFront.dist);
		bool leftOk = mLeftNextToWall && (irDiff < 0.015f);
		mMap.localize(mPose, mset, newPose, leftOk, rightOk);
		mPose = newPose;
		calculateMeasurements(); // we have to recalculate these because mPose has changed
	}

	pose_pub.publish(mPose);
}

void Mapper::mapping() {
	int leftType = 0;
	int rightType = 0;
	followedWallDirection(leftType, rightType); // gets the type for new walls (Vertical, Horizontal or none)

	if (mMeasurements[RIGHT_BACK].valid) {
		mMap.addMeasurement(mMeasurements[RIGHT_BACK], rightType);
	} 

	if (mMeasurements[RIGHT_FRONT].valid) {
		mMap.addMeasurement(mMeasurements[RIGHT_FRONT], rightType);
	}

	if (mMeasurements[LEFT_BACK].valid) {
		mMap.addMeasurement(mMeasurements[LEFT_BACK], leftType);
	} 

	if (mMeasurements[LEFT_FRONT].valid) {
		mMap.addMeasurement(mMeasurements[LEFT_FRONT], leftType);
	} 

	cleanMap();
	// mMap.print();
}

void Mapper::followedWallDirection(int& left, int& right) {
	float theta = mPose.theta;
	left = WallSegment::NONE;
	right = WallSegment::NONE;
	if (mRotating) { // maybe remove again
		return;
	}
	if (mMappingState == Mapping) {
		if ((fabs(theta) <= M_PI/5.0f) || (fabs(theta - M_PI) <= M_PI/5.0f)) {
			if (mRightNextToWall) {
				right = WallSegment::HORIZONTAL;
			}
			if (mLeftNextToWall) {
				left = WallSegment::HORIZONTAL;
			}
		} 

		if ((fabs(theta - M_PI/2.0f) <= M_PI/5.0f) || (fabs(theta - 3.0f/2.0f*M_PI) <= M_PI/5.0f)) {
			if (mRightNextToWall) {
				right = WallSegment::VERTICAL;
			}
			if (mLeftNextToWall) {
				left = WallSegment::VERTICAL;
			}
		}
	}
}	

 void Mapper::checkIfNextToWall() {
        bool rF = mMeasurements[RIGHT_FRONT].valid;
        bool rB = mMeasurements[RIGHT_BACK].valid;
        if (!rF) {
            mRightWallStartDist = mOdometry.distance;
        }
        float wallLength = mOdometry.distance - mRightWallStartDist;
        // std::cout << "wall length is " << wallLength << " currentDist is " << mOdometry.distance << " start dist: " << mRightWallStartDist << std::endl;
        mRightNextToWall = rB && (wallLength >= IR_BASE_RIGHT);

        bool lF = mMeasurements[LEFT_FRONT].valid;
        bool lB = mMeasurements[LEFT_BACK].valid;

        if (!lF) {
        	mLeftWallStartDist = mOdometry.distance;
        }
        wallLength = mOdometry.distance - mLeftWallStartDist;
        mLeftNextToWall = lB && (wallLength >= IR_BASE_RIGHT); // TODO check whether IR base left is equal to IR base right
    }

void Mapper::receive_MapperCommand(const amee::MapperCommand::ConstPtr &msg) {
	switch (msg->type) {
		case PauseCommand:
			mMappingState = Pause;
			break;
		case MapCommand:
			mMappingState = PauseMapping;
			break;
		case FindEdgesCommand:
			findEdges();
			break;
		case LocalizeCommand:
			mMappingState = Localizing;
			break;
		case FindEdgesToUnexploredCommand:
			//TODO
			break;
		default:
			std::cout << "Unknown mapper command received." << std::endl;
	}
}


void mapTest(ros::Publisher& vispub) {
	Map map;

	Map::Point p;
	p.x = 0;
	p.y = 0;
	for (int i = 0; i < 100; ++i) {
		p.x += 0.01f; 
		// map.addMeasurement(p, WallSegment::HORIZONTAL);
	}

	for (int i = 0; i < 100; ++i) {
		p.y += 0.01f;
		// map.addMeasurement(p, WallSegment::VERTICAL); 
	}

	for (int i = 0; i < 10; ++i) {
		p.x -= 0.01f;
		// map.addMeasurement(p, WallSegment::HORIZONTAL);
	}

	for (int i = 0; i < 10; ++i) {
		p.y -= 0.01f;
		// map.addMeasurement(p, WallSegment::VERTICAL);
	}

	MapVisualization vis;
	map.getVisualization(vis);
	vispub.publish(vis);

}

void testWallSegment() {
	Map::Point p;
	HorizontalWallSegment* wall = new HorizontalWallSegment(p);
	Map::Point sensor(0.0f,-0.12f);
	Map::Point intersection;
	for (int i=0; i < 100;++i) {
		p.x += 0.01f;
		sensor.x += 0.01f;

		wall->mapMeasurement(sensor,p , intersection);
		std::cout << "measurement hits wall at " << intersection.x << " " << intersection.y << std::endl;
	}

	p.x = 10.0f;
	p.y = 2.0f;
	float t;
	std::cout << "Point " << p.x << " " << p.y << " belongs to wall: " << wall->belongsToWall(sensor,p, intersection,t) << std::endl;

	p.x = 0.86f;
	p.y = -0.04f;
	std::cout << "Point " << p.x << " " << p.y << " belongs to wall: " << wall->belongsToWall(sensor,p, intersection,t) << std::endl;


	p.x = 0.86f;
	p.y = 0.04f;
	std::cout << "Point " << p.x << " " << p.y << " belongs to wall: " << wall->belongsToWall(sensor,p, intersection,t) << std::endl;	
	
	p.x = 0.76f;
	p.y = -0.02f;
	std::cout << "Point " << p.x << " " << p.y << " belongs to wall: " << wall->belongsToWall(sensor,p, intersection, t) << std::endl;

	delete wall;
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "MapperNode");
	ros::NodeHandle n;

	// create the mapper
	Mapper mapper;
	
	// create subscriber for distances
	ros::Subscriber	dist_sub = n.subscribe("/amee/sensors/irdistances", 100, &Mapper::receive_distances, &mapper);
	ros::Subscriber odo_sub = n.subscribe("/amee/motor_control/odometry", 100, &Mapper::receiveOdometry, &mapper);
	ros::Subscriber state_sub = n.subscribe("/amee/follow_wall_states",10, &Mapper::receive_FollowWallState, &mapper);
	ros::Subscriber tag_sub = n.subscribe("/amee/tag",10, &Mapper::receive_tag, &mapper);
	ros::Subscriber command_sub = n.subscribe("/amee/map/mapper_commands",10, &Mapper::receive_MapperCommand, &mapper);

	ros::Publisher pose_pub = n.advertise<amee::Pose>("/amee/pose",5);
  	ros::Publisher marker_pub = n.advertise<amee::MapVisualization>("/amee/map/visualization", 10);
  	ros::Publisher graph_pub = n.advertise<amee::GraphMsg>("/amee/map/graph",10);

    mapper.setVisualizationPublisher(marker_pub);
    mapper.setGraphPublisher(graph_pub);
    mapper.setPosePublisher(pose_pub);

	ros::Rate loop_rate(30); //30
	

	// testWallSegment();
	while(ros::ok()){
		
		// go to sleep for a short while
		loop_rate.sleep();

		// call all callbacks
		ros::spinOnce();
		
		// // map!
		mapper.doMapping();
		// mapTest(marker_pub);
	}

	return 0;
}
