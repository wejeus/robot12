#include "Mapper.h"
#include <iostream>
#include <cmath>
#include "../MovementControl/MoveFollowWall.h"
#include "WallSegment.h"

using namespace amee;

Mapper::Mapper(ros::Publisher pub) {
	map_pub = pub;
	mInitialized = false;
	mMappingState = Pause;
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
	Map::Point p;
	p.x = mPose.x;
	p.y = mPose.y;
	mTagPositions.push_back(p);
}

void Mapper::receiveOdometry(const amee::Odometry::ConstPtr &msg) {
	Odometry lastOdometry = mOdometry;
	mOdometry.x = msg->x;
	mOdometry.y = msg->y;
	mOdometry.angle = msg->angle;
	mOdometry.angle = mOdometry.angle * M_PI / 180.0f; // transform to radians 
	mOdometry.timestamp = msg->timestamp;
	mOdometry.leftWheelDistance = msg->leftWheelDistance;
	mOdometry.rightWheelDistance = msg->rightWheelDistance;
	//std::cout << "Odometry " << mOdometry.x << " " << mOdometry.y << " " << mOdometry.angle << std::endl;
	// set pose.theta based on angular change
	mPose.theta += mOdometry.angle - lastOdometry.angle;
	mPose.theta -= floor(mPose.theta / (2.0f * M_PI)) * 2.0f * M_PI; // move theta to [0,2PI]

	// Calc x & y with theta
	float leftDistance = mOdometry.leftWheelDistance - lastOdometry.leftWheelDistance;
	float rightDistance = mOdometry.rightWheelDistance - lastOdometry.rightWheelDistance;
	float distance = (leftDistance + rightDistance) / 2.0f;

	mPose.x += cos(mPose.theta) * distance;
	mPose.y += sin(mPose.theta) * distance;
}

void Mapper::receive_FollowWallState(const amee::FollowWallStates::ConstPtr &msg) {
	int type = 0;
	switch(msg->state) {
		case amee::MoveFollowWall::INIT:
			mLastNodeId = -1;
			break;
		case amee::MoveFollowWall::ALIGN_TO_WALL_OUT:
			init(); //  only does sth if not initalized
			type = amee::Node::NODE_NEXT_TO_WALL;
			addNode(type);
			mMappingState = NextToWall;
			break;
		case amee::MoveFollowWall::ROTATE_LEFT_IN:
			type = amee::Node::NODE_ROTATE_LEFT;
			addNode(type);
			mMappingState = Rotating;
			break;
		case amee::MoveFollowWall::ROTATE_RIGHT_IN:
			mMappingState = Rotating;
			type = amee::Node::NODE_ROTATE_RIGHT;
			addNode(type);
		default:
			mMappingState = Pause;
	}
}

void Mapper::addNode(int type) {
	amee::Node n(mPose,mNodeId);
	n.setType(type);
	mGraph.addNode(n);
	if (mLastNodeId != -1) {
		mGraph.connectNodes(mNodeId, mLastNodeId);
		mLastNodeId = mNodeId;
	}
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
		Measurement base;
		base.valid = false;
		base.pos.x = 0.0f;
		base.pos.y = 0.0f;
		mMeasurements.resize(4,base);
		mMappingState = Pause;
		mVisualizeTimer = 0;
		mCleanTimer = 0;
	}
	
}

void Mapper::calculateMeasurements() {
	// calculate a position for each sensor reading

	// right back
	if (isValidDistance(mDistances.rightBack)) {
		// right back position of 0
		Map::Point p;
		p.x = -0.052f;
		p.y = -0.12f;

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
		p.x = 0.052f;
		p.y = -0.12f;

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
		p.x = 0.052f;
		p.y = 0.12f;

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
		p.x = -0.052f;
		p.y = 0.12f;

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

void Mapper::visualize() {
	if (mVisualizeTimer == 8) {
		mVisualizeTimer = 0;
	
		mMap.getVisualization(mVis);
		mVis.robotPose.x = mPose.x;
		mVis.robotPose.y = mPose.y;
		mVis.robotPose.theta = mPose.theta;

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

		graph_pub.publish(mGraph.getGraphMsg());

	}
	++mVisualizeTimer;
}

void Mapper::cleanMap() {
	if (mCleanTimer == 8) {
		mCleanTimer = 0;
		Map::Point pos(mPose.x, mPose.y);
		mMap.reduceNumWalls(pos, 0.4f);
	}
	++mCleanTimer;
}

void Mapper::doMapping() {

	switch(mMappingState) {
		case Pause:
		case Rotating:	
			mapping();
			break;
		case NextToWall:
			mapping();
			break;
	}
	
	//TODO
	// std::cout << "Diff in timestamp: " << (mPose.timestamp - mDistances.timestamp) << std::endl;
}

void Mapper::mapping() {
	if (!mInitialized) {
		return;
	}
	// // set origin
	// mCurrentPos.x = mPose.x - mStartPos.x;
	// mCurrentPos.y = mPose.y - mStartPos.y;
	
	// rotate odometry coordinate system so that it is parallel to the map's system
	//mCurrentPos.rotate(-mStartAngle); 
	// mCurrentAngle = mPose.theta;// - mStartAngle;
	// std::cout << "Current position in maze: " << mPose.x << ", " << mPose.y << std::endl;
	// std::cout << "Current angle in maze: " << mPose.theta * (180.0f / M_PI) << std::endl;

	calculateMeasurements();

	int newType = followedWallDirection();

	WallSegment* walls[mMeasurements.size()];

	for (unsigned int i = 0; i < mMeasurements.size(); ++i) {
		Measurement m = mMeasurements[i];
		// std::cout << i <<": is valid? " << m.valid << " x " << m.pos.x << " y " << m.pos.y << std::endl;
		if (m.valid) {
			walls[i] = mMap.addMeasurement(m.pos, newType);
		} else {
			walls[i] = NULL;
		}
	}

	if((walls[RIGHT_BACK] == walls[RIGHT_FRONT]) && (walls[RIGHT_BACK] != NULL) && mMappingState == NextToWall) {
			// both measurements have been associated with the same wall
			// TODO change position
			WallSegment* wall = walls[RIGHT_BACK];
			
			// determine relative theta to the wall
			float diffDist = mDistances.rightFront - mDistances.rightBack;
			float meanDist = (mDistances.rightFront + mDistances.rightBack) / 2.0f;
			float relativeTheta = atan(diffDist / IR_BASE_RIGHT);
			std::cout << "Old pose: x:" << mPose.x << " y:" << mPose.y << " theta: " << mPose.theta << std::endl;
			// determine theta of the wall (orienation we are heading to)
			float wallTheta = 0.0f;
			if (wall->getType() == WallSegment::VERTICAL) {
				
				// std::cout << "wall x " << wall->getX() << " pose x " << mPose.x << std::endl;
				float sideOfWall = 1.0f;
				if (mPose.x <= wall->getX()) {
					wallTheta = M_PI / 2.0f;
					sideOfWall = -1.0f;
				} else {
					wallTheta = 3.0f / 2.0f * M_PI;
				}

				// now reset theta accordingly
				mPose.theta = wallTheta + relativeTheta;

				// now we want to reset the x coordinate
				float normalDistToWall = cos(relativeTheta) * meanDist;
				float centerDistToWall = normalDistToWall + cos(relativeTheta) * 0.12f;
				mPose.x = wall->getX() + sideOfWall * centerDistToWall;
			} else {
				// std::cout << "wall y " << wall->getY() << " pose y " << mPose.y << std::endl;
				float sideOfWall = 1.0f;
				if (mPose.y <= wall->getY()) {
					wallTheta = M_PI;
					sideOfWall = -1.0f;
				} else {
					wallTheta = 0.0f;
				}

				// now reset theta accordingly
				mPose.theta = wallTheta + relativeTheta;

				// now we want to reset the y coordinate
				float normalDistToWall = cos(relativeTheta) * meanDist;
				float centerDistToWall = normalDistToWall + cos(relativeTheta) * 0.12f;
				mPose.y = wall->getY() + sideOfWall * centerDistToWall;
			}

			std::cout << "New pose: x:" << mPose.x << " y:" << mPose.y << " theta: " << mPose.theta << std::endl;

			// std::cout << "Wall theta " << wallTheta * (180.0f / M_PI) << ", relativeTheta: " << relativeTheta * (180.0f / M_PI)
			// << " sum: " << (wallTheta + relativeTheta) * (180.0f / M_PI) << std::endl;
	}

	cleanMap();
	visualize();
	// mMap.print();
}

int Mapper::followedWallDirection() {
	float theta = mPose.theta;
	// if (mMappingState == NextToWall) {
		if ((fabs(theta) <= M_PI/4.0f) || (fabs(theta - M_PI) <= M_PI/4.0f)) {
			return WallSegment::HORIZONTAL;
		} 
		if ((fabs(theta - M_PI/2.0f) <= M_PI/4.0f) || (fabs(theta - 3.0f/2.0f*M_PI) <= M_PI/4.0f)) {
			return WallSegment::VERTICAL;
		}
		return WallSegment::NONE;
	// } else {
	// 	return WallSegment::NONE;
	// }
}


void mapTest(ros::Publisher& vispub) {
	Map map;

	Map::Point p;
	p.x = 0;
	p.y = 0;
	for (int i = 0; i < 100; ++i) {
		p.x += 0.01f; 
		map.addMeasurement(p, WallSegment::HORIZONTAL);
	}

	for (int i = 0; i < 100; ++i) {
		p.y += 0.01f;
		map.addMeasurement(p, WallSegment::VERTICAL); 
	}

	for (int i = 0; i < 10; ++i) {
		p.x -= 0.01f;
		map.addMeasurement(p, WallSegment::HORIZONTAL);
	}

	for (int i = 0; i < 10; ++i) {
		p.y -= 0.01f;
		map.addMeasurement(p, WallSegment::VERTICAL);
	}

	MapVisualization vis;
	map.getVisualization(vis);
	vispub.publish(vis);

}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "MapperNode");
	ros::NodeHandle n;

	ros::Publisher	map_pub;
	//map_pub = n.advertise<Velocity>("/amee/map", 100);

	// create the mapper
	Mapper mapper(map_pub);
	
	// create subscriber for distances
	ros::Subscriber dist_sub;
	dist_sub = n.subscribe("/amee/sensors/irdistances", 100, &Mapper::receive_distances, &mapper);
	ros::Subscriber odo_sub = n.subscribe("/amee/motor_control/odometry", 100, &Mapper::receiveOdometry, &mapper);
	ros::Subscriber state_sub = n.subscribe("/amee/follow_wall_states",10, &Mapper::receive_FollowWallState, &mapper);
	ros::Subscriber tag_sub = n.subscribe("/amee/tag",10, &Mapper::receive_tag, &mapper);

  	ros::Publisher marker_pub = n.advertise<amee::MapVisualization>("/amee/map/visualization", 10);
  	ros::Publisher graph_pub = n.advertise<amee::MapVisualization>("/amee/map/graph",10);

    mapper.setVisualizationPublisher(marker_pub);
    mapper.setGraphPublisher(graph_pub);

	ros::Rate loop_rate(30); //30
	
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
