#include "Mapper.h"
#include <iostream>
#include <cmath>
#include "../MovementControl/MoveFollowWall.h"

using namespace amee;

Mapper::Mapper(ros::Publisher pub) {
	map_pub = pub;
	mInitialized = false;
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

void Mapper::receive_pose(const Pose::ConstPtr &msg) {
	// TODO others and synchronize with distances
	mPose.timestamp = msg->timestamp;
	// we want the angle in radians and in [0,2PI]
	mPose.theta = msg->theta;
	mPose.theta -= (floor(mPose.theta / 360.0f) * 360.0f); // move angle in [0,360]
	mPose.theta = mPose.theta * M_PI / 180.0f; 
	// mPose.distance = msg->distance;
	mPose.x = msg->x;
	mPose.y = msg->y;
	
}

void Mapper::init(const FollowWallStates::ConstPtr &msg) {
	if (msg->state == amee::MoveFollowWall::ALIGNED_TO_WALL && !mInitialized) {
		mStartAngle = mPose.theta - (mPose.theta - floor(mPose.theta / 90.0f + 0.5f) * 90.0f);
		mStartPos.x = mPose.x;
		mStartPos.y = mPose.y;
		mInitialized = true;
		Measurement base;
		base.valid = false;
		base.pos.x = 0.0f;
		base.pos.y = 0.0f;
		mMeasurements.resize(7,base);

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

		mMeasurements[0].valid = true;
		p.rotate(mCurrentAngle);
		mMeasurements[0].pos = p + mCurrentPos;
	} else {
		mMeasurements[0].valid = false;
	}

	// right front
	if (isValidDistance(mDistances.rightFront)) {
		// right front position of 0
		Map::Point p;
		p.x = 0.052f;
		p.y = -0.12f;

		// add distance vector (positive y is left of the robot, negative y right)
		p.y += -mDistances.rightFront;

		mMeasurements[1].valid = true;
		p.rotate(mCurrentAngle);
		mMeasurements[1].pos = p + mCurrentPos;
	} else {
		mMeasurements[1].valid = false;
	}
	// left front
	if (isValidDistance(mDistances.leftFront)) {
		// left front position of 0
		Map::Point p;
		p.x = 0.052f;
		p.y = 0.12f;

		// add distance vector (positive y is left of the robot, negative y right)
		p.y += mDistances.leftFront;

		mMeasurements[2].valid = true;
		p.rotate(mCurrentAngle);
		mMeasurements[2].pos = p + mCurrentPos;
	} else {
		mMeasurements[2].valid = false;
	}

	// left back
	if (isValidDistance(mDistances.leftBack)) {
		// left back position of 0
		Map::Point p;
		p.x = -0.052f;
		p.y = 0.12f;

		// add distance vector (positive y is left of the robot, negative y right)
		p.y += mDistances.leftBack;

		mMeasurements[3].valid = true;
		p.rotate(mCurrentAngle);
		mMeasurements[3].pos = p + mCurrentPos;
	} else {
		mMeasurements[3].valid = false;
	}
	
}

bool Mapper::isValidDistance(float dist) {
	return dist >= 0.0f && dist <= 0.15f;
}

void Mapper::setVisualizationPublisher(ros::Publisher pub) {
	vis_pub = pub;
}

void Mapper::visualize() {
	if (mVisualizeTimer == 8) {
		mVisualizeTimer = 0;
	
		mMap.getVisualization(mVis);
		mVis.robotPose.x = mCurrentPos.x;
		mVis.robotPose.y = mCurrentPos.y;
		mVis.robotPose.theta = mCurrentAngle;

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
	}
	++mVisualizeTimer;
}

void Mapper::setAngleToN90Deg() {
	// float diffToPi = M_PI - mCurrentAngle;
	// diffToPi = diffToPi >= 0.0f ? diffToPi : -diffToPi;
	// float diffToHalfPi = M_PI/2.0f - mCurrentAngle;
	// diffToHalfPi = diffToHalfPi >= 0.0f ? diffToHalfPi : -diffToHalfPi;
	// float diffToThreeHalfPi = 3.0f / 2.0f * M_PI - mCurrentAngle;
	// diffToThreeHalfPi = diffToThreeHalfPi >= 0.0f ? diffToThreeHalfPi : -diffToThreeHalfPi;

	float timesPi = mCurrentAngle / (M_PI / 2);
	mCurrentAngle = floor(timesPi + 0.5f) * (M_PI/2.0f);
	std::cout << "setting angle to " << mCurrentAngle << std::endl;
}
void Mapper::cleanMap() {
	if (mCleanTimer == 8) {
		mCleanTimer = 0;
		mMap.reduceNumWalls(mCurrentPos, 0.4f);
	}
	++mCleanTimer;
}

void Mapper::doMapping() {
	if (mInitialized) {
		// set origin
		mCurrentPos.x = mPose.x - mStartPos.x;
		mCurrentPos.y = mPose.y - mStartPos.y;
		
		// rotate odometry coordinate system so that it is parallel to the map's system
		//mCurrentPos.rotate(-mStartAngle); 
		mCurrentAngle = mPose.theta;// - mStartAngle;
		std::cout << "Current position in maze: " << mCurrentPos.x << ", " << mCurrentPos.y << std::endl;
		std::cout << "Current angle in maze: " << mCurrentAngle * (180.0f / M_PI) << std::endl;
	
		// setAngleToN90Deg();	
	
		calculateMeasurements();

		for (unsigned int i = 0; i < mMeasurements.size(); ++i) {
			Measurement m = mMeasurements[i];
			// std::cout << i <<": is valid? " << m.valid << " x " << m.pos.x << " y " << m.pos.y << std::endl;
			if (m.valid) {
				mMap.addMeasurement(m.pos);
			}
		}
		cleanMap();
		visualize();
		mMap.print();

	}


	//TODO
	// std::cout << "Diff in timestamp: " << (mPose.timestamp - mDistances.timestamp) << std::endl;
}

void Mapper::receive_tag(const amee::Tag::ConstPtr& msg) {
	Map::Point p;
	p.x = mCurrentPos.x;
	p.y = mCurrentPos.y;
	mTagPositions.push_back(p);
}

void mapTest(ros::Publisher& vispub) {
	Map map;

	Map::Point p;
	p.x = 0;
	p.y = 0;
	for (int i = 0; i < 100; ++i) {
		p.x += 0.01f; 
		map.addMeasurement(p);
	}

	for (int i = 0; i < 100; ++i) {
		p.y += 0.01f;
		map.addMeasurement(p); 
	}

	for (int i = 0; i < 10; ++i) {
		p.x -= 0.01f;
		map.addMeasurement(p);
	}

	for (int i = 0; i < 10; ++i) {
		p.y -= 0.01f;
		map.addMeasurement(p);
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
	ros::Subscriber odo_sub = n.subscribe("/amee/pose", 100, &Mapper::receive_pose, &mapper);
	ros::Subscriber state_sub = n.subscribe("/amee/follow_wall_states",10, &Mapper::init, &mapper);
	ros::Subscriber tag_sub = n.subscribe("/amee/tag",10, &Mapper::receive_tag, &mapper);

  	ros::Publisher marker_pub = n.advertise<amee::MapVisualization>("/amee/map/visualization", 10);

    mapper.setVisualizationPublisher(marker_pub);

	ros::Rate loop_rate(30); //30
	
	while(ros::ok()){
		
		// go to sleep for a short while
		loop_rate.sleep();

		// call all callbacks
		ros::spinOnce();
		
		// map!
		mapper.doMapping();
		// mapTest(marker_pub);
	}

	return 0;
}
