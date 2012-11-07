#include "Mapper.h"
#include <iostream>
#include <cmath>
#include <amee/MapVisualization.h>

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

void Mapper::receive_odometry(const Odometry::ConstPtr &msg) {
	// TODO others and synchronize with distances
	mOdometry.timestamp = msg->timestamp;
	// we want the angle in radians and in [0,2PI]
	mOdometry.angle = msg->angle;
	mOdometry.angle -= (floor(mOdometry.angle / 360.0f) * 360.0f); // move angle in [0,360]
	mOdometry.angle = mOdometry.angle * M_PI / 180.0f; 
	mOdometry.distance = msg->distance;
	mOdometry.x = msg->x;
	mOdometry.y = msg->y;

	if (!mInitialized) {
		init();
	}
	
}

void Mapper::init() {
	mStartAngle = mOdometry.angle;
	mStartPos.x = mOdometry.x;
	mStartPos.y = mOdometry.y;
	mInitialized = true;
	Measurement base;
	base.valid = false;
	base.pos.x = 0.0f;
	base.pos.y = 0.0f;
	mMeasurements.resize(7,base);
}

void Mapper::calculateMeasurements() {
	// calculate a position for each sensor reading

	// right back
	if (isValidDistance(mDistances.rightBack)) {
		// right back position of 0
		Map::Point p;
		p.x = -0.05f;
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
		p.x = 0.05f;
		p.y = -0.12f;

		// add distance vector (positive y is left of the robot, negative y right)
		p.y += -mDistances.rightFront;

		mMeasurements[1].valid = true;
		p.rotate(mCurrentAngle);
		mMeasurements[1].pos = p + mCurrentPos;
	} else {
		mMeasurements[1].valid = false;
	}
	
}

bool Mapper::isValidDistance(float dist) {
	return dist >= 0.0f && dist <= 15.0f;
}

void Mapper::setVisualizationPublisher(ros::Publisher pub) {
	vis_pub = pub;
}

void Mapper::visualize() {
	MapVisualization vis;
	mMap.getVisualization(vis);
	vis_pub.publish(vis);
}

void Mapper::doMapping() {
	if (mInitialized) {
		// set origin
		mCurrentPos.x = mOdometry.x - mStartPos.x;
		mCurrentPos.y = mOdometry.y - mStartPos.y;
		
		// rotate odometry coordinate system so that it is parallel to the map's system
		mCurrentPos.rotate(mStartAngle); 
		mCurrentAngle = mOdometry.angle - mStartAngle;
		std::cout << "Current position in maze: " << mCurrentPos.x << ", " << mCurrentPos.y << std::endl;
		std::cout << "Current angle in maze: " << mCurrentAngle * (180.0f / M_PI) << std::endl;
		calculateMeasurements();

		for (int i = 0; i < mMeasurements.size(); ++i) {
			Measurement m = mMeasurements[i];
			std::cout << i <<": is valid? " << m.valid << " x " << m.pos.x << " y " << m.pos.y << std::endl;
			if (m.valid) {
				mMap.addMeasurement(m.pos);
			}
		}
		visualize();
		mMap.print();

	}


	//TODO
	// std::cout << "Diff in timestamp: " << (mOdometry.timestamp - mDistances.timestamp) << std::endl;
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
	ros::Subscriber odo_sub = n.subscribe("/amee/motor_control/odometry", 100, &Mapper::receive_odometry, &mapper);

  	ros::Publisher marker_pub = n.advertise<amee::MapVisualization>("/amee/map/visualization", 10);

    mapper.setVisualizationPublisher(marker_pub);

	ros::Rate loop_rate(10); //30
	
	while(ros::ok()){
		
		// go to sleep for a short while
		loop_rate.sleep();

		// call all callbacks
		ros::spinOnce();
		
		// map!
		mapper.doMapping();
		//mapTest(marker_pub);
	}

	return 0;
}
