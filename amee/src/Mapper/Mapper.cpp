#include "Mapper.h"
#include <iostream>
#include <cmath>
#include <visualization_msgs/Marker.h>

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
	visualization_msgs::Marker points;
    points.header.frame_id = "/map_visual";
    points.header.stamp = ros::Time::now();
    points.ns = "points_and_lines";
    points.action = visualization_msgs::Marker::ADD;
    points.pose.orientation.w = 1.0;

    points.id = 0;



    points.type = visualization_msgs::Marker::POINTS;
    // line_strip.type = visualization_msgs::Marker::LINE_STRIP;
    // line_list.type = visualization_msgs::Marker::LINE_LIST;



    // POINTS markers use x and y scale for width/height respectively
    points.scale.x = 0.2;
    points.scale.y = 0.2;

    // // LINE_STRIP/LINE_LIST markers use only the x component of scale, for the line width
    // line_strip.scale.x = 0.1;
    // line_list.scale.x = 0.1;

    // Points are green
    points.color.g = 1.0f;
    points.color.a = 1.0;

    // // Line strip is blue
    // line_strip.color.b = 1.0;
    // line_strip.color.a = 1.0;

    // // Line list is red
    // line_list.color.r = 1.0;
    // line_list.color.a = 1.0;

    // Create the vertices for the points and lines
    for (unsigned int i = 0; i < mMeasurements.size(); ++i)
    {

      geometry_msgs::Point p;
      p.x = mMeasurements[i].pos.x;
      p.y = mMeasurements[i].pos.y;
      p.z = 0.0f;

      points.points.push_back(p);
      // line_strip.points.push_back(p);

      // The line list needs two points for each line
      // line_list.points.push_back(p);
      // p.z += 1.0;
      // line_list.points.push_back(p);
    }


    vis_pub.publish(points);
    // marker_pub.publish(line_strip);
    // marker_pub.publish(line_list);
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
		// visualize();
		mMap.print();

	}


	//TODO
	// std::cout << "Diff in timestamp: " << (mOdometry.timestamp - mDistances.timestamp) << std::endl;
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

  	ros::Publisher marker_pub = n.advertise<visualization_msgs::Marker>("visualization_marker", 10);

    mapper.setVisualizationPublisher(marker_pub);

	ros::Rate loop_rate(30);
	
	while(ros::ok()){
		
		// go to sleep for a short while
		loop_rate.sleep();

		// call all callbacks
		ros::spinOnce();
		
		// map!
		mapper.doMapping();
	}

	return 0;
}
