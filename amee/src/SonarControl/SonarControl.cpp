#include "ros/ros.h"
#include "SonarControl.h"
#include <iostream>
#include <std_msgs/Int32.h>

using namespace amee;
using namespace roboard_drivers;

SonarControl::SonarControl(ros::Publisher& pub) {
	servo_pub = pub;
	mLastAngle = DEGREE_MIN;
	mDirection = 1;
	mServoMsg.id = PORT_ID;
	publishServo();
}

void SonarControl::setDistancePublisher(ros::Publisher pub) {
	distance_pub = pub;
}

void SonarControl::publishServo() {
	mServoMsg.angle = mLastAngle;
	servo_pub.publish(mServoMsg);
}

void SonarControl::receiveDistance(const sonar::ConstPtr &msg) {
	mSonarMsg.distance = msg->distance;
	mSonarMsg.timestamp = msg->timestamp;
	mSonarMsg.angle = mLastAngle;

	distance_pub.publish(mSonarMsg);
}

void SonarControl::moveOn() {
	if (mLastAngle > DEGREE_MAX) {
		mDirection = -1;
	} else if (mLastAngle < DEGREE_MIN) {
		mDirection = 1;
	}

	mLastAngle += mDirection * DEGREE_STEP;

	publishServo();
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "SonarControlNode");
	ros::NodeHandle n;

	// create the control
	ros::Publisher servo_pub = n.advertise<roboard_drivers::servo>("/serial/move_servo",1);
	SonarControl control(servo_pub);

	// create subscriber for sensor input
	ros::Subscriber sonar_sub;
	sonar_sub = n.subscribe("/roboard/sonar", 1, &SonarControl::receiveDistance, &control);

	ros::Rate loop_rate(12);
	ros::Publisher sonar_interval = n.advertise<std_msgs::Int32>("/roboard/sonar_interval", 1);
	while(sonar_interval.getNumSubscribers() == 0 && ros::ok()) {
	 	loop_rate.sleep();
	 } 

	std_msgs::Int32 interval;
	interval.data = 40;
	sonar_interval.publish(interval);

	// create publisher for distances
	ros::Publisher distance_pub = n.advertise<Sonar>("/amee/sensors/sonar", 100);
	control.setDistancePublisher(distance_pub);
	
		
	
	while(ros::ok()){

	 	// go to sleep for a short while
	 	loop_rate.sleep();

	 	// call all callbacks
	 	ros::spinOnce();

	 	// let control move the servo
	 	control.moveOn();
	 }

	return 0;
}
