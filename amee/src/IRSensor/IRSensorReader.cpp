#include "ros/ros.h"
#include <string.h>
#include "IRSensorReader.h"
#include <iostream>
#include <cmath>
#include <std_msgs/Int32.h>
#include <sys/time.h>
#include "amee/IRDistances.h"

using namespace amee;
using namespace roboard_drivers;

IRSensorReader::IRSensorReader() {
	SensorCalibration baseCalib;
	baseCalib.m = 0.0f;
	baseCalib.b = 0.0f;
	baseCalib.k = 0.0f;
	mSensorCalibrations.resize(NUM_PORTS, baseCalib);
	
	// set values determined by manual calibration

	// RIGHT FRONT SHORT RANGE
	baseCalib.m = 0.0338;
	baseCalib.b = 0.1818;
	baseCalib.k = 0.0572;
	mSensorCalibrations[RIGHT_FRONT] = baseCalib;

	// RIGHT BACK SHORT RANGE
	baseCalib.m = 0.0417;
	baseCalib.b = -1.0692;
	baseCalib.k = 0.049;
	mSensorCalibrations[RIGHT_BACK] = baseCalib;

	// LEFT FRONT SHORT RANGE
	baseCalib.m = 0.0387;
	baseCalib.b = -0.8602;
	baseCalib.k = 0.0557;
	mSensorCalibrations[LEFT_FRONT] = baseCalib;

	// LEFT BACK SHORT RANGE
	baseCalib.m = 0.0321;
	baseCalib.b = 0.4011;
	baseCalib.k = 0.0565;
	mSensorCalibrations[LEFT_BACK] = baseCalib;

	// FRONT SHORT RANGE
	// baseCalib.m = 0.0346;
	// baseCalib.b = -0.0489;
	// baseCalib.k = 0.0751;
	// mSensorCalibrations[FRONT_SHORTRANGE] = baseCalib;

	// RIGHT LONG RANGE ABOVE OF THE WHEEL
	baseCalib.m = 0.0181f;
	baseCalib.b = -0.1464f;
	baseCalib.k = 0.1373f;
	mSensorCalibrations[WHEEL_RIGHT] = baseCalib;

	// LEFT LONG RANGE ABOVE OF THE WHEEL
	baseCalib.m = 0.0185f;
	baseCalib.b = -0.1672f;
	baseCalib.k = 0.1301f;
	mSensorCalibrations[WHEEL_LEFT] = baseCalib;

	mAveragedValues.resize(NUM_PORTS, 0);
	mLastReadings.resize(NUM_PORTS,0);
	mNumAveraged = 0;
}

float IRSensorReader::limit(float val) {
	return std::min(std::max(-1.0f, val),1.0f);
}

void IRSensorReader::setDistancePublisher(ros::Publisher pub) {
	distance_pub = pub;
}

void IRSensorReader::receiveRawData(const adc_val::ConstPtr &msg) {
	//std::cout << "Message received: " << msg->timestamp << ": " << msg->val0 << std::endl;
	if (mNumAveraged < MAX_NUM_AVERAGED) {
		int values[] = {msg->val0, msg->val1, msg->val2, msg->val3, msg->val4, msg->val5, msg->val6, msg->val7};
		for (int i = 0; i < NUM_PORTS; ++i) {
			mAveragedValues[i] += values[i];
		}
		++mNumAveraged;
	} else {
		for (int i = 0; i < NUM_PORTS; ++i) {
			mLastReadings[i] = mAveragedValues[i] / MAX_NUM_AVERAGED;
			mAveragedValues[i] = 0;
		}
		float distances[NUM_PORTS];
		for (int i = 0; i < NUM_PORTS; ++i) {

			distances[i] = 1.0f / (mSensorCalibrations[i].m * mLastReadings[i] + mSensorCalibrations[i].b) - mSensorCalibrations[i].k;

		}
		//std::cout << std::endl;

		IRDistances distanceMsg;
		struct timeval time;
		gettimeofday(&time, NULL);
		distanceMsg.timestamp = time.tv_sec+double(time.tv_usec)/1000000.0;  ;//msg->timestamp;
		distanceMsg.rightFront = limit(distances[RIGHT_FRONT]);
		distanceMsg.rightBack = limit(distances[RIGHT_BACK]);
		//distanceMsg.frontShortRange = distances[FRONT_SHORTRANGE];
		distanceMsg.wheelRight = limit(distances[WHEEL_RIGHT]);
		distanceMsg.leftBack = limit(distances[LEFT_BACK]);
		distanceMsg.leftFront = limit(distances[LEFT_FRONT]);
		distanceMsg.wheelLeft = limit(distances[WHEEL_LEFT]);

		distanceMsg.obstacleInFront = ((mLastReadings[LEFT_FRONT_WALL_DETECTOR] >= 290)  && (mLastReadings[LEFT_FRONT_WALL_DETECTOR] <= 560))
								   || ((mLastReadings[RIGHT_FRONT_WALL_DETECTOR] >= 265) && (mLastReadings[RIGHT_FRONT_WALL_DETECTOR] <= 550));

		//TODO publish all the other correct distances
		
		// std::cout << "timestamp: " << distanceMsg.timestamp << std::endl;
		// std::cout << "leftBack: " << mLastReadings[LEFT_BACK] << std::endl;
		// std::cout << "leftFront: " << mLastReadings[LEFT_FRONT] << std::endl;
		// std::cout << "wheelLeft: " << mLastReadings[WHEEL_LEFT] << std::endl;
		// std::cout << "rightBack: " << mLastReadings[RIGHT_BACK] << std::endl;
		// std::cout << "rightFront: " << mLastReadings[RIGHT_FRONT] << std::endl;
		// std::cout << "wheelRight: " << mLastReadings[WHEEL_RIGHT] << std::endl;
		// std::cout << "frontShort: " << mLastReadings[FRONT_SHORTRANGE] << std::endl;
		 // std::cout << "frontLeftWallDetactor: " << mLastReadings[LEFT_FRONT_WALL_DETECTOR] << std::endl;
		 // std::cout << "frontRightWallDetactor: " << mLastReadings[RIGHT_FRONT_WALL_DETECTOR] << std::endl;

		
		distance_pub.publish(distanceMsg);

		mNumAveraged = 0;
	}
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "IRSensorReader");//Creates a node named "IRSensorReader"
	ros::NodeHandle n;

	// create the reader
	IRSensorReader reader;

	// make sure we get ad values
	// ros::Publisher serial_enable_adc = n.advertise<enable_adc_val>("/serial/enable_adc", 1);
	// enable_adc_val enable_msg;
	// enable_msg.enable0 = true;
	// enable_msg.enable1 = true;
	// enable_msg.enable2 = false;
	// enable_msg.enable3 = false;
	// enable_msg.enable4 = false;
	// serial_enable_adc.publish(enable_msg);

	// create subscriber for sensor input
	ros::Subscriber raw_sensor_sub;
	raw_sensor_sub = n.subscribe("/roboard/adc", 1000, &IRSensorReader::receiveRawData, &reader);

	// set our reader loop at 100Hz
	ros::Rate loop_rate(100);
	ros::Publisher adc_interval = n.advertise<std_msgs::Int32>("/roboard/adc_interval", 10);
	while(adc_interval.getNumSubscribers() == 0 && ros::ok()) {
	 	loop_rate.sleep();
	 } 

	std_msgs::Int32 interval;
	interval.data = 10;
	adc_interval.publish(interval);

	// create publisher for distances
	ros::Publisher distance_pub = n.advertise<IRDistances>("/amee/sensors/irdistances", 1000);
	reader.setDistancePublisher(distance_pub);
	
		
	
	while(ros::ok()){

	 	// go to sleep for a short while
	 	loop_rate.sleep();

	 	// call all callbacks
	 	ros::spinOnce();
	 }

	return 0;
}
