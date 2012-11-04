#include "ros/ros.h"
#include <string.h>
#include "IRSensorReader.h"
#include <iostream>
#include <cmath>
#include <std_msgs/Int32.h>
// #include "roboard_drivers/serial_adc_val.h"
// #include "roboard_drivers/enable_adc_val.h"
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
	mSensorCalibrations[LEFT_FRONT] = baseCalib;
	mSensorCalibrations[LEFT_BACK] = baseCalib;

	// RIGHT BACK SHORT RANGE
	baseCalib.m = 0.0417;
	baseCalib.b = -1.0692;
	baseCalib.k = 0.049;
	mSensorCalibrations[RIGHT_BACK] = baseCalib;

	// FRONT SHORT RANGE
	baseCalib.m = 0.0346;
	baseCalib.b = -0.0489;
	baseCalib.k = 0.0751;
	mSensorCalibrations[FRONT_SHORTRANGE] = baseCalib;

	// RIGHT LONG RANGE ABOVE OF THE WHEEL
	baseCalib.m = 0.0181f;
	baseCalib.b = -0.1464f;
	baseCalib.k = 0.1373f;
	mSensorCalibrations[WHEEL_RIGHT] = baseCalib;
	mSensorCalibrations[WHEEL_LEFT] = baseCalib;

	mAveragedValues.resize(NUM_PORTS, 0);
	mLastReadings.resize(NUM_PORTS,0);
	mNumAveraged = 0;
}

void IRSensorReader::calibrate(int sensor) {
	// std::cout << "Starting calibration for sensor " << sensor << ", type 0 and press enter to continue" << std::endl;
	// float input = 0;
	// std::cin >> input;
	// std::cout << "	Place an obstacle where zero distance is supposed to be and type 0 followed by enter" << std::endl;
	// std::cin >> input;
	// ros::spinOnce();

	// // alpha is the voltage for zero distance and describes the amplitude
	// SensorCalibration calib;
	// calib.alpha = mLastReadings[sensor];
	
	// std::cout << "	Remove any obstacle so that the sensor can 'see' as far as possible and type 0 followed by enter" << std::endl;
	// std::cin >> input;
	// ros::spinOnce();

	// // c is the voltage for infinite distance and describes an offset
	// calib.c = mLastReadings[sensor];

	// std::cout << "	Place an obstacle to a distance of your choice ( > 0) and enter the distance followed by enter" << std::endl;
	// std::cin >> input;
	// ros::spinOnce();

	// // lambda is the parameter that determines how fast the voltage drops
	// calib.lambda = log(calib.alpha / (mLastReadings[sensor] - calib.c)) / input;
	
	// std::cout << "Calibration is done: " << std::endl;
	// std::cout << "	alpha = " << calib.alpha << std::endl;
	// std::cout << "	lambda = " << calib.lambda << std::endl;
	// std::cout << "	c = " << calib.c << std::endl;

	// mSensorCalibrations[sensor] = calib;
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
			// if (mLastReadings[i] == mSensorCalibrations[i].c) {
			// 	distances[i] = 1000.0f; // TODO set to limits::max
			// } else {
				//distances[i] = log(mSensorCalibrations[i].alpha / (mLastReadings[i] - mSensorCalibrations[i].c)) / mSensorCalibrations[i].lambda;
				//std::cout << mLastReadings[i] << " ";
				distances[i] = 1.0f / (mSensorCalibrations[i].m * mLastReadings[i] + mSensorCalibrations[i].b) - mSensorCalibrations[i].k;
				// distances[i] = distances[i] >= 0.0f ? distances[i] : 0.0f;
				//int lastDigit = (int)(distances[i] * 1000.0f) % 10;
				//distances[i] = lastDigit > 4 ? ceil(distances[i] * 100.0f) / 100.0f : floor(distances[i] * 100.0f) / 100.0f;
			// } 
		}
		//std::cout << std::endl;

		IRDistances distanceMsg;
		distanceMsg.timestamp = msg->timestamp;
		distanceMsg.rightFront = distances[RIGHT_FRONT];
		distanceMsg.rightBack = distances[RIGHT_BACK];
		distanceMsg.frontShortRange = distances[FRONT_SHORTRANGE];
		distanceMsg.wheelRight = distances[WHEEL_RIGHT];
		distanceMsg.leftBack = distances[LEFT_BACK];
		distanceMsg.leftFront = distances[LEFT_FRONT];
		distanceMsg.wheelLeft = distances[WHEEL_LEFT];
		//TODO publish all the other correct distances
		
		std::cout << "leftBack: " << mLastReadings[LEFT_BACK] << std::endl;
		std::cout << "leftFront: " << mLastReadings[LEFT_FRONT] << std::endl;
		std::cout << "wheelLeft: " << mLastReadings[WHEEL_LEFT] << std::endl;
		
		
		
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
	ros::Publisher adc_interval = n.advertise<std_msgs::Int32>("roboard/adc_interval", 10);
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
