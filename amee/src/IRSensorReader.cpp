#include "ros/ros.h"
#include <string.h>
#include "IRSensorReader.h"
#include <iostream>
#include <cmath>
#include "roboard_drivers/serial_adc_val.h"
#include "roboard_drivers/enable_adc_val.h"
#include "amee/IRDistances.h"

using namespace amee;
using namespace roboard_drivers;

IRSensorReader::IRSensorReader(int numSensors) {
	SensorCalibration baseCalib;
	baseCalib.alpha = 0.0f;
	baseCalib.lambda = 0.0f;
	baseCalib.c = 0.0f;
	mSensorCalibrations.resize(numSensors, baseCalib);
}

void IRSensorReader::calibrate(int sensor) {
	std::cout << "Starting calibration for sensor, press enter to continue" << sensor << std::endl;
	char input = 0;
	std::cin >> input;
	std::cout << "	Set zero distance and press enter if the distance is set.";// TODO
	std::cin >> input;
	std::cout << "loop" << std::endl;
	ros::spinOnce();
	std::cout << "	Set infinite distance and press enter if the distance is set.";// TODO
	std::cin >> input;
	std::cout << "loop" << std::endl;
	ros::spinOnce();
}

void IRSensorReader::setDistancePublisher(ros::Publisher pub) {
	distance_pub = pub;
}

void IRSensorReader::receiveRawData(const serial_adc_val::ConstPtr &msg) {
	std::cout << "Message received: " << msg->timestamp << ": " << msg->val0 << std::endl;
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "IRSensorReader");//Creates a node named "IRSensorReader"
	ros::NodeHandle n;

	// create the reader
	IRSensorReader reader(1);

	// make sure we get ad values
	ros::Publisher serial_enable_adc = n.advertise<enable_adc_val>("/serial/enable_adc", 1);
	enable_adc_val enable_msg;
	enable_msg.enable0 = true;
	enable_msg.enable1 = false;
	enable_msg.enable2 = false;
	enable_msg.enable3 = false;
	enable_msg.enable4 = false;
	serial_enable_adc.publish(enable_msg);

	// create subscriber for sensor input
	ros::Subscriber raw_sensor_sub;
	raw_sensor_sub = n.subscribe("/serial/adc", 1000, &IRSensorReader::receiveRawData, &reader);

	// create publisher for distances
	ros::Publisher distance_pub = n.advertise<IRDistances>("/amee/sensors/irdistances", 1000);
	reader.setDistancePublisher(distance_pub);

	// set our reader loop at 100Hz
	ros::Rate loop_rate(100);
		
	reader.calibrate(0);
	// while(ros::ok()){

	// 	// go to sleep for a short while
	// 	loop_rate.sleep();

	// 	// call all callbacks
	// 	ros::spinOnce();
	// }

	return 0;
}
