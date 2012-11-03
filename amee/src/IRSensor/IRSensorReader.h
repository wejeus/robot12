#ifndef IR_SENSOR_READER_H
#define IR_SENSOR_READER_H

#include "roboard_drivers/adc_val.h"
#include <vector>

class IRSensorReader {
private:
	struct SensorCalibration {
		float m;
		float b;
		float k;
	};
	
	std::vector<SensorCalibration> mSensorCalibrations;
	ros::Publisher distance_pub;
	std::vector<int> mAveragedValues;
	std::vector<int> mLastReadings;
	int mNumAveraged;
	int mNumSensors;

	static const int MAX_NUM_AVERAGED = 6;
	static const int NUM_PORTS = 8;
		
	// sensor ports (these are not the same as the numbers on the roboard!)
	static const int RIGHT_FRONT = 1;
	static const int RIGHT_BACK = 2;
	static const int FRONT_SHORTRANGE = 3;
	static const int WHEEL_RIGHT = 6;
	static const int WHEEL_LEFT = 7;
	static const int LEFT_FRONT = 0;
	static const int LEFT_BACK = 5;


public:
		IRSensorReader();
		void setDistancePublisher(ros::Publisher pub);
		void calibrate(int sensor);
		void receiveRawData(const roboard_drivers::adc_val::ConstPtr &msg);
	// protected:
	// private:
};
#endif