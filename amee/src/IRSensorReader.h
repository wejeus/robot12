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

	static const int MAX_NUM_AVERAGED = 5;
	static const int NUM_PORTS = 8;
	//TODO all the other sensors
	static const int RIGHT_FRONT = 1;
	static const int RIGHT_BACK = 2;



public:
		IRSensorReader();
		void setDistancePublisher(ros::Publisher pub);
		void calibrate(int sensor);
		void receiveRawData(const roboard_drivers::adc_val::ConstPtr &msg);
	// protected:
	// private:
};
#endif