#ifndef IR_SENSOR_READER_H
#define IR_SENSOR_READER_H

#include "roboard_drivers/serial_adc_val.h"
#include <vector>

class IRSensorReader {
private:
	struct SensorCalibration {
		float alpha;
		float lambda;
		float c;
	};
	
	std::vector<SensorCalibration> mSensorCalibrations;
	ros::Publisher distance_pub;
	std::vector<int> mAveragedValues;
	std::vector<int> mLastReadings;
	int mNumAveraged;
	int mNumSensors;

	static const int MAX_NUM_AVERAGED = 5;


public:
		IRSensorReader(int numSensors);
		void setDistancePublisher(ros::Publisher pub);
		void calibrate(int sensor);
		void receiveRawData(const roboard_drivers::serial_adc_val::ConstPtr &msg);
	// protected:
	// private:
};
#endif