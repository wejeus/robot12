#ifndef SONAR_CONTROL_H
#define SONAR_CONTROL_H

#include "roboard_drivers/sonar.h"
#include "roboard_drivers/servo.h"
#include "amee/Sonar.h"

class SonarControl {
private:
	
	int mLastAngle;
	int mDirection;
	ros::Publisher servo_pub;
	ros::Publisher distance_pub;
	roboard_drivers::servo mServoMsg;
	amee::Sonar mSonarMsg;

	static const int PORT_ID = 4;
	static const int DEGREE_MIN = -30;
	static const int DEGREE_MAX = 30;
	static const int DEGREE_STEP = 10;

public:
		SonarControl(ros::Publisher& servo_pub);
		void setDistancePublisher(ros::Publisher pub);
		void receiveDistance(const roboard_drivers::sonar::ConstPtr &msg);
		void publishServo(); // publish mLastAngle to servo
		//TODO listen to on/off topic
		void moveOn();
};
#endif