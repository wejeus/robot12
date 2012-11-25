#include "ros/ros.h"

#include <iostream>
#include <std_msgs/Int32.h>
#include "PhaseOneControl.h"
#include "PhaseTwoControl.h"

using namespace std;

enum PHASE {IDLE, ONE, TWO};
enum STATE {RUN, STOP};

PHASE mPhase = IDLE;
STATE mState = STOP;

// Should determine state (phaseOne or phaseTwo)
void buttonOneCallback(const std_msgs::Int32 &msg) {
	switch (msg.phase) {
		case 1:
			mPhase = ONE;
			break
		case 2:
			mPhase = TWO;
			break
		default:
			mPhase = IDLE;
	}
}

// Should react to button click and start currently selected state.
void buttonTwoCallback(const std_msgs::Int32 &msg) {
	switch (msg.state) {
		case 1:
			mState = RUN;
			break
		default:
			mState = STOP;
	}

	// Once started it ignores new events from button
	if (mState == RUN) {
		if (mPhase == ONE) {
			executePhaseOne();
		} else if (mPhase == TWO) {
			executePhaseTwo();
		} else {
			cout << "MotherBrain: UNKNOWN STATE!" << endl;
		}

		mState = STOP;
		mPhase = IDLE;
	}
}

void executePhaseOne() {
	// TODO
	PhaseTwoControl phaseControl;
	phaseControl.execute();
}

void executePhaseOne() {
	PhaseTwoControl phaseControl;
	phaseControl.execute();
}


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "MotherBrain");//Creates a node named "StrategyControl"
	ros::NodeHandle nodeHandle;

	ros::Subscriber buttonOne = nodeHandle.subscribe("/StrategyControl/ButtonOne", 10, &buttonOneCallback);
	ros::Subscriber buttonTwo = nodeHandle.subscribe("/StrategyControl/ButtonTwo", 10, &buttonTwoCallback);
	
	// ros::Rate loop_rate(6);
	
	while(ros::ok()){
		// go to sleep for a short while
		// loop_rate.sleep();
		// call all callbacks
		// ros::spinOnce();
		ros::spin();
	}

	return 0;
}