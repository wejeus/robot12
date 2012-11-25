#include "ros/ros.h"

#include <iostream>
#include <std_msgs/Int32.h>
#include "PhaseOneControl.h"
#include "PhaseTwoControl.h"

using namespace std;

enum PHASE {ONE = 1, TWO = 2};
enum STATE {IDLE, RUN};

PHASE mPhase = ONE;
STATE mState = IDLE;

void executePhaseOne();
void executePhaseTwo();

// // Should determine state (phaseOne or phaseTwo)
// void buttonOneCallback(const std_msgs::Int32 &msg) {
// 	if (mPhase == ONE) {
// 		cout << "Switching to phase TWO" << endl;
// 		mPhase = TWO;
// 	} else {
// 		cout << "Switching to phase ONE" << endl;
// 		mPhase = ONE;
// 	}
// }

// // Should react to button click and start currently selected state.
// void buttonTwoCallback(const std_msgs::Int32 &msg) {
// 	if (mState == IDLE) {
// 		cout << "Starting phase: " << mPhase << endl;
// 		mState = RUN;

// 		// Once started it ignores new events from button
// 		if (mPhase == ONE) {
// 			executePhaseOne();
// 		} else if (mPhase == TWO) {
// 			executePhaseTwo();
// 		} else {
// 			cout << "MotherBrain: UNKNOWN STATE!" << endl;
// 		}

// 		mState = IDLE;
// 	}
// }

void executePhaseOne() {
	// TODO
}

void executePhaseTwo(ros::NodeHandle nodeHandle) {
	amee::PhaseTwoControl phaseControl(nodeHandle);

	while(phaseControl.isRunning()) {
		phaseControl.rescue();
	}
}


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "MotherBrain");//Creates a node named "StrategyControl"
	ros::NodeHandle nodeHandle;

	// ros::Subscriber buttonOne = nodeHandle.subscribe("/StrategyControl/ButtonOne", 10, &buttonOneCallback);
	// ros::Subscriber buttonTwo = nodeHandle.subscribe("/StrategyControl/ButtonTwo", 10, &buttonTwoCallback);
	
	// Testing code.
	executePhaseTwo(nodeHandle);
	cout << "done." << endl;

	// ros::Rate loop_rate(6);
	// while(ros::ok()){
	// 	// go to sleep for a short while
	// 	// loop_rate.sleep();
	// 	// call all callbacks
	// 	// ros::spinOnce();
	// 	ros::spin();
	// }

	return 0;
}