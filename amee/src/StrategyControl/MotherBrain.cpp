#include "ros/ros.h"

#include <stdio.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include "PhaseOneControl.h"
#include "PhaseTwoControl.h"
#include <unistd.h>


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

const int EXPLORE = 1;
const int RESCUE = 2;
int mMission;

int main(int argc, char **argv) {
	
	ros::init(argc, argv, "MotherBrain");//Creates a node named "StrategyControl"
	ros::NodeHandle nodeHandle;

	// ros::Subscriber buttonOne = nodeHandle.subscribe("/StrategyControl/ButtonOne", 10, &buttonOneCallback);
	// ros::Subscriber buttonTwo = nodeHandle.subscribe("/StrategyControl/ButtonTwo", 10, &buttonTwoCallback);
	int c;
    while ((c = getopt (argc, argv, "re")) != -1) {
        switch (c) {
            case 'e': //explore (phase 1)
            	mMission = EXPLORE;
                break;
            case 'r': // rescue (phase 2)
            	mMission = RESCUE;
				break;
            case '?':
                if (optopt == 's')
                    printf("Option -%c requires an argument.\n", optopt);
                else if (isprint (optopt))
                    printf("Unknown option `-%c'.\n", optopt);
                else
                    printf("Unknown option character `\\x%x'.\n", optopt);
                return 1;
            default:
                abort();
        }
    }

    if (mMission == EXPLORE) {
    	printf("Mission: Explore maze");
    	executePhaseTwo(nodeHandle); 
    	printf("Mission: Explore maze - Accomplished!");
    } else if (mMission == RESCUE) {
    	printf("Mission: Resce tags");
    	executePhaseTwo(nodeHandle); 
    	printf("Mission: Rescue tags - Accomplished!");
    } else {
        printf("Mission: Unknown");
    }
	// Testing code.

	// executePhaseTwo(nodeHandle); 
	// cout << "done." << endl;

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