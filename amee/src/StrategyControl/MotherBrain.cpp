#include "ros/ros.h"

#include <stdio.h>
#include <iostream>
#include <std_msgs/Int32.h>
#include "PhaseOneControl.h"
#include "PhaseTwoControl.h"
#include <unistd.h>
#include "amee/StrategyCommand.h"


using namespace std;
using namespace amee;

enum PHASE {ONE = 1, TWO = 2};
enum STATE {IDLE, RUN};

PHASE mPhase = ONE;
STATE mState = IDLE;

// void executePhaseOne();
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

void executePhaseOne(ros::NodeHandle node) {
	PhaseOneControl phaseControl;

    ros::Subscriber pose_sub = n.subscribe("/amee/pose", 5, &PhaseOneControl::receivePose, &phaseControl);
    ros::Subscriber state_sub = n.subscribe("/amee/follow_wall_states",10, &PhaseOneControl::receive_FollowWallState, &mapper);
    ros::Subscriber tag_sub = n.subscribe("/amee/tag",10, &Mapper::receive_tag, &mapper);
    ros::Subscriber command_sub = n.subscribe("/amee/map/mapper_commands",10, &Mapper::receive_MapperCommand, &mapper);

    ros::Publisher pose_pub = n.advertise<amee::Pose>("/amee/pose",5);
    ros::Publisher marker_pub = n.advertise<amee::MapVisualization>("/amee/map/visualization", 10);
    ros::Publisher graph_pub = n.advertise<amee::GraphMsg>("/amee/map/graph",10);
}

PhaseTwoControl *phaseTwoControl;

void executePhaseTwo() {
	if (phaseTwoControl->isRunning()) {
		phaseTwoControl->rescue();
	}
}

const int EXPLORE = 1;
const int RESCUE = 2;
int mMission;


int main(int argc, char **argv) {
	
	ros::init(argc, argv, "MotherBrain");//Creates a node named "StrategyControl"
	ros::NodeHandle nodeHandle;

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
    	printf("Mission: Explore maze\n");
        executePhaseOne(nodeHandle);
    	printf("Mission: Explore maze - Accomplished!\n");
    } else if (mMission == RESCUE) {
    	printf("Mission: Rescue tags\n");
        ros::Publisher sp = nodeHandle.advertise<StrategyCommand>("/StrategyControl/StrategyCommand", 1);
        phaseTwoControl = new PhaseTwoControl(nodeHandle, sp);
        printf("Mission: Rescue tags - Accomplished!\n");
    } else {
        printf("Mission: Unknown\n");
    }
	// Testing code.

	// executePhaseTwo(nodeHandle); 
	// cout << "done." << endl;

	// ros::Rate loop_rate(6);
	// while(ros::ok()) {
 //        cout << "." << endl;
 //        executePhaseTwo();
	// 	// go to sleep for a short while
	// 	loop_rate.sleep();
	// 	// call all callbacks
	// 	ros::spinOnce();
	// }

	return 0;
}