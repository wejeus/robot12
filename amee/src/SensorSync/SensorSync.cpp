#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include "amee/IRDistances.h"
#include "amee/Odometry.h"
#include <string.h>
#include <iostream>
#include <queue>
#include <utility>

/**
 * A simple node that synchronizes two messages. That means:
 * Listen to /amee/motor_control/odometry and to /amee/sensors/irdistances and match the messages based on their timestamp (they won't be exactly equal, so we are looking for those that are within a threshold, and drop the rest).
*/

#ifndef DEBUG
#define DEBUG 1 //set this to 1 or 0 to enable and disable the print outs
#endif

using namespace amee;
ros::Subscriber	ir_sub;
ros::Subscriber odo_sub;
//ros::Publisher cor_pub;

#define MAX_ELEM 20
#define MAX_MS_DIFF 0.1 //milisecs

std::queue<IRDistances> mIRList;
std::queue<Odometry> mOdoList;
std::queue<std::pair<IRDistances,Odometry> > mCorrList;

#if DEBUG == 1
int mIR_count = 0, mOdo_count = 0, pair_count = 0;

void printCorrections(){
	while(!mCorrList.empty()){
		std::pair<IRDistances, Odometry> p = mCorrList.front();
		mCorrList.pop();
		std::cout << "Pair time-diff: " << fabs(p.first.timestamp - p.second.timestamp)
			  << "\tIRs drop: "   << mIR_count - pair_count
			  << "\tOdos drop: "  << mOdo_count - pair_count
			  << "\tPairs: " << pair_count << std::endl;
	}
}
#endif

/**
 * Stable marriage problem? Naah, its too overkill...
 */
void doCorrection(){
	IRDistances tmpIR; Odometry tmpOdo;
	while(mIRList.size() > 1 && mOdoList.size() > 1){

		tmpIR = mIRList.front();
		tmpOdo = mOdoList.front();

		if(fabs(tmpIR.timestamp - tmpOdo.timestamp) < MAX_MS_DIFF){
			mIRList.pop(); mOdoList.pop();
			std::pair<IRDistances, Odometry> p = std::make_pair(tmpIR,tmpOdo);
			mCorrList.push(p);
#if DEBUG == 1
			++mIR_count; ++mOdo_count; ++pair_count;
#endif
		}else if(tmpIR.timestamp - tmpOdo.timestamp > 0){
			mOdoList.pop();
#if DEBUG == 1
			++mOdo_count;
#endif
		}else{//if tmpIR.timestamp - tmpOdo.timestamp < 0
			mIRList.pop();
#if DEBUG == 1
			++mIR_count;
#endif
		}
	}
}

void receive_ir(const IRDistances::ConstPtr &msg){
	mIRList.push(*msg);
}

void receive_odo(const Odometry::ConstPtr &msg){
	mOdoList.push(*msg);
}

int main(int argc, char **argv){
	ros::init(argc, argv, "SensorSync");
	ros::NodeHandle n;
	ir_sub = n.subscribe("/amee/sensors/irdistances", 10000, receive_ir);
	odo_sub = n.subscribe("/amee/motor_control/odometry", 10000, receive_odo);

	ros::Rate loop_rate(10);
/*	struct timeval start, end;*/

	while(ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();
		doCorrection();

#if DEBUG == 1
		loop_rate.sleep();
		printCorrections();
		loop_rate.sleep();
#endif
	}
	return 0;
}
