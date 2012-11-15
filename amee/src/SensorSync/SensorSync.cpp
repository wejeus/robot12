#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include "amee/IRDistances.h"
#include "amee/Odometry.h"
#include <string.h>
#include <iostream>
#include <queue>
#include <utility>

/*

a simple node that synchronizes two messages. That means:
Listen to /amee/motor_control/odometry and to /amee/sensors/irdistances and match the messages based on their timestamp(they won't be exactly equal, so you have to look for the minimal difference).

We'll need this kind of functionality for the mapper and maybe for other nodes as well. To spare roscore I think the best would be to copy the code later to the nodes where it's needed. It might be a good idea to design the code in a way that the message type is easily changeable ( assuming that every message has a field timestamp).
At the moment I'm just matching the last two received messages, but by taking a look at their timestamps I saw that this is not always working. So what the mapper needs is the IR distances measured at a certain known position. Without synchronizing it might happen that the mapper maps walls based on IR readings that were measured at a different position than the mapper thinks, what of course would result in an incorrect map.

*/

using namespace amee;
ros::Subscriber	ir_sub;
ros::Subscriber odo_sub;
//ros::Publisher cor_pub;

#define MAX_ELEM 20
#define MAX_MS_DIFF 0.1 //milisecs

std::queue<IRDistances> mIRList;
std::queue<Odometry> mOdoList;
std::queue<std::pair<IRDistances,Odometry> > mCorrList;

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
			++mIR_count; ++mOdo_count; ++pair_count;
		}else if(tmpIR.timestamp - tmpOdo.timestamp > 0){
			mOdoList.pop();
			++mOdo_count;
		}else{//if tmpIR.timestamp - tmpOdo.timestamp < 0
			mIRList.pop();
			++mIR_count;
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
	//cor_pub = n.advertise</*"TODO:add msg type"*/>("/amee/sensors/imu", 10000);

	ros::Rate loop_rate(10);
	struct timeval start, end;

	while(ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();
		doCorrection();

		loop_rate.sleep();
		printCorrections();
		loop_rate.sleep();
	}
	return 0;
}
