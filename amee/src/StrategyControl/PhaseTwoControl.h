#ifndef PHASE_TWO_CONTROL_H
#define PHASE_TWO_CONTROL_H

#include "ros/ros.h"
#include "amee/Pose.h"
#include "../Graph/Graph.h"
#include <fstream>
#include <iostream>
#include "amee/GraphMsg.h"
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <stdlib.h>

namespace amee {

class PhaseTwoControl {

	public:
		PhaseTwoControl(ros::NodeHandle &nodeHandle);
		~PhaseTwoControl();
		void phaseInfoCallback(const std_msgs::Int32 &msg);
		void rescue();
		bool isRunning() { return mIsRunning; }

	private:
		ros::Subscriber mPhaseInfo;
		ros::Publisher mStrategyControl;
		bool mIsRunning;
		Graph *mGraph;
};

};//namespace amee
#endif