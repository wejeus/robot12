#ifndef PHASE_TWO_CONTROL_H
#define PHASE_TWO_CONTROL_H

#include "ros/ros.h"
#include "amee/Pose.h"
#include "../Graph/Graph.h"
#include <fstream>
#include <iostream>
#include "amee/GraphMsg.h"
#include <std_msgs/Int32.h>
#include "amee/StrategyCommand.h"
#include "StrategyControl.h"
#include <stdio.h>
#include <stdlib.h>
#include <vector>

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
		ros::Publisher mStrategyCommand;
		bool mIsRunning;
		bool mIsFinishing;
		bool mTargetReached;
		Graph *mGraph;
		std::vector<int> mTargets;
		int mNextTargetNode;
		int mCurrentNode;
};

};//namespace amee
#endif