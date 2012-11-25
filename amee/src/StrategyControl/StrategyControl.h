#ifndef STRATEGY_CONTROL_H
#define STRATEGY_CONTROL_H

#include "amee/Pose.h"
#include "../Graph/Graph.h"
#include "amee/GraphMsg.h"
#include "amee/StrategyCommand.h"
#include "StrategyState.h"

namespace amee {
class StrategyClassify;
class StrategyExplore;
class StrategyGoTo;

class StrategyControl {

	public:
		StrategyControl(ros::Publisher& pub);
		~StrategyControl();
		void receive_command(const amee::StrategyCommand::ConstPtr &msg);
		void receive_pose(const amee::Pose::ConstPtr &msg);
		void receive_graph(const amee::GraphMsg::ConstPtr &msg);
		void doControl();
		void init();

		static const int TYPE_STRATEGY_CLASSIFY = 1;
		static const int TYPE_STRATEGY_EXPLORE = 2;
		static const int TYPE_STRATEGY_GET_OUT = 3;
		static const int TYPE_STRATEGY_GO_TO = 4;

	private:
		bool mMapInitialized;

		amee::StrategyData mStrategyData;
		amee::GraphMsg::ConstPtr mGraphMsg;

		ros::Publisher speed_pub;

		amee::StrategyState* mCurrentState;

		amee::StrategyClassify* mClassifyState;
		amee::StrategyExplore* mExploreState;
		amee::StrategyGoTo* mGoToState;
};

};//namespace amee
#endif
