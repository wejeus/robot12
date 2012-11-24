#ifndef STRATEGY_CONTROL_H
#define STRATEGY_CONTROL_H

#include "amee/Pose.h"
#include "amee/StrategyCommand.h"
#include "StrategyState.h"

namespace amee {
class StrategyClassify;
class StrategyExplore;
class StrategyGo2Tag;
class StrategyGetOut;

class StrategyControl {

	public:
		StrategyControl(ros::Publisher& pub);
		~StrategyControl();
		void receive_command(const amee::StrategyCommand::ConstPtr &msg);
		void receive_pose(const amee::Pose::ConstPtr &msg);
		void doControl();
		void init();

		static const int TYPE_STRATEGY_CLASSIFY = 1;
		static const int TYPE_STRATEGY_EXPLORE = 2;
		static const int TYPE_STRATEGY_GO2TAG = 3;
		static const int TYPE_STRATEGY_GET_OUT = 4;

	private:
		amee::StrategyData mStrategyData;

		ros::Publisher speed_pub;

		amee::StrategyState* mCurrentState;

		amee::StrategyClassify* mClassifyState;
		amee::StrategyExplore* mExploreState;
		amee::StrategyGo2Tag* mGo2TagState;
		amee::StrategyGetOut* mGetOutState;
};
}
#endif
