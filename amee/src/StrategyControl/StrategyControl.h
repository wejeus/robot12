#ifndef STRATEGY_CONTROL_H
#define STRATEGY_CONTROL_H

#include "amee/Pose.h"
#include "../Graph/Graph.h"
#include "amee/GraphMsg.h"
#include "amee/StrategyCommand.h"
#include "amee/MapperEvent.h"
#include "amee/MovementEvent.h"
#include "StrategyState.h"

namespace amee {
// class StrategyClassify;
// class StrategyExplore;
class StrategyGoTo;
class StrategyRescue;
class StrategyExplore;

class StrategyControl {

	public:
		StrategyControl(ros::Publisher& pub, ros::Publisher &phaseInfo, ros::Publisher &pathPub);
		~StrategyControl();
		void receive_command(const amee::StrategyCommand::ConstPtr &msg);
		void receive_pose(const amee::Pose::ConstPtr &msg);
		void receive_graph(const amee::GraphMsg::ConstPtr &msg);
		void receive_mapper_event(const amee::MapperEvent::ConstPtr &msg);
		void receive_movement_event(const amee::MovementEvent::ConstPtr &msg);
		void receive_timerP1(const ros::TimerEvent &event);
		void receive_timerP2(const ros::TimerEvent &event);

		// void doControl();
		void init();

		static const int TYPE_STRATEGY_CLASSIFY = 1;
		static const int TYPE_STRATEGY_EXPLORE = 2;
		static const int TYPE_STRATEGY_GET_OUT = 3;
		static const int TYPE_STRATEGY_GO_TO = 4;
		static const int TYPE_STRATEGY_RESCUE = 5;

	private:
		
		// amee::StrategyData mStrategyData;
		amee::GraphMsg::ConstPtr mGraphMsg;
		amee::Pose mPose;

		ros::Publisher speed_pub;

		amee::StrategyState* mCurrentState;

		// amee::StrategyClassify* mClassifyState;
		// amee::StrategyExplore* mExploreState;
		amee::StrategyGoTo* mGoToState;
		amee::StrategyRescue* mRescueState;

};

};//namespace amee
#endif
