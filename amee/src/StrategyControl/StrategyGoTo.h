#ifndef STRATEGYGO_TO_H
#define STRATEGYGO_TO_H

#include "StrategyState.h"
#include "../Graph/Graph.h"
#include "../Graph/PathFinderAlgo.h"
#include "amee/Pose.h"
#include <std_msgs/Int32.h>
#include "amee/NodeMsg.h"
#include "ros/ros.h"
#include <queue>

namespace amee{
	class StrategyGoTo : public StrategyState {
	public:
		StrategyGoTo(ros::Publisher &pub, ros::Publisher &phaseInfo);
		~StrategyGoTo();

		virtual void init(const StrategyData &data);
		void init(const StrategyData &data, const amee::GraphMsg::ConstPtr& graphMsg);
		void init(const StrategyData &data, const amee::GraphMsg::ConstPtr& graphMsg, const float& x, const float& y);
		void init(const StrategyData &data, const amee::GraphMsg::ConstPtr& graphMsg, const unsigned int& id);
		virtual bool isRunning() const;
		virtual void doControl(const StrategyData &data);

		static const float EUCLIDEAN_POSITION_DISTANCE = 0.035f;

	private:
		bool mRunning;
		bool mRestartFollowingWall;
		bool mFollowWallPossible;
		ros::Publisher mPub;
		ros::Publisher mPhaseInfo;
		amee::Graph mGraph;
		std::queue<amee::NodeMsg> mPath;

		StrategyData mStrategyData;

		inline float EuclidDist(const Pose& p, const float& x, const float& y) const;

	}; //StrategyGoTo class

}; //namespace amee

#endif //STRATEGYGO_TO_H
