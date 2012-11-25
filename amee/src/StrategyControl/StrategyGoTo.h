#ifndef STRATEGYGO_TO_H
#define STRATEGYGO_TO_H

#include "StrategyState.h"
#include "../Graph/Graph.h"
#include "../Graph/PathFinderAlgo.h"
#include "amee/Pose.h"
#include "ros/ros.h"
#include <queue>

namespace amee{
	class StrategyGoTo : public StrategyState {
	public:
		StrategyGoTo(ros::Publisher &pub);
		~StrategyGoTo();

		virtual void init(const StrategyData &data);
		void init(const StrategyData &data, const amee::GraphMsg::ConstPtr& graphMsg);
		void init(const StrategyData &data, const amee::GraphMsg::ConstPtr& graphMsg, const float& x, const float& y);
		virtual bool isRunning() const;
		virtual void doControl(const StrategyData &data);

		static const float EUCLIDEAN_POSITION_DISTANCE = 0.1f;

	private:
		bool mRunning;
		ros::Publisher mPub;
		amee::Graph mGraph;
		std::queue<amee::Pose> mPath;

		StrategyData mStrategyData;

		inline float EuclidDist(const Pose& p, const float& x, const float& y) const;

	}; //StrategyGoTo class

}; //namespace amee

#endif //STRATEGYGO_TO_H
