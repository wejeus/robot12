#ifndef STRATEGY_GET_OUT_H
#define STRATEGY_GET_OUT_H

#include "StrategyState.h"
#include "../Graph/Graph.h"
#include "../Graph/PathFinderAlgo.h"
#include "amee/Pose.h"
#include "ros/ros.h"
#include <queue>

namespace amee{
	class StrategyGetOut : public StrategyState {
	public:
		StrategyGetOut(ros::Publisher &pub);
		~StrategyGetOut();

		virtual void init(const StrategyData &data);
		void init(const StrategyData &data, amee::GraphMsg::ConstPtr& graphMsg);
		virtual bool isRunning() const;
		virtual void doControl(const StrategyData &data);

		static const float EUCLIDEAN_POSITION_DISTANCE = 0.1f;

	private:
		bool mRunning;
		bool mGoingOut;
		ros::Publisher mPub;
		amee::Graph mGraph;
		std::queue<amee::Pose> mPath;

		StrategyData mStrategyData;

		inline float EuclidDist(const Pose& p, const float& x, const float& y) const;

	}; //StrategyGetOut class

}; //namespace amee

#endif //STRATEGY_GET_OUT_H
