#ifndef STRATEGY_EXPLORE_H
#define STRATEGY_EXPLORE_H

#include "StrategyState.h"
#include "ros/ros.h"

namespace amee{
	class StrategyExplore : public StrategyState{
	public:
		StrategyExplore(ros::Publisher &pub);
		~StrategyExplore();

		virtual void init(const SensorData &data);
		virtual bool isRunning() const;
		virtual void doControl(const SensorData &data);
	private:
		bool mRunning;
		ros::Publisher mPub;
	}; //StrategyExplore class

}; //namespace amee

#endif //STRATEGY_EXPLORE_H
