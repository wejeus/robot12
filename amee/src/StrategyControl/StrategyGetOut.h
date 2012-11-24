#ifndef STRATEGY_GET_OUT_H
#define STRATEGY_GET_OUT_H

#include "StrategyState.h"
#include "ros/ros.h"

namespace amee{
	class StrategyGetOut : public StrategyState{
	public:
		StrategyGetOut(ros::Publisher &pub);
		~StrategyGetOut();

		virtual void init(const StrategyData &data);
		virtual bool isRunning() const;
		virtual void doControl(const StrategyData &data);
	private:
		bool mRunning;
		ros::Publisher mPub;
	}; //StrategyGetOut class

}; //namespace amee

#endif //STRATEGY_GET_OUT_H
