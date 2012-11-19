#ifndef STRATEGY_CLASSIFY_H
#define STRATEGY_CLASSIFY_H

#include "StrategyState.h"
#include "ros/ros.h"

namespace amee{
	class StrategyClassify : public StrategyState{
	public:
		StrategyClassify(ros::Publisher &pub);
		~StrategyClassify();

		virtual void init(const SensorData &data);
		virtual bool isRunning() const;
		virtual void doControl(const SensorData &data);
	private:
		bool mRunning;
		ros::Publisher mPub;
	}; //StrategyClassify class

}; //namespace amee

#endif //STRATEGY_CLASSIFY_H
