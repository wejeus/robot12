#ifndef STRATEGY_GO_2_TAG_H
#define STRATEGY_GO_2_TAG_H

#include "StrategyState.h"
#include "ros/ros.h"

namespace amee{
	class StrategyGo2Tag : public StrategyState{
	public:
		StrategyGo2Tag(ros::Publisher &pub);
		~StrategyGo2Tag();

		virtual void init(const SensorData &data);
		virtual bool isRunning() const;
		virtual void doControl(const SensorData &data);
	private:
		bool mRunning;
		ros::Publisher mPub;
	}; //StrategyGo2Tag class

}; //namespace amee

#endif //STRATEGY_GO_2_TAG_H
