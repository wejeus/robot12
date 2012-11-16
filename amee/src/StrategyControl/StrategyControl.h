#ifndef STRATEGY_CONTROL_H
#define STRATEGY_CONTROL_H

#include "amee/IRDistances.h"
#include "amee/Odometry.h"
#include "amee/StrategyCommand.h"
#include "roboard_drivers/sonar.h"
#include "StrategyState.h"

namespace amee {
class StrategyClassify;
class StrategyExplore;
class StrategyGo2Tag;

class StrategyControl {

	public:
		StrategyControl(ros::Publisher& pub, ros::Publisher& statespub);
		~StrategyControl();
		void receive_distances(const amee::IRDistances::ConstPtr &msg);
		void receive_odometry(const amee::Odometry::ConstPtr &msg);
		void receive_command(const amee::StrategyCommand::ConstPtr &msg);
		void receive_sonar(const roboard_drivers::sonar::ConstPtr &msg);
		void setSpeedPublisher(ros::Publisher& pub);
		void doControl();
		void init();

		static const int TYPE_STRATEGY_CLASSIFY = 1;
		static const int TYPE_STRATEGY_EXPLORE = 2;
		static const int TYPE_STRATEGY_GO2TAG = 3;

	private:
//		amee::SensorData mSensorData;

		ros::Publisher speed_pub;

		amee::StrategyState* mCurrentState;

		amee::StrategyClassify* mClassifyState;
		amee::StrategyExplore* mExploreState;
		amee::StrategyGo2Tag* mGo2TagState;
};
}
#endif
