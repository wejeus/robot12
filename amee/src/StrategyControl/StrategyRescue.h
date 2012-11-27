#ifndef STRATEGY_RESCUE_H
#define STRATEGY_RESCUE_H

#include "StrategyState.h"
#include "../Graph/Graph.h"
#include "../Graph/PathFinderAlgo.h"
#include "amee/Pose.h"
#include <std_msgs/Int32.h>
#include "amee/NodeMsg.h"
#include "ros/ros.h"
#include <queue>
#include "StrategyGoTo.h"

namespace amee{
	class StrategyGoTo;

	class StrategyRescue : public StrategyState {
	public:
		StrategyRescue(ros::Publisher &pub, ros::Publisher &phaseInfo, ros::Publisher &pathPub);
		~StrategyRescue();

		// virtual void init(const StrategyData &data);
		// void init(const StrategyData &data, const amee::GraphMsg::ConstPtr& graphMsg);
		// void init(const StrategyData &data, const amee::GraphMsg::ConstPtr& graphMsg, const float& x, const float& y);
		void init(const amee::Pose &pose, const amee::GraphMsg::ConstPtr& graphMsg);
		virtual bool isRunning() const;
		virtual void receive_pose(const amee::Pose::ConstPtr &msg);
		virtual void receive_graph(const amee::GraphMsg::ConstPtr &msg);
		virtual void receive_mapper_event(const amee::MapperEvent::ConstPtr &msg);
		virtual void receive_movement_event(const amee::MovementEvent::ConstPtr &msg);
		virtual void receive_timerP1(const ros::TimerEvent &event);
		virtual void receive_timerP2(const ros::TimerEvent &event);


		static const float EUCLIDEAN_POSITION_DISTANCE = 0.035f;


	private:
		bool mRunning;
		// bool mRestartFollowingWall;
		bool mFollowingWall;
		ros::Publisher mCommandPub;
		ros::Publisher mPhaseInfo;
		ros::Publisher mPathPub;
		amee::Graph mGraph;
		std::queue<amee::NodeMsg> mPath;
		amee::Pose mPose;

		StrategyData mStrategyData;
		StrategyGoTo* mStrategyGoTo;

		void moveToNextWaypoint();
		void stop();
		inline float EuclidDist(const Pose& p, const float& x, const float& y) const;

	}; //StrategyGoTo class

}; //namespace amee

#endif //STRATEGY_RESCUE_H
