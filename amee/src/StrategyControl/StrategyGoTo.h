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
		StrategyGoTo(ros::Publisher &pub, ros::Publisher &phaseInfo, ros::Publisher &pathPub);
		~StrategyGoTo();

		// virtual void init(const StrategyData &data);
		// void init(const StrategyData &data, const amee::GraphMsg::ConstPtr& graphMsg);
		// void init(const StrategyData &data, const amee::GraphMsg::ConstPtr& graphMsg, const float& x, const float& y);
		void init(const amee::Pose &pose, const amee::GraphMsg::ConstPtr& graphMsg, const unsigned int& id);
		virtual bool isRunning() const;
		virtual void receive_pose(const amee::Pose::ConstPtr &msg);
		virtual void receive_graph(const amee::GraphMsg::ConstPtr &msg);
		virtual void receive_mapper_event(const amee::MapperEvent::ConstPtr &msg);
		virtual void receive_movement_event(const amee::MovementEvent::ConstPtr &msg);

		static const float NODE_REACHED_DISTANCE = 0.07f;

	private:
		bool mRunning;

		enum GoToState {
			Start, MoveCoordinate, Rotate, Align, FollowWall, FinalRotate, CollisionRecovery
		};

		GoToState mState;
		
		// bool mRestartFollowingWall;
		// bool mFollowingWall;
		ros::Publisher mCommandPub;
		ros::Publisher mPhaseInfo;
		ros::Publisher mPathPub;
		amee::Graph mGraph;
		std::queue<amee::NodeMsg> mPath;
		amee::Pose mPose;
		unsigned int mTargetId;
		int mUnknownNodeCounter;

		StrategyData mStrategyData;

		void moveToNextWaypoint();
		void handleCollision();
		void startFollowWall();
		void startMoveCoordinate(Pose& pose);
		void stop();
		void reinit();
		float getAngleChange(float from, float to);
		inline float EuclidDist(const Pose& p1, const Pose& p2) const;

	}; //StrategyGoTo class

}; //namespace amee

#endif //STRATEGYGO_TO_H
