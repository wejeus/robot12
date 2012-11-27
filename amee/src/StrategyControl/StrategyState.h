#ifndef STRATEGY_STATE_H
#define STRATEGY_STATE_H
#include "amee/Pose.h"
#include "amee/GraphMsg.h"
#include "amee/MapperEvent.h"
#include "amee/MovementEvent.h"
#include "ros/ros.h"


namespace amee{

//	enum STATE{ MOVE_STOP, MOVE_STRAIGHT, MOVE_ROTATE, MOVE_COORDINATE, MOVE_FOLLOW_WALL, MOVE_LOST};

	struct StrategyData {
		float x;
		float y;
		float theta;
	};

	class StrategyState{
	public:
		//Constructors & destructor
		StrategyState(){};
		virtual ~StrategyState(){};


		// virtual void init(const StrategyData& data) = 0;
		virtual bool isRunning() const = 0;
		// virtual void doControl(const StrategyData& data) = 0;
		virtual void receive_pose(const amee::Pose::ConstPtr &msg) = 0;
		virtual void receive_graph(const amee::GraphMsg::ConstPtr &msg) = 0;
		virtual void receive_mapper_event(const amee::MapperEvent::ConstPtr &msg) = 0;
		virtual void receive_movement_event(const amee::MovementEvent::ConstPtr &msg) = 0;
		virtual void receive_timerP1(const ros::TimerEvent &event) = 0;
		virtual void receive_timerP2(const ros::TimerEvent &event) = 0;
		
	}; //StrategyState

}; //namespace amee

#endif
