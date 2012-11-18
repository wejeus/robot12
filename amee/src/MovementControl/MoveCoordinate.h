#ifndef MOVE_COORDINATE_H
#define MOVE_COORDINATE_H

#include "MovementState.h"
#include "ros/ros.h"

namespace amee{
	class MoveRotate;
	class MoveStraight;

	class MoveCoordinate : public MovementState{
	public:
		MoveCoordinate(ros::Publisher &pub);
		~MoveCoordinate();

		virtual void init(const SensorData &data);
		virtual bool isRunning() const;
		virtual void doControl(const SensorData &data);
	private:
		bool mRunning;
		float x, y;
		ros::Publisher mPub;

		// allows us to rotate by calling its doControl after we initialized it as long as we want to rotate
		amee::MoveRotate *mRotater;
		// allows us to move straight
		amee::MoveStraight *mStraightMove;
		
	}; //MoveCoordinate

}; //namespace amee

#endif //MOVE_COORDINATE_H
