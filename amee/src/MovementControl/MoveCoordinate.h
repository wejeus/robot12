#ifndef MOVE_COORDINATE_H
#define MOVE_COORDINATE_H

#include "MovementState.h"
#include "ros/ros.h"

namespace amee{
	class MoveCoordinate : public MovementState{
	public:
		MoveCoordinate(ros::Publisher &pub);
		~MoveCoordinate();

		virtual void init(const SensorData &data);
		virtual bool isRunning() const;
		virtual void doControl(const SensorData &data);
	private:
		bool mRunning;
		ros::Publisher mPub;
	}; //MoveCoordinate

}; //namespace amee

#endif //MOVE_COORDINATE_H
