#ifndef MOVE_STOP_H
#define MOVE_STOP_H

#include "MovementState.h"
#include "ros/ros.h"

namespace amee {
	class MoveStop : public MovementState {
		
		public:
			MoveStop(ros::Publisher &pub);
			~MoveStop();

			virtual void init(const SensorData &data);
			virtual bool isRunning() const;
			virtual void doControl(const SensorData &data);

		private:
			ros::Publisher mPub;
	}; //MoveStraight

}; //namespace amee

#endif //MOVE_STOP_H
