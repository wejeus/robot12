#ifndef MOVE_STRAIGHT_H
#define MOVE_STRAIGHT_H

#include "MovementState.h"
#include "ros/ros.h"

namespace amee {
	class MoveStraight : public MovementState {
		
		public:
			MoveStraight(ros::Publisher &pub);
			~MoveStraight();

			static const float DEFAULT_VELOCITY = 0.15f;

			virtual void init(const SensorData &data);
			void init(const float velocity = DEFAULT_VELOCITY);
			virtual bool isRunning() const;
			virtual void doControl(const SensorData &data);

		private:
			bool mRunning;
			ros::Publisher mPub;
			float mVelocity;
	}; //MoveStraight

}; //namespace amee

#endif //MOVE_STRAIGHT_H
