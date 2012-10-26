#ifndef MOVE_STRAIGHT_H
#define MOVE_STRAIGHT_H

#include "MovementState.h"
#include "amee/Velocity.h"

namespace amee{
	class MoveStraight : public MovementState{
	#define DEFAULT_VELOCITY 0.15
	public:
		MoveStraight();
		MoveStraight(ros::Publisher &);
		~MoveStraight();

		virtual void init(SensorData &data);
		void init(const float velocity);
		virtual const bool& isRunning() const;
		virtual void doControl(SensorData &data);
		const float & getCurrentVelocity()const;
	private:
		bool mRunning;
		ros::Publisher mPub;
		float mVelocity;
	}; //MoveStraight

}; //namespace amee

#endif //MOVE_STRAIGHT_H
