#ifndef MOVE_ROTATE_H
#define MOVE_ROTATE_H

#include "MovementState.h"
#include "ros/ros.h"

namespace amee{
	class MoveRotate : public MovementState {
	public:
		MoveRotate(ros::Publisher& pub, ros::Publisher& movement_event_pub);
		MoveRotate(ros::Publisher& pub);
		~MoveRotate();

		/**
			Initializes this rotation movement with default values:
				rotation 90 degrees.
		**/
		virtual void init(const SensorData& data);
		void init(const SensorData& data, const float& degrees);
		virtual bool isRunning() const;
		virtual void doControl(const SensorData& data);
	
	private:
		ros::Publisher mSpeedPub;
		ros::Publisher mEventPub;
		float mTargetAngle;
		float mIsRotating;
		float mCurrentRelativeAngle;
		float mStartingAngle;
		bool mStartedRotation;
		bool mPublishEvents;

		void publishSpeeds(float left, float right);

	}; //MoveRotate

}; //namespace amee

#endif //MOVE_ROTATE_H
