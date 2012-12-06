#ifndef MOVE_COLLISION_RECOVERY_H
#define MOVE_COLLISION_RECOVERY_H

#include "MovementState.h"
#include "ros/ros.h"

namespace amee{
	class MoveAlignWall;
	class MoveStraight;
	class MoveRotate;

	class MoveCollisionRecovery : public MovementState {
	public:
		MoveCollisionRecovery(ros::Publisher& pub, ros::Publisher& event_pub);
		~MoveCollisionRecovery();

		virtual void init(const SensorData& data);
		virtual bool isRunning() const;
		virtual void doControl(const SensorData& data);

		static const float BACKWARDS_DIST = 0.02f;
		static const float MAX_ANGLE = 180.0f;
	
	private:
		ros::Publisher mSpeedPub;
		ros::Publisher mEventPub;
		MoveAlignWall* mWallAligner;
		MoveStraight* mStraightMover;
		MoveRotate* mRotater;
		
		bool mIsRunning;
		bool mRotaterInitialized;
		bool mAlignerInitialized;
		// bool mRefDistReached;
		// bool mCanAlign;
		// float mRefDist;
		
		bool collisionAhead(const SensorData& data);
		void publishSpeeds(float left, float right);
		bool wallOnRight(const SensorData& data);

	}; 

}; //namespace amee

#endif //MOVE_COLLISION_RECOVERY_H
