#ifndef MOVE_FOLLOW_WALL_H
#define MOVE_FOLLOW_WALL_H

#include "MovementState.h"
#include "ros/ros.h"

namespace amee{
	class MoveRotate;
	class MoveStraight;
	class MoveAlignWall;

	class MoveFollowWall : public MovementState {
	public:
		MoveFollowWall(ros::Publisher& pub);
		~MoveFollowWall();

		virtual void init(const SensorData& data);
		virtual bool isRunning() const;
		virtual void doControl(const SensorData& data);
		
	private:
		enum WallFollowState {FollowWall, LookForEndOfWall, MoveTail,
			RotateRight, RotateLeft, LookForBeginningOfWall, AlignToWall};

		struct State
		{
			WallFollowState mState;
			bool initialized;
			void set(WallFollowState state) {
				mState = state;
				initialized = false;
			}
		};

		static const float MOVEMENT_SPEED = 0.3;
		static const float MAX_ROTATION_SPEED = 0.1;
		static const float MIN_ROTATION_SPEED = 0.06;
		static const float IR_BASE_RIGHT = 0.104;

		static const float TOO_CLOSE_TO_WALL = 0.03f;
		static const float TOO_FAR_FROM_WALL = 0.08f;

		float linearSpeed;
		float K_p_keepRef;
		float K_p_reachRef;
		float K_i_keepRef;
		float K_i_reachRef;

		float K_d;

		float maxErrorSum;
		float refDistance;
		float noWallDistance;
		float wallDistTol;
		float error_sum;
		float last_error;
		
		State mState; // current state of the wall follower		
		SensorData mSensorData;

		// allows us to rotate by calling its doControl after we initialized it as long as we want to rotate
		amee::MoveRotate *mRotater;
		// allows us to move straight
		amee::MoveStraight *mStraightMove;
		// allows us to align the robot parallel to the wall
		amee::MoveAlignWall *mWallAligner;

		ros::Publisher mVelPub;

		bool mRunning;

		// variables for lookForBeginningOfWall
		bool mFoundWallRightBack;
		bool mFoundWallRightFront;

		// states
		void followWallState();
		void lookForEndOfWallState();
		void moveTailState();
		void rotateRightState();
		void rotateLeftState();
		void lookForBeginningOfWallState();
		void alignToWall();

		// control function for wall following
		void followWall();

		// helper functions
		bool seesWall(float distance);
		bool wallInFront();

		// function to publish speeds
		void publishSpeeds(float left, float right);



		
	}; //MoveFollowWall

}; //namespace amee

#endif //MOVE_FOLLOW_WALL_H
