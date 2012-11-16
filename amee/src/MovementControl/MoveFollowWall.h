#ifndef MOVE_FOLLOW_WALL_H
#define MOVE_FOLLOW_WALL_H

#include "MovementState.h"
#include "ros/ros.h"

namespace amee{
	class MoveRotate;
	class MoveStraight;
	class MoveAlignWall;
	class MoveAlignToFrontWall;

	class MoveFollowWall : public MovementState {
	public:
		MoveFollowWall(ros::Publisher& pub, ros::Publisher& statesPub);
		~MoveFollowWall();

		virtual void init(const SensorData& data);
		virtual bool isRunning() const;
		virtual void doControl(const SensorData& data);

		static const float FOLLOWED_WALL = 1;
		static const float FOUND_END_OF_WALL =  2;
		static const float MOVED_TAIL = 3;
		static const float ROTATED_RIGHT = 4;
		static const float ROTATED_LEFT = 5;
		static const float FOUND_BEGINNING_OF_WALL = 6;
		static const float ALIGNED_TO_WALL = 7;

	private:
		enum WallFollowState {FollowWall, LookForEndOfWall, MoveTail,
			RotateRight, RotateLeft, LookForBeginningOfWall, AlignToWall, HandleEvilWalls, AlignToFrontWall, TIntersectionHandling};

		struct State
		{
			WallFollowState mState;
			bool initialized;
			void set(WallFollowState state) {
				mState = state;
				initialized = false;
			}
		};

		static const float MOVEMENT_SPEED = 0.25;
		static const float MAX_ROTATION_SPEED = 0.1;
		static const float MIN_ROTATION_SPEED = 0.06;
		static const float IR_BASE_RIGHT = 0.104;

		static const float MIN_WALL_DISTANCE = 0.03f;
		static const float MAX_WALL_DISTANCE = 0.05f;

		static const float TAIL_LENGTH = 0.09f;

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
		// allows us to align the robot to a front wall
		amee::MoveAlignToFrontWall *mFrontWallAligner;

		ros::Publisher mVelPub;
		ros::Publisher mFollowWallStatesPub;

		bool mRunning;

		// variables for lookForBeginningOfWall
		bool mFoundWallRightBack;
		bool mFoundWallRightFront;

		// variable for WallDetection
		float mSeenWallStartDist;

		// states
		void followWallState();
		void lookForEndOfWallState();
		void moveTailState();
		void rotateRightState();
		void rotateLeftState();
		void lookForBeginningOfWallState();
		void alignToWallState();
		void alignToFrontWallState();
		void tIntersectionHandlingState();
		void handleEvilWallsState();

		// Publish states
    	void PublishState(int state);

		// control function for wall following
		void followWall();

		// helper functions
		bool seesWall(float distance);
		bool wallInFront();
		bool frontAlignmentPossible();

		bool nextToWall();

		// function to publish speeds
		void publishSpeeds(float left, float right);



		
	}; //MoveFollowWall

}; //namespace amee

#endif //MOVE_FOLLOW_WALL_H
