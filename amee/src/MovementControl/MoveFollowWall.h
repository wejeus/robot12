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
			RotateRight, RotateLeft, LookForBeginningOfWall, AlignToWall, HandleEvilWalls, AlignToFrontWall, TIntersectionHandling, EdgeOfWall};

		struct State
		{
			WallFollowState mState;
			bool initialized;
			void set(WallFollowState state) {
				mState = state;
				initialized = false;
			}
		};

		static const float MOVEMENT_SPEED = 0.2f;
		static const float SLOW_MOVEMENT_SPEED = 0.08f;
		static const float MAX_ROTATION_SPEED = 0.1;
		static const float MIN_ROTATION_SPEED = 0.06;
		static const float IR_BASE_RIGHT = 0.104;
		static const float IR_ERROR_THRESHOLD = 0.02f;

		static const float K_p_keepRef = 0.8f;
        static const float K_p_reachRef = 0.3f;
        static const float K_i_keepRef = 0.0f;
        static const float K_i_reachRef = 0.0f;


        static const float MAX_ERROR_SUM = 100.0f;
        static const float REF_DISTANCE = 0.04f;
        static const float NO_WALL_DISTANCE = 0.15f;
        static const float WALL_DIST_TOL = 0.01f;

		static const float MIN_WALL_DISTANCE = 0.03f;
		static const float MAX_WALL_DISTANCE = 0.05f;

		static const float TAIL_LENGTH = 0.09f;


		float maxErrorSum;
		
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
		void edgeOfWallState();

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
