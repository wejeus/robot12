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

		// Constants for state change messages.
		static const int FOLLOW_WALL_IN = 0;
		static const int FOLLOW_WALL_OUT = 1;
		static const int LOOK_FOR_END_OF_WALL_IN = 2;
		static const int LOOK_FOR_END_OF_WALL_OUT = 3;
		static const int MOVE_TAIL_IN =  4;
		static const int MOVE_TAIL_OUT = 5;
		static const int ROTATE_RIGHT_IN = 6;
		static const int ROTATE_RIGHT_OUT = 7;
		static const int ROTATE_LEFT_IN = 8;
		static const int ROTATE_LEFT_OUT = 9;
		static const int LOOK_FOR_BEGINNING_OF_WALL_IN = 10;
		static const int LOOK_FOR_BEGINNING_OF_WALL_OUT = 11;
		static const int ALIGN_TO_WALL_IN = 12;
		static const int ALIGN_TO_WALL_OUT = 13;
		static const int ALIGN_TO_FRONT_WALL_IN = 14;
		static const int ALIGN_TO_FRONT_WALL_OUT = 15;
		static const int HANDLE_EVIL_WALLS_IN = 16;
		static const int HANDLE_EVIL_WALLS_OUT = 17;
		static const int T_INTERSECTION_HANDLING_IN = 18;
		static const int T_INTERSECTION_HANDLING_OUT = 19;
		static const int EDGE_OF_WALL_IN = 20;
		static const int EDGE_OF_WALL_OUT = 21;
		static const int INIT = 22;

	private:
		enum WallFollowState {FollowWall, LookForEndOfWall, MoveTail,
			RotateRight, RotateLeft, LookForBeginningOfWall, AlignToWall, HandleEvilWalls, AlignToFrontWall, TIntersectionHandling, 
			EdgeOfWall};

		struct State
		{
			WallFollowState mState;
			bool initialized;
			float handleWallInitDistance;
			double gapStartDist;
			void set(WallFollowState state) {
				mState = state;
				initialized = false;
				handleWallInitDistance = 0.6f * TAIL_LENGTH;
			}
		};

		static const float MOVEMENT_SPEED = 0.16f;
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
        static const float LEFT_WALL_TOO_CLOSE = 0.03f;
        static const float NO_WALL_DISTANCE = 0.15f;
        static const float WALL_DIST_TOL = 0.01f;

		static const float MIN_WALL_DISTANCE = 0.03f;
		static const float MAX_WALL_DISTANCE = 0.045f;

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

		// variables for lookForBeginningOfWall (could be moved into struct)
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
    	void publishState(int state);

		// control function for wall following
		void followWall();

		// helper functions
		bool seesWall(float distance);
		bool wallInFront();
		

		bool nextToWall();

		// function to publish speeds
		void publishSpeeds(float left, float right);



		
	}; //MoveFollowWall

}; //namespace amee

#endif //MOVE_FOLLOW_WALL_H
