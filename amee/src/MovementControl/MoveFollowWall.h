#ifndef MOVE_FOLLOW_WALL_H
#define MOVE_FOLLOW_WALL_H

#include "MovementState.h"
#include "MoveRotate.h"
#include "ros/ros.h"

namespace amee{
	class MoveFollowWall : public MovementState {
	public:
		MoveFollowWall(ros::Publisher& pub);
		~MoveFollowWall();

		virtual void init(const SensorData& data);
		virtual bool isRunning() const;
		virtual void doControl(const SensorData& data);
		
	private:
		enum WallFollowState {foundFrontWall, followSideWall, foundEndOfWall};

		static const float MOVEMENT_SPEED = 0.3;
		static const float MAX_ROTATION_SPEED = 0.1;
		static const float MIN_ROTATION_SPEED = 0.06;
		static const float IR_BASE_RIGHT = 0.104;

		float linearSpeed;
		float K_p_keepRef;
		float K_p_reachRef;
		float K_i_keepRef;
		float K_i_reachRef;
		float maxErrorSum;
		float refDistance;
		float noWallDistance;
		float wallDistTol;
		float error_sum;
		
		WallFollowState mState;		
		SensorData mSensorData;

		// allows us to rotate by calling its doControl after we initialized it as long as we want to rotate
		amee::MoveRotate *mRotater;

		ros::Publisher mVelPub;

		bool mRunning;
		void followWall();
		void rotate();
		void publishSpeeds(float left, float right);



		
	}; //MoveFollowWall

}; //namespace amee

#endif //MOVE_FOLLOW_WALL_H
