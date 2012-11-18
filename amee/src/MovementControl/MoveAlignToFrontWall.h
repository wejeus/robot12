#ifndef MOVE_ALIGN_FRONT_WALL_H
#define MOVE_ALIGN_FRONT_WALL_H

#include "MovementState.h"
#include "ros/ros.h"

namespace amee{
	class MoveAlignWall;
	class MoveStraight;

	class MoveAlignToFrontWall : public MovementState {
	public:
		MoveAlignToFrontWall(ros::Publisher& pub);
		~MoveAlignToFrontWall();

		virtual void init(const SensorData& data);
		virtual void init(const SensorData& data, float refDist);
		virtual bool isRunning() const;
		virtual void doControl(const SensorData& data);
	
	private:
		ros::Publisher mSpeedPub;
		MoveAlignWall* mWallAligner;
		MoveStraight* mStraightMover;
		
		bool mIsRunning;
		bool mRefDistReached;
		bool mCanAlign;
		float mRefDist;
		
		void publishSpeeds(float left, float right);
		bool frontAlignmentPossible(const SensorData& data);

	}; 

}; //namespace amee

#endif //MOVE_ALIGN_FRONT_WALL_H
