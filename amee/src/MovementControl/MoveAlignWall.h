#ifndef MOVE_ALIGN_WALL_H
#define MOVE_ALIGN_WALL_H

#include "MovementState.h"
#include "ros/ros.h"

namespace amee{
	class MoveAlignWall : public MovementState {
	public:
		MoveAlignWall(ros::Publisher& pub);
		~MoveAlignWall();

		/**
			Initializes this alignment movement with default values:
				align to wall on the right
		**/
		virtual void init(const SensorData& data);
		void init(const SensorData& data, const int& side);
		virtual bool isRunning() const;
		virtual void doControl(const SensorData& data);
	
		const static int RIGHT_WALL = 0;	
		const static int LEFT_WALL = 1;
	private:
		ros::Publisher mSpeedPub;
		int mWallSide;
		bool mIsRunning;
		float mStartingAngle;

		void stop();
		void publishSpeeds(float left, float right);

	}; //MoveRotate

}; //namespace amee

#endif //MOVE_ALIGN_WALL_H