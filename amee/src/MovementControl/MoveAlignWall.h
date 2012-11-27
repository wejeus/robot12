#ifndef MOVE_ALIGN_WALL_H
#define MOVE_ALIGN_WALL_H

#include "MovementState.h"
#include "ros/ros.h"

namespace amee{
		

	static const int ALIGN_RIGHT_WALL = 0;	
	static const int ALIGN_LEFT_WALL = 1;
	static const int ALIGN_FRONT_WALL = 2;

	class MoveAlignWall : public MovementState {
	public:
		MoveAlignWall(ros::Publisher& pub);
		MoveAlignWall(ros::Publisher& pub, ros::Publisher& eventPub);
		~MoveAlignWall();

		/**
			Initializes this alignment movement with default values:
				align to wall on the right
		**/
		virtual void init(const SensorData& data);
		void init(const SensorData& data, const int& side);
		virtual bool isRunning() const;
		virtual void doControl(const SensorData& data);
	private:
		ros::Publisher mSpeedPub;
		ros::Publisher mEventPub;
		int mWallSide;
		bool mIsRunning;
		bool mPublishEvents;
		float mStartingAngle;

		void stop();
		void publishSpeeds(float left, float right);

	}; //MoveRotate

}; //namespace amee

#endif //MOVE_ALIGN_WALL_H
