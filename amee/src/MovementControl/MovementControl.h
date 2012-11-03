#ifndef MOVEMENT_CONTROL_H
#define MOVEMENT_CONTROL_H

#include "amee/IRDistances.h"
#include "amee/Odometry.h"
#include "amee/MovementCommand.h"
#include "MovementState.h"

namespace amee {
class MoveRotate;
class MoveStraight;
class MoveStop;
class MoveFollowWall;

class MovementControl {

	public:
		MovementControl(ros::Publisher pub);
		~MovementControl();
		void receive_distances(const amee::IRDistances::ConstPtr &msg);
		void receive_odometry(const amee::Odometry::ConstPtr &msg);
		void receive_command(const amee::MovementCommand::ConstPtr &msg);
		void setSpeedPublisher(ros::Publisher& pub);
		void doControl();
		void init();

		static const int TYPE_MOVE_STRAIGHT = 1;
		static const int TYPE_MOVE_ROTATE = 2;
		static const int TYPE_MOVE_COORDINATE = 3;
		static const int TYPE_FOLLOW_WALL = 4;
		static const int TYPE_STOP = 5;

	private:



		amee::SensorData mSensorData;

		ros::Publisher speed_pub;

		amee::MovementState* mCurrentState;

		amee::MoveRotate* mRotationState;
		amee::MoveStraight* mStraightState;
		amee::MoveStop* mStopState;
		amee::MoveFollowWall* mFollowWallState;
		//TODO add other states here

};
}
#endif