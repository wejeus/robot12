#ifndef MOVEMENT_CONTROL_H
#define MOVEMENT_CONTROL_H

#include "amee/IRDistances.h"
#include "amee/Odometry.h"
#include "amee/MovementCommand.h"
#include "roboard_drivers/sonar.h"
#include "MovementState.h"

namespace amee {
class MoveRotate;
class MoveStraight;
class MoveStop;
class MoveFollowWall;
class MoveAlignWall;
class MoveAlignToFrontWall;

class MovementControl {

	public:
		MovementControl(ros::Publisher& pub, ros::Publisher& statespub);
		~MovementControl();
		void receive_distances(const amee::IRDistances::ConstPtr &msg);
		void receive_odometry(const amee::Odometry::ConstPtr &msg);
		void receive_command(const amee::MovementCommand::ConstPtr &msg);
		void receive_sonar(const roboard_drivers::sonar::ConstPtr &msg);
		void setSpeedPublisher(ros::Publisher& pub);
		void doControl();
		void init();

		static bool wallInFront(const SensorData& data) {
		   	return data.irdistances.obstacleInFront 
	            || (data.irdistances.wheelRight <= 0.05f && data.irdistances.wheelRight >= -0.03f)
	            || (data.irdistances.wheelLeft <= 0.05f && data.irdistances.wheelLeft >= -0.03f)
	            || (data.sonarDistance <= 0.13f);
		}

		static const int TYPE_MOVE_STRAIGHT = 1;
		static const int TYPE_MOVE_ROTATE = 2;
		static const int TYPE_MOVE_COORDINATE = 3;
		static const int TYPE_FOLLOW_WALL = 4;
		static const int TYPE_STOP = 5;
		static const int TYPE_ALIGN_TO_WALL = 6;
		static const int TYPE_ALIGN_TO_FRONT_WALL = 7;

	private:

		amee::SensorData mSensorData;

		ros::Publisher speed_pub;

		amee::MovementState* mCurrentState;

		amee::MoveRotate* mRotationState;
		amee::MoveStraight* mStraightState;
		amee::MoveStop* mStopState;
		amee::MoveFollowWall* mFollowWallState;
		amee::MoveAlignWall* mAlignWallState;
		amee::MoveAlignToFrontWall* mAlignToFrontWallState;
		//TODO add other states here

};
}
#endif