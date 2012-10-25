#ifndef MOVEMENT_CONTROL_H
#define MOVEMENT_CONTROL_H

#include "amee/IRDistances.h"

class MovementControl {

private:

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

	amee::IRDistances mDistances;

	ros::Publisher speed_pub;

	public:
		MovementControl(ros::Publisher pub);
		void receive_distances(const amee::IRDistances::ConstPtr &msg);
		void setSpeedPublisher(ros::Publisher pub);
		void doControl();
	private:
		void followWall();
		void rotate();
		void publishSpeeds(float left, float right);
};
#endif