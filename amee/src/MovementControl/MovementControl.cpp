#include "ros/ros.h"
#include "MovementControl.h"
#include "amee/Velocity.h"
#include <iostream>
#include <cmath>
#include "MoveRotate.h"
#include "MoveStraight.h"
#include "MoveStop.h"
#include "MoveFollowWall.h"
#include "MoveAlignWall.h"
#include "MoveAlignToFrontWall.h"
#include "amee/FollowWallStates.h"
#include <std_msgs/Int32.h>

using namespace amee;


MovementControl::MovementControl(ros::Publisher& pub, ros::Publisher& statesPub) {
	mRotationState = new MoveRotate(pub);
	mStraightState = new MoveStraight(pub);
	mStopState = new MoveStop(pub);
	mFollowWallState = new MoveFollowWall(pub, statesPub);
	mAlignWallState = new MoveAlignWall(pub);
	mAlignToFrontWallState = new MoveAlignToFrontWall(pub);
	mCurrentState = mStopState;
}

MovementControl::~MovementControl() {
	delete mRotationState;
	delete mStraightState;
	delete mStopState;
	delete mFollowWallState;
	delete mAlignWallState;
	delete mAlignToFrontWallState;
}

void MovementControl::setSpeedPublisher(ros::Publisher& pub) {
	speed_pub = pub;
}

// Callback for IR distances
void MovementControl::receive_distances(const IRDistances::ConstPtr &msg)
{
	mSensorData.irdistances.rightFront = msg->rightFront;
	mSensorData.irdistances.rightBack = msg->rightBack;
	// mSensorData.irdistances.frontShortRange = msg->frontShortRange;
	mSensorData.irdistances.obstacleInFront = msg->obstacleInFront;
	mSensorData.irdistances.wheelRight = msg->wheelRight;
	mSensorData.irdistances.leftBack = msg->leftBack;
	mSensorData.irdistances.leftFront = msg->leftFront;
	mSensorData.irdistances.wheelLeft = msg->wheelLeft;
}

void MovementControl::receive_odometry(const Odometry::ConstPtr &msg) {
	mSensorData.odometry.angle = msg->angle;
	mSensorData.odometry.distance = msg->distance;
	mSensorData.odometry.x = msg->x;
	mSensorData.odometry.y = msg->y;
	// TODO others
}

void MovementControl::doControl() {
	if (mCurrentState->isRunning()) {
		mCurrentState->doControl(mSensorData);
	}

    std::cout << "Front wall detected: " << wallInFront(mSensorData) << std::endl;
}

void MovementControl::receive_sonar(const roboard_drivers::sonar::ConstPtr &msg) {
	mSensorData.sonarDistance = msg->distance / 100.0f;
	// std::cout << "Sonar received: " << mSensorData.sonarDistance << std::endl;
}

void MovementControl::receive_command(const amee::MovementCommand::ConstPtr &msg) {
	int type = msg->type;
	float angle = msg->angle;
	float distance = msg->distance;
	switch(type) {
		case TYPE_MOVE_STRAIGHT:
			// std::cout << "MOVE STRAIGHT COMMAND RECEIVED" << std::endl;
			mCurrentState = mStraightState;
			mStraightState->init(mSensorData, distance);
		break;
		case TYPE_MOVE_ROTATE:		
			// std::cout << "ROTATE COMMAND RECEIVED" << std::endl;
			mCurrentState = mRotationState;
			mRotationState->init(mSensorData, angle);
		break;
		case TYPE_MOVE_COORDINATE:
		 	// std::cout << "MOVE TO COORDINATE" << std::endl;
		 	//TODO initialize MoveCoonrdinate here
		 	// and change state
		break;
		case TYPE_FOLLOW_WALL:
		 	// std::cout << "FOLLOW WALL" << std::endl;
		 	mFollowWallState->init(mSensorData);
		 	mCurrentState = mFollowWallState;
		break;
		case TYPE_STOP:
		 	// std::cout << "STOP COMMAND RECEIVED" << std::endl;
		 	mCurrentState = mStopState;
		break;
		case TYPE_ALIGN_TO_WALL:
			// std::cout << "ALIGN TO WALL COMMAND RECEIVED" << std::endl;
			mAlignWallState->init(mSensorData);
			mCurrentState = mAlignWallState;
		break;
		case TYPE_ALIGN_TO_FRONT_WALL:
			mAlignToFrontWallState->init(mSensorData);
			mCurrentState = mAlignToFrontWallState;
		break;
		// default: std::cout << "GAY COMMAND RECEIVED" << std::endl;
	}
}

void MovementControl::init() {
	mRotationState->init(mSensorData);
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "MovementControlNode");//Creates a node named "MotorControl"
	ros::NodeHandle n;

	ros::Publisher vel_pub = n.advertise<Velocity>("/amee/motor_control/set_wheel_velocities", 100);
	ros::Publisher wall_pub = n.advertise<FollowWallStates>("/amee/follow_wall_states", 100);
	ros::Publisher sonar_interval_pub = n.advertise<std_msgs::Int32>("roboard/sonar_interval", 100);

	// create the controller and initialize it
	MovementControl control(vel_pub, wall_pub);
	

	ros::Subscriber dist_sub;

	// create subscriber for distances
	dist_sub = n.subscribe("/amee/sensors/irdistances", 100, &MovementControl::receive_distances, &control);
	ros::Subscriber odo_sub = n.subscribe("/amee/motor_control/odometry", 100, &MovementControl::receive_odometry, &control);
	ros::Subscriber sonar_sub = n.subscribe("/roboard/sonar",1, &MovementControl::receive_sonar, &control);

	ros::Subscriber command_sub = n.subscribe("/MovementControl/MovementCommand",10,&MovementControl::receive_command, &control);

	ros::Rate loop_rate(100);
	while((vel_pub.getNumSubscribers() == 0 || sonar_interval_pub.getNumSubscribers() == 0) && ros::ok()) {
		loop_rate.sleep();
	} 

	std_msgs::Int32 interval;
	interval.data = 50;
	sonar_interval_pub.publish(interval);

	control.init();
	
	while(ros::ok()){
		
		// go to sleep for a short while
		loop_rate.sleep();

		// call all callbacks
		ros::spinOnce();
		
		// drive!
		control.doControl();
	}

	return 0;
}
