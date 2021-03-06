#include <iostream>
#include "amee/Velocity.h"
#include "amee/MovementEvent.h"
#include "MoveCoordinate.h"
#include "MoveRotate.h"
#include "MoveStraight.h"
#include "MovementControl.h"
#include <cmath>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace amee;
using namespace std;

MoveCoordinate::MoveCoordinate(ros::Publisher& pub, ros::Publisher& movement_event_pub) {
	mPub = pub;
	mMovementEventPub = movement_event_pub;

	mRotater = new MoveRotate(mPub);
	mStraightMove = new MoveStraight(mPub);
	mRunning = false;
}

MoveCoordinate::~MoveCoordinate() {
	delete mRotater;
	delete mStraightMove;
}

void MoveCoordinate::init(const SensorData& data) {
	mRunning = true;
	mRotationDone = false;
}

void MoveCoordinate::init(const SensorData &data, const float &x, const float &y){
	init(data);

	cout << "In MoveCoordinate init, destpos: (" 
		 << x << ", " << y << ")" << endl;

	float angle = atan2(y, x) * 180 / M_PI;
	angle = checkDirection(angle);

	float destPoint[2] = {x, y};
	float curPoint[2] = {0.0f, 0.0f};

	mDistance = euclidDist(destPoint, curPoint);
	// cout << "distance " << distance << ", angle " << angle << endl;

	cout << "Rotate: " << angle << ", and then move " << mDistance << endl;

	mRotater->init(data, angle);


}

bool MoveCoordinate::isRunning() const { return mRunning; }

void MoveCoordinate::doControl(const SensorData& data) {

	if(mRunning) {

		if (mRotater->isRunning()) {
			mRotater->doControl(data);
		}else if(!mRotationDone){//only one time
			cout << "In MoveCoordinate, done rotating, now initiating straight movement" << endl;

			mRotationDone = true;
			mStraightMove->init(data, mDistance);
		}

		if(mRotationDone){//rotation done, go straight to the point
			// if(wallInFront(data)){
			// 	//publish the movementEvent type
			// 	MovementEvent me;
			// 	me.type = MovementControl::MOVEMENT_EVENT_TYPE_OBSTICLE_IN_FRONT;
			// 	mMovementEventPub.publish(me);

			// 	mRunning = false;
			// 	std::cout << "In MoveCoordinate, wall-In-Front, stopping!" << std::endl;

			// 	//publish stop
			// 	Velocity vel;
			// 	vel.left = vel.right = 0.0f; 
			// 	mPub.publish(vel);

			// 	return;
			// }
			if(mStraightMove->isRunning()){
				mStraightMove->doControl(data);
			}else{
				cout << "In MoveCoordinate, done with the MoveCoordinate movement." << endl;
				mRunning = false;


				//publish the movementEvent type
				MovementEvent me;
				me.type = MovementControl::MOVEMENT_EVENT_TYPE_DONE_MOVING_COORDINATE;
				mMovementEventPub.publish(me);

			}
		}
	}
}

bool MoveCoordinate::wallInFront(const SensorData& data) {
   	return (data.irdistances.wheelRight <= 0.02f && data.irdistances.wheelRight >= -0.03f)
            || (data.irdistances.wheelLeft <= 0.02f && data.irdistances.wheelLeft >= -0.03f)
            || (data.sonarDistance <= 0.03f);
}

// bool MoveCoordinate::distReached() const {
// 	return (sqrt(pow(mX - mCurX, 2) + pow(mY - mCurY, 2)) < DISTANCE_THRESHHOLD);
// }

// float MoveCoordinate::getRotationAngle(const float p1[2], const float p2[2]) const {
// 	float dX = p2[0] - p1[0];
// 	float dY = p2[1] - p1[1];

// 	float v = atan2(dY, dX) * 180 / M_PI;
// 	float angleDifference = v - mCurAngle;

// 	return checkDirection(angleDifference);

// }


float MoveCoordinate::checkDirection(const float f) const {
	float tmp = fabs(f);
	if(tmp > 180.0f && tmp < 360.0f){
		if(f < 0)
			return f+360.0f;
		return f-360.0f;
	}

	float f_mod = f;
	return fmod(f_mod, 360);
}

float MoveCoordinate::euclidDist(const float p1[2], const float p2[2]) const {
	float p[2] = {p2[0]-p1[0], p2[1]-p1[1]};
	return sqrt(norm(p));
}

float MoveCoordinate::norm(const float p[2]) const {
	return (p[0]*p[0] + p[1]*p[1]);
}


