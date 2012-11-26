#include <iostream>
#include "amee/Velocity.h"
#include "MoveCoordinate.h"
#include "MoveRotate.h"
#include "MoveStraight.h"
#include <cmath>
#include <numeric>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace amee;
using namespace std;

MoveCoordinate::MoveCoordinate(ros::Publisher& pub) {
	mPub = pub;
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

	mRotater->init(mSensorData, angle);


}

bool MoveCoordinate::isRunning() const { return mRunning; }

void MoveCoordinate::doControl(const SensorData& data) {

	if(mRunning) {

		//TODO: remove this
		mSensorData = data;

		if (mRotater->isRunning()) {
			mRotater->doControl(mSensorData);
		}else if(!mRotationDone){//only one time
			cout << "In MoveCoordinate, done rotating, now initiating straight movement" << endl;

			mRotationDone = true;
			mStraightMove->init(mSensorData, mDistance);
		}

		if(mRotationDone){//rotation done, go straight to the point
			if(mStraightMove->isRunning()){
				mStraightMove->doControl(mSensorData);
			}else{
				cout << "In MoveCoordinate, done with the MoveCoordinate movement." << endl;
				mRunning = false;
			}
		}
	}
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


