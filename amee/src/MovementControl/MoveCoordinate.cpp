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
	// TODO: set initial values
	mX = data.odometry.x;
	mY = data.odometry.y;
	mAngle = data.odometry.angle;
	mRunning = true;
	mFirstRun = true;
	mRotationDone = false;
}

bool MoveCoordinate::isRunning() const { return mRunning; }

void MoveCoordinate::doControl(const SensorData& data) {

	if(mRunning) {
		mCurX = data.odometry.x;
		mCurY = data.odometry.y;

		//TODO: remove this
		mSensorData = data;

		if(mFirstRun){
			cout << "In MoveCoordinate mFirstRun, destpos: (" 
				 << mX << ", " << mY << ")" << " curPos: ("
				 << mCurX << ", " << mCurY << ")" << endl;

			float destPoint[2] = {mX, mY};
			float curPoint[2] = {mCurX, mCurY};

			// float distance = norm(curPoint);
			// float angle = acos(inner_product(destPoint, destPoint+3, curPoint, 0.0f) / (norm(destPoint)*norm(curPoint)));
			// cout << "distance " << distance << ", angle " << angle << endl;

			mAngle = getRotationAngle(curPoint, destPoint);
			mDistance = euclidDist(curPoint, destPoint);
			cout << "Rotate: " << mAngle << ", and then move " << mDistance << endl;

			mRotater->init(mSensorData, mAngle);

			// Velocity v; //TODO: set the volocity data values
			// mPub.publish(v);
			mFirstRun = false;
		}

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

float MoveCoordinate::getRotationAngle(const float p1[2], const float p2[2]) const {
	float dX = p2[0] - p1[0];
	float dY = p2[1] - p1[1];

	float v = atan(dY/dX) * 180 / M_PI;
	float angleDifference = v - mCurAngle;

	return checkDirection(angleDifference);

}


float MoveCoordinate::checkDirection(const float f) const {
	float tmp = fabs(f);
	if(tmp > 180.0f && tmp < 360.0f){
		if(f < 0)
			return f+360.0f;
		return f-360.0f;
	}

	return f;
}

float MoveCoordinate::euclidDist(const float p1[2], const float p2[2]) const {
	float p[2] = {p2[0]-p1[0], p2[1]-p1[1]};
	return sqrt(norm(p));
}

float MoveCoordinate::norm(const float p[2]) const {
	return (p[0]*p[0] + p[1]*p[1]);
}


