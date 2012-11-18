#include <iostream>
#include "amee/Velocity.h"
#include "MoveCoordinate.h"
#include "MoveRotate.h"
#include "MoveStraight.h"
#include <cmath>
#include <complex>

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

		if(mFirstRun){

			complex<float> ref_vec(1.0f, 0.0f);
			complex<float> point(mCurX, mCurY);

			float distance = norm(point);
			float angle = acos(inner_prod(ref_vec, point,0) / (norm(ref_vec)*norm(point)));
			cout << "distance " << distance << ", angle " << angle << endl;

			mRotater->init(data);

			// Velocity v; //TODO: set the volocity data values
			// mPub.publish(v);
			mFirstRun = false;
		}

		if (mRotater->isRunning()) {
			mRotater->doControl(mSensorData);
		}else{
			mRotationDone = true;
			mStraightMove->init(mSensorData);
		}

		if(mRotationDone){//rotation done, go straight to the point
			if(mStraightMove->isRunning()){
				mStraightMove->doControl(mSensorData);
			}else{
				mRunning = false;
			}
		}
	}

/* from python code
	ref_vec = (1.0, 0.0) # Reference vector, set to coordinate axis in direction of robot
    point = (x, y)
    distance = norm(point)
    angle = math.acos(dot_product(ref_vec, point) / (norm(ref_vec)*norm(point)))
    self.move_rotate(angle * (180/math.pi))
    self.move_straight(distance)
*/
}

bool distReached() {
	return (sqrt(pow(mX - mCurX, 2) + pow(mY - mCurY, 2)) < DISTANCE_THRESHHOLD);
}
