#include "MoveStraight.h"
#include "amee/Velocity.h"

namespace amee{


MoveStraight::MoveStraight(ros::Publisher & pub){
	mPub = pub;
	mFirstRun = false;
	mRunning = false;
}

//destructor
MoveStraight::~MoveStraight(){}

/**
 * Sets the velocity of the wheels, if no value is given the default value will be set
 **/
void MoveStraight::init(const float vel) {
	mVelocity = vel;
	mRunning = true;
	mFirstRun = true;
	mGoingDistance = false;
}

void MoveStraight::init(const SensorData &data){
	init();
}

void MoveStraight::init(const SensorData &data, float distance) {
	mTargetDistance = distance;
	mGoingDistance = true;	
	mStartingDistance = data.odometry.distance;
	mVelocity = 0.15f;
	mRunning = true;
	mFirstRun = true;
}

bool MoveStraight::isRunning() const {return mRunning;}

/**
 * Publishes a 
 **/
void MoveStraight::doControl(const SensorData &data){
	// std::cout << mStartingDistance << std::endl << mTargetDistance << std::endl << data.odometry.distance << std::endl;

	if (mGoingDistance) {
		float travelledDistance = data.odometry.distance - mStartingDistance;
		// std::cout << travelledDistance << std::endl;
		if (fabs(mTargetDistance - travelledDistance) <= 0.01f) {
			// std::cout << "MoveStraight: Distance reached!!!" << std::endl;
			mRunning = false;
			Velocity v; v.right = 0.0f; v.left = 0.0f;
			mPub.publish(v);
			return;
		}
	}

	if (mFirstRun) {
		Velocity v; v.right = mVelocity; v.left = mVelocity;
		mPub.publish(v);
		mFirstRun = false;
	}

}


}; // namespace amee
