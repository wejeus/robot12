#include "MoveStraight.h"
#include "amee/Velocity.h"

namespace amee{


MoveStraight::MoveStraight(ros::Publisher & pub){
	mPub = pub;
}

//destructor
MoveStraight::~MoveStraight(){}

/**
 * Sets the velocity of the wheels, if no value is given the default value will be set
 **/
void MoveStraight::init(const float vel) {
	mVelocity = vel;
	mRunning = true;
}

void MoveStraight::init(const SensorData &data){
	init();
}

bool MoveStraight::isRunning() const {return mRunning;}

/**
 * Publishes a 
 **/
void MoveStraight::doControl(const SensorData &data){
	Velocity v; v.right = mVelocity; v.left = mVelocity;
	mPub.publish(v);
}


}; // namespace amee
