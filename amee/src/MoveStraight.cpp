#include "MoveStraight.h"

namespace amee{


//default constructor
MoveStraight::MoveStraight(){}

MoveStraight::MoveStraight(ros::Publisher & pub){
	mPub = pub;
}

//destructor
MoveStraight::~MoveStraight(){}

/**
 * Sets the velocity of the wheels, if no value is given the default value will be set
 **/
void MoveStraight::init(const float vel = DEFAULT_VELOCITY):mVelocity(vel), mRunning(false){}

const bool& MoveStraight::isRunning() const {return mRunning;}
const float& MoveStraight::getCurrentVelocity()const{return mVelocity;}

/**
 * Publishes a 
 **/
void MoveStraight::doControl(SensorData &data){
	mRunning = true;
	Velocity v; v.right = mVelocity; v.left = mVelocity;
	mPub.publish(v);
	mRunning = false;	
}


}; // namespace amee
