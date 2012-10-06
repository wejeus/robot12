#include "ros/ros.h"
#include <string.h>
#include "MotorControl.h"
#include <iostream>
#include <cmath>

#define WHEEL_RADIUS 0.1
#define WHEEL_BASE 0.5f
#define TICS_PER_REVOLUTION 500.0f // encoder tics/rev
#define REVOLUTION_PER_SEC_LEFT 1.2f 
#define REVOLUTION_PER_SEC_RIGHT 1.0f
#define NUM_AVERAGED_MEASUREMENTS 15
#define ROTATION_SPEED 0.4f
#define EPSILON_SPEED 0.001f;

using namespace amee;
using namespace robo;


void MotorControl::setMotorPublisher(ros::Publisher pub) {
	mot_pub = pub;
}

void MotorControl::setOdometryPublisher(ros::Publisher pub) {
	odo_pub = pub;
}

//Callback function for the "/encoder" topic. Stores the two last encoder values.
void MotorControl::receive_encoder(const Encoder::ConstPtr &msg)
{
	int right = msg->right;
	int left = msg->left;
	double timestamp = msg->timestamp;
	
	mMeasurementValidCounter = mMeasurementValidCounter > 0 ? mMeasurementValidCounter - 1 : 0;

	struct timeval start;
	gettimeofday(&start, NULL);
	//std::cout << start.tv_sec << std::endl;

	// We filter noise by calculating the average of multiple measurements (TODO replace by Kalman filter)
	// accumulate NUM_AVERAGED_MEASUREMENTS and then calculate the mean 
	if (mMeasurementCounter < NUM_AVERAGED_MEASUREMENTS) {
		mMeasurementAccumulator.left += left;
		mMeasurementAccumulator.right += right;
		mMeasurementAccumulator.timestamp += timestamp;
		++mMeasurementCounter; 	
	} else { // enough measurement accumulated, calculate mean now
		// save old encoder mean
		mPrevEncoder.timestamp = mCurrentEncoder.timestamp;
		mPrevEncoder.left = mCurrentEncoder.left;
		mPrevEncoder.right = mCurrentEncoder.right;

		// calculate new mean
		mCurrentEncoder.timestamp = mMeasurementAccumulator.timestamp / (float)(mMeasurementCounter);
		mCurrentEncoder.left = mMeasurementAccumulator.left / (float)(mMeasurementCounter);	
		mCurrentEncoder.right = mMeasurementAccumulator.right / (float)(mMeasurementCounter);

		publishOdometry();

		// reset accumulator
		mMeasurementAccumulator.timestamp = 0;
		mMeasurementAccumulator.left = 0;
		mMeasurementAccumulator.right = 0;
		mMeasurementCounter = 0;
	}
}

void MotorControl::publishOdometry() {
	// calculate odometry stuff and publish it
	float tDistLeft = (mCurrentEncoder.left - mPrevEncoder.left) / TICS_PER_REVOLUTION * (2.0f * M_PI * WHEEL_RADIUS);
	float tDistRight = (mCurrentEncoder.right - mPrevEncoder.right) / TICS_PER_REVOLUTION * (2.0f * M_PI * WHEEL_RADIUS);
	mOdometry.leftWheelDistance += tDistLeft;
	mOdometry.rightWheelDistance += tDistRight;
	mOdometry.angle += ((tDistRight - tDistLeft) / WHEEL_BASE) /  M_PI * 180.0f;
	mOdometry.distance =  (mOdometry.leftWheelDistance + mOdometry.rightWheelDistance) / 2.0f;
	
	//TODO calculate x,y position and publish

	odo_pub.publish(mOdometry);
}

// Calculates the current pwm velocities of both wheels based on the last two encoder values;
void MotorControl::calcWheelPWMVelocities(Velocity& velocity)
{
	
	// Right & left wheel velocity in ticks/s
	double deltaTime = (mCurrentEncoder.timestamp - mPrevEncoder.timestamp);
	//std::cout << "deltaTime " << deltaTime << std::endl;
	float leftTicksVelocity = ((float)mCurrentEncoder.left - (float)mPrevEncoder.left) / (float)deltaTime;
	float rightTicksVelocity = ((float)mCurrentEncoder.right - (float)mPrevEncoder.right) / (float)deltaTime;

	//std::cout << "measured ticks: prev:" << mPrevEncoder.left << " " << mPrevEncoder.right  << " " << mCurrentEncoder.left << " " << mCurrentEncoder.right << std::endl;

	//std::cout << "measured ticks velocities: " << leftTicksVelocity << " " << rightTicksVelocity << std::endl;

	velocity.left = leftTicksVelocity / (REVOLUTION_PER_SEC_LEFT * TICS_PER_REVOLUTION);
	velocity.right = rightTicksVelocity / (REVOLUTION_PER_SEC_RIGHT * TICS_PER_REVOLUTION); 

	std::cout << "measured pwm velocities: " << velocity.left << " " << velocity.right << std::endl;
}

void MotorControl::receive_speed(const amee::Velocity::ConstPtr &v) {
	setSpeed(v->left, v->right);
}

// Sets the given speed ( in m/s ) to both motors.
void MotorControl::setSpeed(float vLeft, float vRight) {
	mVelocity.left = vLeft;
	mVelocity.right = vRight;
	mMeasurementValidCounter = 2 * NUM_AVERAGED_MEASUREMENTS;
	/*mOdometry.leftWheelDistance = 0.0f;
	mOdometry.rightWheelDistance = 0.0f;
	mOdometry.distance = 0.0f;
	mOdometry.angle = 0.0f;*/
}

void MotorControl::drive()
{

	// transform given velocities to pwm velocity
	// TODO this calculation has only need to be done once (in setSpeed(..))
	float leftVPWM = mVelocity.left / (2.0f * M_PI * WHEEL_RADIUS * REVOLUTION_PER_SEC_LEFT);
	float rightVPWM = mVelocity.right / (2.0f * M_PI * WHEEL_RADIUS * REVOLUTION_PER_SEC_RIGHT);
	std::cout << "target pwm velocities " << leftVPWM << " " << rightVPWM << std::endl;
	
	if (measurementsValid()) {
		// calculate current velocity (in pwm) based on the last two encoder values 
		Velocity pwmVelocity;
		calcWheelPWMVelocities(pwmVelocity);

		// calculate errors: error < 0 <-> too fast, error > 0 <-> too slow 
		float leftError = leftVPWM - pwmVelocity.left; // error on the left side
		float rightError = rightVPWM - pwmVelocity.right; // error on the left side	
	
		mMotor.right	= mMotor.right + 0.2 * rightError;//set right motorspeed[-1,1]
		mMotor.left	= mMotor.left + 0.2 * leftError;//set left motorspeed[-1,1]

	} else { // we have no valid measurements of the current speed, so just set it to the non controlled values
		mMotor.left = leftVPWM;
		mMotor.right = rightVPWM;
	}
	
	//printf("set motor pwm velocities: %f %f \n", mMotor.left, mMotor.right);
	mot_pub.publish(mMotor);
}

void MotorControl::init() {
	mMeasurementValidCounter = 2 * NUM_AVERAGED_MEASUREMENTS;
	mMeasurementCounter = 0;

	mPrevEncoder.left = 0;
	mPrevEncoder.right = 0;
	mPrevEncoder.timestamp = 0;

	mCurrentEncoder.timestamp = 0;
	mCurrentEncoder.left = 0;
	mCurrentEncoder.right = 0;

	mMeasurementAccumulator.timestamp = 0;
	mMeasurementAccumulator.left = 0;
	mMeasurementAccumulator.right = 0;
	
	mOdometry.leftWheelDistance = 0.0f;
	mOdometry.rightWheelDistance = 0.0f;
	mOdometry.distance = 0.0f;
	mOdometry.angle = 0.0f;
	
	mMotor.left = 0;
	mMotor.right = 0;
}

// Returns true if the current measurements are valid. This is the case if the measurements have been made after
// the current speed was set.
bool MotorControl::measurementsValid() {
	return mMeasurementValidCounter == 0;
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "MotorControl");//Creates a node named "MotorControl"
	ros::NodeHandle n;

	// create the controller and initialize it
	MotorControl control;
	control.init();

	ros::Subscriber	enc_sub;
	ros::Publisher	int_pub;

	// create subscriber for velocity commands
	ros::Subscriber velocities_sub;
	velocities_sub = n.subscribe("/amee/motor_control/set_wheel_velocities", 1000, &MotorControl::receive_speed, &control);

	// create subscriber for encoder values
	enc_sub = n.subscribe("/serial/encoder", 1000, &MotorControl::receive_encoder, &control);

	// create publisher for low level motor speeds (pwm)
	ros::Publisher mot_pub = n.advertise<Motor>("/serial/motor_speed", 100000);
	control.setMotorPublisher(mot_pub);
	
	// create publisher for odometry values
	ros::Publisher odo_pub = n.advertise<Odometry>("/amee/motor_control/odometry", 10000);
	control.setOdometryPublisher(odo_pub);

	//used to publish a topic that changes the intervall between the "/encoder" topics published.
	int_pub = n.advertise<std_msgs::Int32>("/serial/encoder_interval", 100000);

	// set our control loop at 6Hz (TODO increase frequency when using Kalman filter)
	ros::Rate loop_rate(6);
	
	// make sure the robot isn't moving on startup
	control.setSpeed(0.0f, 0.0f);
	
	while(ros::ok()){

		std_msgs::Int32 interval;
		interval.data = 10; // we want the encoder values as fast as possible
		// publish the encoder interval
		int_pub.publish(interval);

		// go to sleep for a short while
		loop_rate.sleep();

		// call all callbacks
		ros::spinOnce();
		
		// drive!
		control.drive();
	}

	return 0;
}
