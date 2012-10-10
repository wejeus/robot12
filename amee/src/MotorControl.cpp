#include "ros/ros.h"
<<<<<<< HEAD
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
=======
#include <std_msgs/Int32.h>
#include "robo/Encoder.h"
#include "robo/Motor.h"
#include "amee/Velocity.h"
#include <string.h>

#define WHEEL_RADIUS 0.1
#define TICS_PER_REVOLUTION 500 // encoder tics/rev

using namespace amee;
using namespace robo;
ros::Subscriber	enc_sub;
ros::Publisher	mot_pub;
ros::Publisher	int_pub;

void calcWheelVelosities(Encoder, Encoder, Velocity);
float linearVelocityControl(float, Encoder, Encoder, Velocity);
float angularVelocityControl(float, Encoder, Encoder, Velocity);


//Callback function for the "/encoder" topic. Prints the enconder value and randomly changes the motorspeed.
void recive_encoder(const Encoder::ConstPtr &msg)
>>>>>>> f79533ded9a80dc296c9a351dc8b6bbd17d15f87
{
	int right = msg->right;
	int left = msg->left;
	double timestamp = msg->timestamp;
<<<<<<< HEAD
	
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
=======
	printf("%f:got encoder L:%i , false R:%i\n",timestamp,left,right);

	Motor motor;
	motor.right	= (0.5f-float(rand()%1000)/1000.0f)*2.0f;//random motorspeed[-1,1]
	motor.left	= (0.5f-float(rand()%1000)/1000.0f)*2.0f;//random motorspeed[-1,1]
	mot_pub.publish(motor);
}

amee::Velocity velocity;
robo::Encoder lastEncoder;
void calcWheelVelosities(Encoder encoderMsg, Encoder lastEncoder, Velocity velocity)
{

	// Right & left wheel velocity in m/s
	float leftVelocity = (encoderMsg.left - lastEncoder.left) / TICS_PER_REVOLUTION * 2 * 3.14 * WHEEL_RADIUS / (encoderMsg.timestamp - lastEncoder.timestamp) * 1000;
	float rightVelocity = (encoderMsg.right - lastEncoder.right) / TICS_PER_REVOLUTION * 2 * 3.14 * WHEEL_RADIUS / (encoderMsg.timestamp - lastEncoder.timestamp) * 1000;
	
	velocity.linear = (leftVelocity + rightVelocity) / 2;
	velocity.angular = leftVelocity - rightVelocity;

	// save encoder value to next iteration
	lastEncoder.timestamp = encoderMsg.timestamp;
	lastEncoder.left      = encoderMsg.left;
	lastEncoder.right     = encoderMsg.right; 
}

// overwrites velocity with the controlled velocity
float linearVelocityControl(float linearVelocity, Encoder encoderMsg, Encoder lastEncoder, Velocity velocity)
{
	// make sure the actual velocity is "velocity"
	//calcWheelVelocities(&encoderMsg, &lastEncoder, &velocity);
	float linearVelocityError = velocity.linear - linearVelocity; // error id how much were going too fast
	
	return linearVelocity = linearVelocity - linearVelocityError;	
}

// overwrites angularVelocity with the controlled angularVelocity
float angularVelocityControl(float angularVelocity, Encoder encoderMsg, Encoder lastEncoder, Velocity velocity)
{
	// make sure robot is turning with "angularVelocity"
	//calcWheelVelocities(&encoderMsg, &lastEncoder, &velocity);
	float angularVelocityError = velocity.angular - angularVelocity;

	return angularVelocity = angularVelocity - angularVelocityError; 
>>>>>>> f79533ded9a80dc296c9a351dc8b6bbd17d15f87
}

int main(int argc, char **argv)
{
<<<<<<< HEAD
	
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

=======
	ros::init(argc, argv, "FakeMotorsTest");//Creates a node named "FakeMotorsTest"
	ros::NodeHandle n;
	enc_sub = n.subscribe("/serial/encoder", 1000, recive_encoder);//when "/encoder" topic is revieved call recive_encoder function
	mot_pub = n.advertise<Motor>("/serial/motor_speed", 100000);//used to publish a topic that changes the motorspeed
	int_pub = n.advertise<std_msgs::Int32>("/serial/encoder_interval", 100000);//used to publish a topic that changes the intervall between the "/encoder" topics published.

	ros::Rate loop_rate(100);
	//The loop randomly changes the intervall with wich the "/encoder" topic is published.
	
	struct timeval start, end;
	while(ros::ok()){
		gettimeofday(&start, NULL);
		std_msgs::Int32 interval;
		interval.data = 10;//+rand()%991;//very random intervall [10,1000] ms
		int_pub.publish(interval);
		loop_rate.sleep();
		ros::spinOnce();

		gettimeofday(&end, NULL);

		while((end.tv_sec+(double(end.tv_usec)/1000000.0)-(start.tv_sec+(double(start.tv_usec)/1000000.0))) < 5.0){//Sleep 50 seconds
			loop_rate.sleep();
			ros::spinOnce();
			gettimeofday(&end, NULL);
		}

		Encoder encoderMsg;
		float refVelocity = 0.5f;
		Velocity controlVelocity;
		controlVelocity.angular = angularVelocityControl(refVelocity, encoderMsg, lastEncoder, velocity);
		controlVelocity.linear = linearVelocityControl(refVelocity, encoderMsg, lastEncoder, velocity);

		printf("controlled velocity is: %f", refVelocity);
	}
>>>>>>> f79533ded9a80dc296c9a351dc8b6bbd17d15f87
	return 0;
}
