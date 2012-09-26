#include "ros/ros.h"
#include <string.h>
#include "MotorControl.h"
#include <iostream>
#include <cmath>

#define WHEEL_RADIUS 0.1
#define TICS_PER_REVOLUTION 500.0f // encoder tics/rev
#define REVOLUTION_PER_SEC_LEFT 1.0f //TODO check
#define REVOLUTION_PER_SEC_RIGHT 1.2f
#define NUM_AVERAGED_MEASUREMENTS 10
#define ROTATION_SPEED 0.4f

using namespace amee;
using namespace robo;
ros::Subscriber	enc_sub;
ros::Publisher	int_pub;


void MotorControl::setMotorPublisher(ros::Publisher pub) {
	mot_pub = pub;
}

//Callback function for the "/encoder" topic. Stores the two last encoder values.
void MotorControl::receive_encoder(const Encoder::ConstPtr &msg)
{
	int right = msg->right;
	int left = msg->left;
	double timestamp = msg->timestamp;
	//printf("%f:got encoder L:%i , false R:%i\n",timestamp,left,right);
	
	mInit = mInit > 0 ? mInit - 1 : 0;

	struct timeval start;
	gettimeofday(&start, NULL);
	//std::cout << start.tv_sec << std::endl;


	if (mMeasurementCounter < NUM_AVERAGED_MEASUREMENTS) {
		mMeasurementAccumulator.left += left;
		mMeasurementAccumulator.right += right;
		mMeasurementAccumulator.timestamp += timestamp;
		++mMeasurementCounter; 	
	} else {
		mPrevEncoder.timestamp = mCurrentEncoder.timestamp;
		mPrevEncoder.left = mCurrentEncoder.left;
		mPrevEncoder.right = mCurrentEncoder.right;

		mCurrentEncoder.timestamp = mMeasurementAccumulator.timestamp / (float)(mMeasurementCounter);
		mCurrentEncoder.left = mMeasurementAccumulator.left / (float)(mMeasurementCounter);	
		mCurrentEncoder.right = mMeasurementAccumulator.right / (float)(mMeasurementCounter);

		mMeasurementAccumulator.timestamp = 0;
		mMeasurementAccumulator.left = 0;
		mMeasurementAccumulator.right = 0;
		mMeasurementCounter = 0;
	}
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

void MotorControl::setSpeed(float vLeft, float vRight) {
	mVelocity.left = vLeft;
	mVelocity.right = vRight;
	//mMotor.right = vRight;
	//mMotor.left = vLeft;
}

// Sets the given speed ( in m/s ) to both motors.
void MotorControl::drive()
{

	// calculate current velocity (in pwm) based on the last two encoder values 
	Velocity pwmVelocity;
	calcWheelPWMVelocities(pwmVelocity);

	// transform given velocities to pwm velocity
	float leftVPWM = mVelocity.left / (2.0f * M_PI * WHEEL_RADIUS * REVOLUTION_PER_SEC_LEFT);
	float rightVPWM = mVelocity.right / (2.0f * M_PI * WHEEL_RADIUS * REVOLUTION_PER_SEC_RIGHT);
	
	std::cout << "target pwm velocities " << leftVPWM << " " << rightVPWM << std::endl;	

	// calculate errors: error < 0 <-> too fast, error > 0 <-> too slow 
	float leftError = leftVPWM - pwmVelocity.left; // error on the left side
	float rightError = rightVPWM - pwmVelocity.right; // error on the left side
	
	mMotor.right	= mMotor.right + 0.1 * rightError;//set right motorspeed[-1,1]
	mMotor.left	= mMotor.left + 0.1 * leftError;//set left motorspeed[-1,1]
	//printf("set motor pwm velocities: %f %f \n", mMotor.left, mMotor.right);
	mot_pub.publish(mMotor);
}

void MotorControl::init() {
	mInit = 2 * NUM_AVERAGED_MEASUREMENTS;
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

	mMotor.left = 0;
	mMotor.right = 0;
}

// overwrites angularVelocity with the controlled angularVelocity
/*float angularVelocityControl(float angularVelocity, Encoder encoderMsg, Encoder lastEncoder, Velocity velocity)
{
	// make sure robot is turning with "angularVelocity"
	//calcWheelVelocities(&encoderMsg, &lastEncoder, &velocity);
	float angularVelocityError = velocity.angular - angularVelocity;

	return angularVelocity = angularVelocity - angularVelocityError; 
}*/

void MotorControl::rotate(float degree) {
	float distToTravel = std::abs(degree / 180.0f * M_PI * 0.25f);
	int sign = degree > 0 ? 1 : -1;
	float travelledDistance = 0.0f;
	ros::Rate loop_rate(5);
	while (travelledDistance < distToTravel) {
		//driveSpeed(sign * ROTATION_SPEED, sign * (- ROTATION_SPEED));
		float distLeft = (mCurrentEncoder.left - mPrevEncoder.left) / TICS_PER_REVOLUTION * (2.0f * M_PI * WHEEL_RADIUS);
		float distRight = (mCurrentEncoder.right - mPrevEncoder.right) / TICS_PER_REVOLUTION * (2.0f * M_PI * WHEEL_RADIUS);
		travelledDistance += (std::abs(distLeft) + std::abs(distRight))/2.0f;
		//std::cout << travelledDistance << std::endl;
		loop_rate.sleep();
		ros::spinOnce();
	}
	stop();	
}

void MotorControl::stop() {
	mMotor.left = 0;
	mMotor.right = 0;
	mot_pub.publish(mMotor);
}

bool MotorControl::isInitialized() {
	return mInit == 0;
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "MotorControl");//Creates a node named "MotorControl"
	ros::NodeHandle n;
	MotorControl control;
	control.init();
	enc_sub = n.subscribe("/serial/encoder", 1000, &MotorControl::receive_encoder, &control);//when "/encoder" topic is revieved call recive_encoder function
	ros::Publisher mot_pub = n.advertise<Motor>("/serial/motor_speed", 100000);//used to publish a topic that changes the motorspeed
	control.setMotorPublisher(mot_pub);
	int_pub = n.advertise<std_msgs::Int32>("/serial/encoder_interval", 100000);//used to publish a topic that changes the intervall between the "/encoder" topics published.


	ros::Rate loop_rate(10);
	ros::Rate rotateWaiter(1);
	//The loop randomly changes the intervall with wich the "/encoder" topic is published.
	
	//struct timeval start, end;
	float refVelocity = 0.5f;
	control.setSpeed(refVelocity, refVelocity);
	
	while(ros::ok()){
		/*gettimeofday(&start, NULL);
		std_msgs::Int32 interval;
		interval.data = 10;//+rand()%991;//very random intervall [10,1000] ms
		int_pub.publish(interval);
		loop_rate.sleep();
		ros::spinOnce(); */

		std_msgs::Int32 interval;
		interval.data = 10; // we want the encoder values as fast as possible
		int_pub.publish(interval);

		/*gettimeofday(&end, NULL);

		while((end.tv_sec+(double(end.tv_usec)/1000000.0)-(start.tv_sec+(double(start.tv_usec)/1000000.0))) < 5.0){//Sleep 50 seconds
			loop_rate.sleep();
			ros::spinOnce();
			gettimeofday(&end, NULL);
		}*/
		loop_rate.sleep();
		ros::spinOnce();
		
		
		if (control.isInitialized()) {
			//std::cout << "rotate" << std::endl;
			//control.rotate(45.0f);
			//break;
			//rotateWaiter.sleep();
			//ros::spinOnce();
			control.drive();
			//printf("controlled velocity is: %f \n", refVelocity);
		} else {
			//printf("Not initialised, needed number of encoder values: %i \n", mInit);
		}	
	}
	//control.stop();
	return 0;
}
