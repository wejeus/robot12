#include "ros/ros.h"
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
{
	int right = msg->right;
	int left = msg->left;
	double timestamp = msg->timestamp;
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
}

int main(int argc, char **argv)
{
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
	return 0;
}
