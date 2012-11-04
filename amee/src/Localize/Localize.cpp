#include "ros/ros.h"
#include <string.h>
#include "MotorControl.h"
#include <iostream>
#include <cmath>

void Localize::init() 
{
	//mMeasurementValidCounter = 2 * NUM_AVERAGED_MEASUREMENTS;
	//mMeasurementCounter = 0;

	ros::Subscriber	enc_sub;
	ros::Subscriber	motorSpeed_sub;

	ros::Publisher pose_pub

	ros::Rate loop_rate(40);

}

void Localize::receiveSpeed(const amee::Velocity::ConstPtr &v) {
	speed.left  = v->left;
	speed.right = v->right;
}

void Localize::receiveEncoder(const amee::Odometry::ConstPtr &odo) {
	encoder.left = odo->left;
	encoder.right = odo->right;
}

void Localize::receiveControlSignal(const amee::Velocity::ConstPtr &controlV) {
	controlSignal.left  = controlV->left;
	controlSignal.right = controlV->right;

}

void Localize::publishPose() {
	pose_pub.publish(pose);
}

// --- EKF functions ---
void Localize::g(amee::Velocity u, amee::Pose mu_t_1) { // velocity motion model
	float dt = // from timestamps 

	mu_t_1bar.x     = mu_t_1.x     - u.linear/u.omega*sin(mu_t_1.theta) + v/w*sin(mu_t_1.theta+u.omega*dt);
	mu_bar.y     = mu_t_1.y     + u.linear/u.omega*cos(mu_t_1.theta) - v/w*cos(mu_t_1.theta+u.omega*dt);
	mu_bar.theta = mu_t_1.theta + u.omega*dt;
}

void Localize::h(z, z_t_1, mu_bar, mu_bar_t_1) { // odometry motion model


}
void Localize::estimatePose()
{
	//Notation tagen from Probabilistic robotics
	float mu_bar [3];
	float sigma_bar [3][3];
	float sigma [3][3];

	mu_bar[0] = pose.x;
	mu_bar[1] = pose.y;
	mu_bar[2] = pose.theta;

	expectedCov[0][0] = 1; 
	pose = motionnModel(encoder, lastEncoder)
	expectedVar = 
}

int main(int argc, char **argv) 
{
	// -- Initiate ROS node
	ros::init(argc, argv, "Localize");//Creates a node named "Localize"
	ros::NodeHandle n;

	// -- Initialize localizer
	Localize lozalize;
	localize.init();

	// -- Subscribe & Publish topics
	enc_sub        = n.subscribe("/serial/encoder", 100, &Localize::receiveEncoder, &localize);         // Subscribe encoder
	motorSpeed_sub = n.subscribe("/serial/motor_speed", 100, &Localize::receiveMotorSpeed, &localize);  // Subscribe motorSpeed
	pose_pub       = n.advertise<Pose>("/pose", 100);                                                   // Publish pose
	

	// -- Estimate pose
	while()
	{
		loop_rate.sleep(); // go to sleep for a short while
		ros::spinOnce();   // call all callbacks
		localize.estimatePose(); // estimate pose
		
	}


	// -- Publish pose
	//localize.publishPose();

}