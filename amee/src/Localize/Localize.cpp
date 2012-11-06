#include "ros/ros.h"
#include <string.h>
//#include "MotorControl.h"
#include <iostream>
#include <cmath>
#include "../Common/RobotConstants.h"
#include "Localize.h"

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
	speed.linear = (v->left + v->right)/2;
	speed.angular = (v->left - v->right)/WHEEL_BASE;
}

void Localize::receiveEncoder(const amee::Odometry::ConstPtr &odo) {
	z.left = odo->left;
	z.right = odo->right;
}

void Localize::receiveControlSignal(const amee::Velocity::ConstPtr &controlV) {
	u.left  = controlV->left;
	u.right = controlV->right;
	u.linear = (controlV->left + controlV->right)/2;
	u.angular = (controlV->left - controlV->right)/WHEEL_BASE;
}

void Localize::publishPose() {
	pose_pub.publish(pose);
}

// --- EKF functions ---
void EKF::init(){


}

void EKF::setQ(float processNoise){
	Q(0,0) = processNoise;
	Q(1,1) = processNoise;
}

void EKF::setR(float measurementNoise){
	R(0,0) = measurementNoise;
	R(1,1) = measurementNoise;
}

Vector3f EKF::g(Vector2f u, Vector3f mu_t_1) { // velocity motion model
	// move to .h?
	Vector3f mu_bar;
	Vector3f movement;

	float dt    = ; // from timestamps 
	float v     = (u(0) + u(1)) /2;
	float omega = (u(0) - u(1)) / WHEEL_BASE;

	// from probabilistic  robotics p. 127
	movement(0) = -v/omega * sin(mu_t_1(2) + v/omega * sin(mu_t_1(2) + omega*dt));
	movement(1) = +v/omega * cos(mu_t_1(2) - v/omega * cos(mu_t_1(2) + omega*dt));
	movement(2) = mu_t_1(2) + omega*dt;

	mu_bar = mu_t_1 + movement;
	return mu_bar;
}

Vector2f EKF::h(Vector3f mu_bar, Vector3f mu_bar_t_1) { // odometry motion model
	Vector3f delta_mu_bar = mu_bar - mu_bar_t_1;
	Vector2f delta_z_hat;

	delta_z_hat(0) = sqrt(pow(delta_mu_bar(0), 2) + pow(delta_mu_bar(1), 2)) + delta_mu_bar(2) * WHEEL_BASE/2;
	delta_z_hat(1) = sqrt(pow(delta_mu_bar(0), 2) + pow(delta_mu_bar(1), 2)) - delta_mu_bar(2) * WHEEL_BASE/2;

	return delta_z_hat;
}
void ekf_main(amee::Pose pose, amee::Pose poseVariance, controlSignal, measurement)
{
	// Create filter
	EKF ekf;
	ekf.init();
	ekf.serR(1.0f); // set measurement noise
	ekf.serQ(1.0f); // set process noise

	//Notation taken from Probabilistic robotics

	// --- convert to matrix form ---
	// Last pose
	Vector3f mu_t_1;
	mu_t_1(0) = pose.x;
	mu_t_1(1) = pose.y;
	mu_t_1(2) = pose.theta;

	Matrix3f::Identity sigma_t_1; // matrix from beginning

	// Control signal
	Vector2f u;
	u(0) = controlSignal.left;
	u(1) = controlSignal.rigt;

	// Last measurement
	Vector2f z_t_1;
	z_t_1(0) = z(0);
	z_t_1(1) = z(1);

	// Measurement
	Vector2f z;
	z(0) = measurement.left;
	z(1) = measurement.right;	

	// initiate (move to .h???)
	Vector3f mu_bar;    // 3by1
	Matrix3f sigma_bar; // 3by3
	Vector2f delta_z;   
	Vector2f delta_z_hat;
	Matrix3f::Identity I;

	// --- EKF algorithm ---
	mu_bar      = EFK::g(u, mu_t_1); // TODO need dt in g-function!!!!!!!!1
	sigma_bar   = G * sigma_t_1 * G.transpose() + R;
	K           = sigma_bar * H.transpose() * (H * sigma_bar * H.transpose() + Q);
	delta_z_hat = EKF::h(delta_mu_bar);
	mu          = mu_bar + K * (delta_z - delta_z_hat);
	sigma       = (I - K * H) * sigma_bar;

	// --- Copy data to pose types ---
	pose.x = mu(0);
	pose.y = mu(1);
	pose.theta = mu(2);

	poseVariance.x = sigma(0,0);
	poseVariance.y = sigma(1,1);
	poseVariance.theta = sigma(2,2);

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
