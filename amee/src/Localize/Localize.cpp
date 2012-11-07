#include "ros/ros.h"
#include <string.h>
//#include "MotorControl.h"
#include <iostream>
#include <cmath>
#include "../Common/RobotConstants.h"
#include "Localize.h"
#include "EKF.h"
//#include "roboard_drivers/Encoder"

void Localize::init() 
{


}

void Localize::receiveMotorSpeed(const amee::Velocity::ConstPtr &v) {
	speed.left  = v->left;
	speed.right = v->right;
	speed.linear = (v->left + v->right)/2;
	speed.angular = (v->left - v->right)/WHEEL_BASE;
}

void Localize::receiveEncoder(const amee::Odometry::ConstPtr &msg) {
	encoderMeasurement.leftWheelDistance = msg->leftWheelDistance;
	encoderMeasurement.rightWheelDistance = msg->rightWheelDistance;
}

void Localize::receiveControlSignal(const amee::Velocity::ConstPtr &ctrl) {
	controlSignal.left  = ctrl->left;
	controlSignal.right = ctrl->right;
	controlSignal.linear = (ctrl->left + ctrl->right)/2;
	controlSignal.angular = (ctrl->left - ctrl->right)/WHEEL_BASE;
}

void Localize::publishPose(ros::Publisher pose_pub) {
	pose_pub.publish(pose);
}

// --- EKF functions ---
void EKF::setQ(float processNoise){
	Q(0,0) = processNoise;
	Q(1,1) = processNoise;
}

void EKF::setR(float measurementNoise){
	R(0,0) = measurementNoise;
	R(1,1) = measurementNoise;
}

amee::Pose EKF::getPose() {
	return pose;
}

Eigen::Vector3f EKF::g(Eigen::Vector2f u, Eigen::Vector3f mu_t_1) { // velocity motion model
	// move to .h?
	//Eigen::Vector3f mu_bar;
	Eigen::Vector3f movement;

	float dt    = 1; // TODO: take from timestamps 
	float v     = (u(0) + u(1)) /2;
	float omega = (u(0) - u(1)) / WHEEL_BASE;

	// Velocity motion model. From probabilistic  robotics p. 127
	movement(0) = -v/omega * sin(mu_t_1(2) + v/omega * sin(mu_t_1(2) + omega*dt));
	movement(1) = +v/omega * cos(mu_t_1(2) - v/omega * cos(mu_t_1(2) + omega*dt));
	movement(2) = mu_t_1(2) + omega*dt;

	mu_bar = mu_t_1 + movement;
	return mu_bar;
}

Eigen::Vector3f EKF::h(Eigen::Vector3f mu_bar, Eigen::Vector3f mu_bar_t_1) { // odometry motion model
	
	// Define relative movement
	Eigen::Vector3f delta_mu_bar = mu_bar - mu_bar_t_1;
	Eigen::Vector3f z_hat;

	// Calc predicted measurement
	z_hat(0) = sqrt(pow(delta_mu_bar(0), 2) + pow(delta_mu_bar(1), 2)) + delta_mu_bar(2) * WHEEL_BASE/2;
	z_hat(1) = sqrt(pow(delta_mu_bar(0), 2) + pow(delta_mu_bar(1), 2)) - delta_mu_bar(2) * WHEEL_BASE/2;
	z_hat(2) = delta_mu_bar(2);

	return z_hat;
}
void EKF::estimate(amee::Pose pose, amee::Velocity controlSignal, amee::Odometry measurement)
{
	//Notation taken from Probabilistic robotics

	// --- convert to matrix form --- move to function???
	// Last pose
	//Eigen::Vector3f mu_t_1;
	mu_t_1(0) = pose.x;
	mu_t_1(1) = pose.y;
	mu_t_1(2) = pose.theta;

	//Eigen::Matrix3f sigma_t_1; 
	sigma_t_1.Identity();	// matrix from beginning

	// Control signal
	//Eigen::Vector2f u;
	u(0) = controlSignal.left;
	u(1) = controlSignal.right;

	// Last measurement
	//Eigen::Vector3f z_t_1;
	//z_t_1(0) = z(0);
	//z_t_1(1) = z(1);
	//z_t_1(2) = z(2);

	// Measurement (movement from last measurement)
	//Eigen::Vector3f z;
	z(0) = measurement.leftWheelDistance;
	z(1) = measurement.rightWheelDistance;	
	z(2) = (measurement.leftWheelDistance - measurement.rightWheelDistance) / WHEEL_BASE;

	// initiate (move to .h???)
	//Eigen::Vector3f mu_bar;    // 3by1
	//Eigen::Matrix3f sigma_bar; // 3by3
	//Eigen::Vector3f delta_z = z - z_t_1;   
	//Eigen::Vector3f delta_z_hat;
	//Eigen::Matrix3f I;
	
	I.Identity();

	// --- EKF algorithm ---
	mu_bar      = EKF::g(u, mu_t_1); // TODO need dt in g-function!!!!!!!!1
	sigma_bar   = G * sigma_t_1 * G.transpose() + R;
	K           = sigma_bar * H.transpose() * (H * sigma_bar * H.transpose() + Q); // TODO K is not a float, Matrix?
	z_hat       = EKF::h(mu_bar, mu_bar_t_1);
	mu          = mu_bar + K * (z - z_hat);
	sigma       = (I - K * H) * sigma_bar;

	// --- Copy data to pose types ---
	pose.x = mu(0);
	pose.y = mu(1);
	pose.theta = mu(2);

	pose.xVar = sigma(0,0);
	pose.yVar = sigma(1,1);
	pose.thetaVar = sigma(2,2);

}

int main(int argc, char **argv) 
{
	// -- Initiate ROS node
	ros::init(argc, argv, "Localize"); //Creates a node named "Localize"
	ros::NodeHandle n;

	// -- Initialize localizer
	Localize localize;
	localize.init();

	// -- Subscribe & Publish topics
	ros::Subscriber enc_sub;
	ros::Subscriber motorSpeed_sub;
	ros::Subscriber controlSignal_sub;
	ros::Publisher  pose_pub;

	enc_sub           = n.subscribe("/serial/encoder", 100, &Localize::receiveEncoder, &localize);              // Subscribe encoder
	motorSpeed_sub    = n.subscribe("/serial/motor_speed", 100, &Localize::receiveMotorSpeed, &localize);     // Subscribe motorSpeed
	controlSignal_sub = n.subscribe("/serial/motor_speed", 100, &Localize::receiveControlSignal, &localize); // Subscribe contrlSignal

	pose_pub = n.advertise<amee::Pose>("/pose", 100); // Publish pose
	
	ros::Rate loop_rate(40);

	// Create filter
	EKF ekf;
	//ekf.init();
	ekf.setR(1.0f); // set measurement noise
	ekf.setQ(1.0f); // set process noise
	ekf.G.Identity();
	ekf.H.Identity();

	// -- Estimate pose
	while(true)
	{
		localize.lastEncoderMeasurement = localize.encoderMeasurement; // Save last before spinOnce

		ros::spinOnce();   // call all callbacks

		localize.measurement.leftWheelDistance  = localize.encoderMeasurement.leftWheelDistance  - localize.lastEncoderMeasurement.leftWheelDistance;
		localize.measurement.rightWheelDistance = localize.encoderMeasurement.rightWheelDistance - localize.lastEncoderMeasurement.rightWheelDistance;
		//localize.measurement.IMUtheta = ;...
		
		ekf.estimate(localize.pose, localize.controlSignal, localize.measurement); // have pose in the init function instead!
		loop_rate.sleep(); // go to sleep for a short while

	}

	// -- Publish pose
	localize.publishPose(pose_pub);

	return 0;
}
