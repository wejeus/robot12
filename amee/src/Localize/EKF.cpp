
#include "ros/ros.h"
#include <string.h>
//#include "MotorControl.h"
#include <iostream>
#include <cmath>
#include "../Amee.h"
#include "EKF.h"
#include "Eigen/Eigen"

void EKF::setStartPose(amee::Pose mPose){
	mPose = mPose;
}

void EKF::setQ(float processNoise){
	Q(0,0) = processNoise;
	Q(1,1) = processNoise;
}

void EKF::setR(float measurementNoise){
	R(0,0) = measurementNoise;
	R(1,1) = measurementNoise;
}

amee::Pose EKF::getPose() {
	return mPose;
}

Eigen::Vector3f EKF::g(Eigen::Vector2f u, Eigen::Vector3f mu_t_1) { // velocity motion model
	// move to .h?
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

void EKF::init(){
	mPose.x = 0;
	mPose.y = 0;
	mPose.theta = 0;
	mPose.xVar = 0;
	mPose.yVar = 0;
	mPose.thetaVar = 0;
	setStartPose(mPose);

	float measurementNoise = 0.1f;
	R(0,0) = measurementNoise;
	R(1,1) = measurementNoise;

	float processNoise = 0.1f;
	Q(0,0) = processNoise;
	Q(1,1) = processNoise;

	G.Identity();
	H.Identity();


}

void EKF::estimate(amee::Velocity controlSignal, amee::Odometry measurement)
{
	//Notation taken from Probabilistic robotics

	// --- convert to matrix form --- move to function???
	// Last pose
	mu_t_1(0) = mPose.x;
	mu_t_1(1) = mPose.y;
	mu_t_1(2) = mPose.theta;

	// Sigma start
	sigma_t_1.Identity();	// matrix from beginning

	// Control signal
	u(0) = controlSignal.left;
	u(1) = controlSignal.right;

	// Measurement (movement from last measurement)
	z(0) = measurement.leftWheelDistance;
	z(1) = measurement.rightWheelDistance;	
	z(2) = (measurement.leftWheelDistance - measurement.rightWheelDistance) / WHEEL_BASE;
	
	I.Identity();

	// --- EKF algorithm ---
	mu_bar      = EKF::g(u, mu_t_1); // TODO need dt in g-function!!!!!!!!1
	sigma_bar   = G * sigma_t_1 * G.transpose() + R;
	K           = sigma_bar * H.transpose() * (H * sigma_bar * H.transpose() + Q); // TODO K is not a float, Matrix?
	z_hat       = EKF::h(mu_bar, mu_bar_t_1);
	mu          = mu_bar + K * (z - z_hat);
	sigma       = (I - K * H) * sigma_bar;

	// --- Copy data to pose types ---
	mPose.x = mu(0);
	mPose.y = mu(1);
	mPose.theta = mu(2);

	mPose.xVar = sigma(0,0);
	mPose.yVar = sigma(1,1);
	mPose.thetaVar = sigma(2,2);

}