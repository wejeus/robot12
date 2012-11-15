
#include "ros/ros.h"
#include <string.h>
//#include "MotorControl.h"
#include <iostream>
#include <cmath>
#include "../Amee.h"
#include "EKF.h"
#include "Eigen/Eigen"

using namespace Eigen;

void EKF::setStartPose(Matrix<float,nos,1> pose, Matrix<float,nos,nos> variance){
	mu = pose;
	sigma = variance;
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

Matrix<float,nos,1> EKF::g(Matrix<float,noc,1> u, Matrix<float,nos,1> mu_t_1) { // velocity motion model
	// move to .h?
	Matrix<float,nos,1> movement;

	float dt     = 0.025f; // TODO: take from timestamps 
	float v      = (u(0) + u(1)) / 2.0f;
	float omega  = (u(0) - u(1)) / WHEEL_BASE;
	float radius = v/omega;

	if(omega <= 0.001) {
		radius = 1; // when omega is 0 the radius becomes inf, set it to something big instead (it will cancel out anyways)
		omega = 0;
	}

	float thetaR = mu_t_1(2)*M_PI/180.0f; 

	// Velocity motion model. From probabilistic  robotics p. 127
	movement(0) = -radius * sin(thetaR) + radius * sin(thetaR + omega*dt);
	movement(1) =  radius * cos(thetaR) - radius * cos(thetaR + omega*dt);
	movement(2) =  omega*dt;

	mu_bar = mu_t_1 + movement;

	std::cout << "v: " << v << std::endl;
	std::cout << "omega: " << omega << std::endl;
	std::cout << "radius: " << radius << std::endl;

	std::cout << "mu_t_1(0): " << mu_t_1(0) << std::endl;
	std::cout << "mu_t_1(1): " << mu_t_1(1) << std::endl;
	std::cout << "mu_t_1(2): " << mu_t_1(2) << std::endl;	

	std::cout << "movement(0): " << movement(0) << std::endl;
	std::cout << "movement(1): " << movement(1) << std::endl;
	std::cout << "movement(2): " << movement(2) << std::endl;


	return mu_bar;
}

Matrix<float,nom,1> EKF::h(Matrix<float,nos,1> mu_bar, Matrix<float,nos,1> mu_bar_t_1) { // odometry motion model
	
	// Define relative movement
	Matrix<float,nos,1> delta_mu_bar = mu_bar - mu_bar_t_1;
	Matrix<float,nom,1> z_hat;

	// Calc predicted measurement
	z_hat(0) = sqrt(pow(delta_mu_bar(0), 2) + pow(delta_mu_bar(1), 2)) + delta_mu_bar(2) * WHEEL_BASE/2;
	z_hat(1) = sqrt(pow(delta_mu_bar(0), 2) + pow(delta_mu_bar(1), 2)) - delta_mu_bar(2) * WHEEL_BASE/2;
	//z_hat(2) = delta_mu_bar(2);

	return z_hat;
}

void EKF::init(){

	Matrix<float,nos,1> pose;
	pose.setZero();
	Matrix<float,nos,nos> variance;
	variance.setZero();
	setStartPose(pose, variance);

	sigma_t_1.setZero(); // matrix from beginning
	mu_t_1.setZero();

	R = R.setIdentity() * PROCESS_NOISE;

	Q = Q.setIdentity() * MEASUREMENT_NOISE;

	I.setIdentity();

}

// rewrite to work with general matrix sizes!!!
void EKF::estimate(amee::Velocity controlSignal, roboard_drivers::Encoder measurement)
{
	// TEST printout
	std::cout << "control signal: " << controlSignal << std::endl;
	std::cout << "measurement: " << measurement << std::endl;

	//Notation taken from Probabilistic robotics

	// --- convert to matrix form --- move to function???
	// Last pose
	//mu_t_1(0) = mPose.x;
	// mu_t_1(1) = mPose.y;
	// mu_t_1(2) = mPose.theta;

	// Control signal
	u(0) = controlSignal.left;
	u(1) = controlSignal.right;

	// Measurement (movement from last measurement)
	z(0) = measurement.left;
	z(1) = measurement.right;	
	//z(2) = (measurement.left - measurement.right) / WHEEL_BASE;
	
	// --- EKF algorithm ---
	mu_bar      = EKF::g(u, mu_t_1); // TODO need dt in g-function!!!!!!!!1
	sigma_bar   = G * sigma_t_1 * G.transpose() + R;
	K           = sigma_bar * H.transpose() * (H * sigma_bar * H.transpose() + Q); // TODO K is not a float, Matrix?
	z_hat       = EKF::h(mu_bar, mu_bar_t_1);
	mu          = mu_bar + K * (z - z_hat);
	sigma       = (I - K * H) * sigma_bar;

	sigma_t_1 = sigma;
	mu_t_1 = mu;

	std::cout << "u: " << controlSignal << std::endl;
	std::cout << "z: " << measurement << std::endl;

	std::cout << "mu_bar: " << mu_bar(0) << std::endl;
	std::cout << "sigma_bar: " << sigma_bar(0,0) << std::endl;
	std::cout << "K: " << K(0) << std::endl;
	std::cout << "z_hat: " << z_hat(0) << std::endl;
	std::cout << "mu: " << mu(0) << std::endl;
	std::cout << "sigma: " << sigma(0) << std::endl;

	// --- Copy data to pose types ---
	mPose.x = mu(0);
	mPose.y = mu(1);
	mPose.theta = mu(2);

	mPose.xVar = sigma(0,0);
	mPose.yVar = sigma(1,1);
	mPose.thetaVar = sigma(2,2);

}
