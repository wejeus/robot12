
#include "ros/ros.h"
#include <string.h>
//#include "MotorControl.h"
#include <iostream>
#include <cmath>
#include "../Amee.h"
#include "EKF.h"
#include "Eigen/Eigen"

#define PROCESS_NOISE 0.1f
#define MEASUREMENT_NOISE 0.1f


using namespace Eigen;

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

Matrix<float,nos,1> EKF::g(Matrix<float,noc,1> u, Matrix<float,nos,1> mu_t_1) { // velocity motion model
	// move to .h?
	Matrix<float,nos,1> movement;

	float dt    = 0.025; // TODO: take from timestamps 
	float v     = (u(0) + u(1)) /2;
	float omega = (u(0) - u(1)) / WHEEL_BASE;
	float radius = v/omega;

	if(omega <= 0.001) {
		radius = 10000; // when omega is 0 the radius becomes inf, set it to something big instead (it will cancel out anyways)
		omega = 0;
	}

	// Velocity motion model. From probabilistic  robotics p. 127
	movement(0) = -radius * sin(mu_t_1(2) + radius * sin(mu_t_1(2) + omega*dt));
	movement(1) = +radius * cos(mu_t_1(2) - radius * cos(mu_t_1(2) + omega*dt));
	movement(2) = mu_t_1(2) + omega*dt;

	mu_bar = mu_t_1 + movement;
	return mu_bar;
}

Matrix<float,nom,1> EKF::h(Matrix<float,nos,1> mu_bar, Matrix<float,nos,1> mu_bar_t_1) { // odometry motion model
	
	// Define relative movement
	Matrix<float,nos,1> delta_mu_bar = mu_bar - mu_bar_t_1;
	Matrix<float,nom,1> z_hat;

	// Calc predicted measurement
	z_hat(0) = sqrt(pow(delta_mu_bar(0), 2) + pow(delta_mu_bar(1), 2)) + delta_mu_bar(2) * WHEEL_BASE/2;
	z_hat(1) = sqrt(pow(delta_mu_bar(0), 2) + pow(delta_mu_bar(1), 2)) - delta_mu_bar(2) * WHEEL_BASE/2;
	z_hat(2) = delta_mu_bar(2);

	return z_hat;
}

#define nos 3
#define nom 2
#define noc 2

void EKF::init(int numberOfStates, int numberOfMeasurements){
	//int nos = numberOfStates;
	//int nom = numberOfMeasurements;
	//int noc = 2; // number of control (right & left wheel speed)
	
	// // EKF variablesx
	// Matrix<float,nos,1> mu;
	// Matrix<float,nos,1> mu_t_1;
	// Matrix<float,nos,1> mu_bar;
	// Matrix<float,nos,1> mu_bar_t_1;

	// Matrix<float,nos,nos> sigma;
	// Matrix<float,nos,nos> sigma_bar;
	// Matrix<float,nos,nos> sigma_t_1;

	// Matrix<float,noc,1> u;
	// Matrix<float,noc,1> u_t_1;

	// Matrix<float,nom,1> z;
	// Matrix<float,nom,1> z_hat;
	
	// Matrix<float,nos,nos> G; // jacobian of g (motion model)
	// Matrix<float,nom,nos> H; // Jacobian of h ()


	// Matrix<float,nos,nom> K; // Kalman gain

	// Matrix<float,nos,nos> R; // Process noice
	// Matrix<float,nom,nom> Q; // Measurement noice

	// Matrix<float,nos,nos> I;
	// I = I.Identity();

	// mPose.x = 0;
	// mPose.y = 0;
	// mPose.theta = 0;
	// mPose.xVar = 0;
	// mPose.yVar = 0;
	// mPose.thetaVar = 0;
	// setStartPose(mPose);

	// R = R.diagonal();
	// //R * PROCESS_NOISE;

	// Q = Q.diagonal() * MEASUREMENT_NOISE;

//	G.Identity();
//	H.Identity();


	mPose.x = 0;
	mPose.y = 0;
	mPose.theta = 0;
	mPose.xVar = 0;
	mPose.yVar = 0;
	mPose.thetaVar = 0;
	setStartPose(mPose);

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
	mu_t_1(0) = mPose.x;
	mu_t_1(1) = mPose.y;
	mu_t_1(2) = mPose.theta;

	// Sigma start
	sigma_t_1.Identity();	// matrix from beginning

	// Control signal
	u(0) = controlSignal.left;
	u(1) = controlSignal.right;

	// Measurement (movement from last measurement)
	z(0) = measurement.left;
	z(1) = measurement.right;	
	z(2) = (measurement.left - measurement.right) / WHEEL_BASE;
	
	I.Identity();

	// --- EKF algorithm ---
	mu_bar      = EKF::g(u, mu_t_1); // TODO need dt in g-function!!!!!!!!1
	sigma_bar   = G * sigma_t_1 * G.transpose() + R;
	K           = sigma_bar * H.transpose() * (H * sigma_bar * H.transpose() + Q); // TODO K is not a float, Matrix?
	z_hat       = EKF::h(mu_bar, mu_bar_t_1);
	mu          = mu_bar + K * (z - z_hat);
	sigma       = (I - K * H) * sigma_bar;

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
