
#include "ros/ros.h"
#include <string.h>
//#include "MotorControl.h"
#include <iostream>
#include <cmath>
#include <limits>

#include "../Amee.h"
#include "EKF.h"
#include "Eigen/Eigen"


// default noises 
#define PROCESS_NOISE 0.001f
#define MEASUREMENT_NOISE 1.0f

using namespace Eigen;

void EKF::init(){

	Matrix<float,nos,1> pose;
	pose.setZero();
	Matrix<float,nos,nos> variance;
	variance.setZero();
	setStartPose(pose, variance);

	sigma_t_1.setZero(); // matrix from beginning
	mu_t_1.setZero();
	z_t_1.setZero();

	R = R.setIdentity() * PROCESS_NOISE;
	Q = Q.setIdentity() * MEASUREMENT_NOISE;

	I.setIdentity();

	// set H to "identity" 
	H.setZero();
	H(0,0) = 1;
	H(1,1) = 1;
	H(2,2) = 1;
	H(3,0) = 1;
	H(4,1) = 1;
	H(5,2) = 1;
}

void EKF::setStartPose(Matrix<float,nos,1> pose, Matrix<float,nos,nos> variance){
	mu = pose;
	sigma = variance;
}

void EKF::setR(Matrix<float,nos,1> processNoise){
	for(int i=0;i<nos;i++){
		Q(i,i) = processNoise(i,1);
	}
}

void EKF::setQ(Matrix<float,nom,1> measurementNoise){
	for(int i=0;i<nom;i++){
		R(i,i) = measurementNoise(i,1);
	} 
}

void EKF::setH(Matrix<float,nom,nos> hJacobian) {
	H = hJacobian;
}

void EKF::setG(Matrix<float,nos,nos> gJacobian) {
	G = gJacobian;
}

void EKF::setMeasurement1(amee::Pose measurement1) {
	mMeasurement1 = measurement1;
}

void EKF::setMeasurement2(amee::Pose measurement2) {
	mMeasurement2 = measurement2;
}

int EKF::getNos(){
	return nos;
}

int EKF::getNom(){
	return nom;
}

int EKF::getNoc(){
	return noc;
}

amee::Pose EKF::getPose() {
	return mPose;
}

Matrix<float,nos,1> EKF::g(Matrix<float,noc,1> u, Matrix<float,nos,1> mu_t_1) { // velocity motion model

	// inputs
	// u:      velocity right/left (m/s) (pwm????????)
	// mu_t_1: prev. state x,y,theta (m, m, degrees)

	Matrix<float,nos,1> movement;

	float dt     = 0.025f; // TODO: take from timestamps 
	float v      = (u(0) + u(1)) / 2.0f;
	float omega  = (u(0) - u(1)) / WHEEL_BASE;
	float radius = v/omega;

	if(omega <= 0.001f) {
		radius = 1.0f; // when omega is 0 the radius becomes inf, set it to something big instead (it will cancel out anyways)
		omega = 0.0f;
	}

	float thetaR = mu_t_1(2) * M_PI/180.0f; // theta in radians (cos and sine takes in radians)
	float omegaR = omega * M_PI/180.0f;

	// Velocity motion model. From probabilistic  robotics p. 127
	movement(0) = -radius * sin(thetaR) + radius * sin(thetaR + omegaR*dt);
	movement(1) =  radius * cos(thetaR) - radius * cos(thetaR + omegaR*dt);
	movement(2) =  omega*dt * 180.0f/M_PI;

	mu_bar = mu_t_1 + movement;

	// Debug output
	std::cout << "v: " << v << std::endl;
	std::cout << "omega: " << omega << std::endl;
	std::cout << "radius: " << radius << std::endl;
	std::cout << "thetaR: " << thetaR << std::endl;
	std::cout << "omegaR: " << omegaR << std::endl;

	std::cout << "movement(0): " << movement(0) << std::endl;
	std::cout << "movement(1): " << movement(1) << std::endl;
	std::cout << "movement(2): " << movement(2) << std::endl;

	return mu_bar;
}

// Matrix<float,nom,1> EKF::h(Matrix<float,nos,1> mu_bar, Matrix<float,nos,1> mu_t_1, Matrix<float,nom,1> z_t_1) { // odometry motion model
	
// 	// Define relative movement
// 	Matrix<float,nos,1> delta_mu_bar = mu_bar - mu_t_1;
// 	Matrix<float,nom,1> z_hat;

// 	// Calc predicted measurement
// 	z_hat(0) = sqrt(pow(delta_mu_bar(0), 2) + pow(delta_mu_bar(1), 2)) + delta_mu_bar(2) * WHEEL_BASE/2.0f;
// 	z_hat(1) = sqrt(pow(delta_mu_bar(0), 2) + pow(delta_mu_bar(1), 2)) - delta_mu_bar(2) * WHEEL_BASE/2.0f;

// 	// convert z_hat to encoder tics
// 	z_hat = z_hat * TICS_PER_REVOLUTION / (2.0f * M_PI * WHEEL_RADIUS);
// 	z_hat(0) = floor(z_hat(0));
// 	z_hat(1) = floor(z_hat(1));

// 	std::cout << "predicted measurement: " << z_hat(0) + z_t_1(0) << "   " << z_hat(1) + z_t_1(1) << std::endl;

// 	return z_hat + z_t_1; // Total encoder count
// }

// Matrix<float,nom,nos> EKF::hJac(Matrix<float,nos,1> mu_bar, Matrix<float,nos,1> mu_t_1) {

// 	//Matrix<float,nos,1> delta_mu_bar = mu_bar - mu_t_1;
// 	float thetaR = mu_t_1(2) * M_PI/180.0f; // theta in radians (cos and sine takes in radians)
// 	//thetaR = thetaR - floor(thetaR / (2.0f*M_PI)) * 2.0f*M_PI;

// 	std::cout << "thetaR : " << thetaR << std::endl;

// 	// if division by zero (cos-term = 0)
// 	if((thetaR >= 0.0f && thetaR <= M_PI/4.0f) || (thetaR >= M_PI*3.0f/4.0f && thetaR <= M_PI*5.0f/4.0f) 
// 		                                       || (thetaR >= M_PI*7.0f/4.0f && thetaR <= M_PI*2.0f)){
// 		// change of encoder left wrt x,y,theta		
// 		H(0,0) = 1/cos(thetaR);
// 		H(0,1) = 0;
// 		H(0,2) = -WHEEL_BASE/4.0f;

// 		// change of encoder right wrt x,y,theta
// 		H(1,0) = 1/cos(thetaR);
// 		H(1,1) = 0;
// 		H(1,2) = WHEEL_BASE/4.0f;

// 	} else {

// 		// change of encoder left wrt x,y,theta
// 		H(0,0) = 0;
// 		H(0,1) = 1/sin(thetaR);
// 		H(0,2) = -WHEEL_BASE/4.0f;

// 		// change of encoder right wrt x,y,theta
// 		H(1,0) = 0;
// 		H(1,1) = 1/sin(thetaR);
// 		H(1,2) = WHEEL_BASE/4.0f;
// 	}
// 		// convert to tics
// 	H = H * TICS_PER_REVOLUTION/(M_PI*WHEEL_BASE);

// 	// // H is the Jacobian of h-function
// 	// H(0,0) = 1.0f/sqrt(pow(delta_mu_bar(0), 2) + pow(delta_mu_bar(1), 2)) * delta_mu_bar(0); // d z_bar(0) / d x
// 	// H(0,1) = 1.0f/sqrt(pow(delta_mu_bar(0), 2) + pow(delta_mu_bar(1), 2)) * delta_mu_bar(1); // d z_bar(0) / d y
// 	// H(0,2) = WHEEL_BASE/2.0f; // d z_bar(0) / d theta

// 	// H(1,0) = 1.0f/sqrt(pow(delta_mu_bar(0), 2) + pow(delta_mu_bar(1), 2)) * delta_mu_bar(0); // d z_bar(1) / d x
// 	// H(1,1) = 1.0f/sqrt(pow(delta_mu_bar(0), 2) + pow(delta_mu_bar(1), 2)) * delta_mu_bar(1); // d z_bar(1) / d y
// 	// H(1,2) = -WHEEL_BASE/2.0f; // d z_bar(1) / d theta		

// 	return H;
// }

// rewrite to work with general matrix sizes!!!
void EKF::estimate(amee::Velocity controlSignal, amee::Pose measurement1, amee::Pose measurement2)
{

	std::cout << "---- New iteration ----" << std::endl;
	// TEST printout
//	std::cout << "- Control signal: " << controlSignal.left << "   " << controlSignal.right << std::endl;
//	std::cout << "- Measurement: " << measurement.left << "   " << measurement.right << std::endl;

	//Notation taken from Probabilistic robotics

	// --- convert to matrix form --- move to function???

	// Control signal
	u(0) = controlSignal.left;
	u(1) = controlSignal.right;

	// Measurement (movement from last measurement)
	// z(0) = measurement.left;
	// z(1) = measurement.right;	
	z(0) = measurement1.x;
	z(1) = measurement1.y;
	z(2) = measurement1.theta;
	z(3) = measurement2.x;
	z(4) = measurement2.y;
	z(5) = measurement2.theta;
	
	std::cout << "- Functions print out " << std::endl;

	// --- EKF algorithm ---
	// Predict state
	mu_bar    = EKF::g(u, mu_t_1); // TODO need dt in g-function!!!!!!!!1
	sigma_bar = G * sigma_t_1 * G.transpose() + R;

	// Calc 
	//H     = hJac(mu_bar, mu_t_1); 								     		 // Jacobian of h
	K = sigma_bar * H.transpose() * (H * sigma_bar * H.transpose() + Q); // Kalman gain
	
	//z_hat = EKF::h(mu_bar, mu_t_1, z_t_1);									 // Predicted measurement
	z_hat(0) = mPose.x;
	z_hat(1) = mPose.y;
	z_hat(2) = mPose.theta;
	z_hat(3) = mPose.x;
	z_hat(4) = mPose.y;
	z_hat(5) = mPose.theta;

	// Update state
	mu    = mu_bar + K * (z - z_hat);
	sigma = (I - K * H) * sigma_bar;

	// Save last state
	sigma_t_1 = sigma;
	mu_t_1    = mu;
	z_t_1     = z;

	std::cout << std::endl << "- Algorithm print out " << std::endl;


	// Debug output
	std::cout << "u: " << controlSignal.left << "   " << controlSignal.right << std::endl;

	std::cout << "mu_t_1(0): " << mu_t_1(0) << "   " << mu_t_1(1) << "   " << mu_t_1(2) << std::endl;	
	std::cout << "mu_bar(0): " << mu_bar(0) << "   " << mu_bar(1) << "   " << mu_bar(2) << std::endl;	

	std::cout << "sigma_bar(0,0): " << sigma_bar(0,0) << "   " << sigma_bar(1,1) << "   " << sigma_bar(2,2) << std::endl;
	std::cout << "sigma_t_1(0,0): " << sigma_t_1(0,0) << "   " << sigma_t_1(1,1) << "   " << sigma_t_1(2,2) << std::endl;

	std::cout << "H: " << H(0,0) << "  " << H(0,1) << "  " << H(0,2) << std::endl;
	std::cout << "H: " << H(1,0) << "  " << H(1,1) << "  " << H(1,2) << std::endl;

	std::cout << "K: " << K(0,0) << "  " << K(0,1) << std::endl;
	std::cout << "K: " << K(1,0) << "  " << K(1,1) << std::endl;
	std::cout << "K: " << K(2,0) << "  " << K(2,1) << std::endl;

//	std::cout << "z: " << measurement.left << "   " << measurement.right << std::endl;
	std::cout << "z_hat: " << z_hat(0) << "    " << z_hat(1) << std::endl;

	std::cout << "mu: " << mu_t_1(0) << "   " << mu_t_1(1) << "   " << mu_t_1(2) << std::endl;
	std::cout << "sigma: " << sigma(0,0) << "   " << sigma(1,1) << "   " << sigma(2,2) << "   " << std::endl;

	// --- Copy data to pose types ---
	mPose.x = mu(0);
	mPose.y = mu(1);
	mPose.theta = mu(2);

	mPose.xVar = sigma(0,0);
	mPose.yVar = sigma(1,1);
	mPose.thetaVar = sigma(2,2);

}
