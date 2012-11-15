#ifndef EKF_H
#define EKF_H

#include <std_msgs/Int32.h>
#include "amee/Velocity.h"
#include "amee/Odometry.h"
#include "amee/Pose.h"
#include "Eigen/Eigen"
#include "roboard_drivers/Encoder.h"

using namespace Eigen;

class EKF {

#define X 0.0f

// number of states, measurements and controls (s,m,c)
#define nos 3
#define nom 2
#define noc 2

private:

public:

	// // EKF variables
	// Eigen::Vector3f mu;
	// Eigen::Vector3f mu_t_1;
	// Eigen::Vector3f mu_bar;
	// Eigen::Vector3f mu_bar_t_1;
	
	// Eigen::Matrix3f sigma;
	// Eigen::Matrix3f sigma_bar;
	// Eigen::Matrix3f sigma_t_1;
	
	// Eigen::Vector2f u;
	// Eigen::Vector2f u_t_1;	
	
	// Eigen::Vector3f z;
	// Eigen::Vector3f z_hat;

	// Eigen::Matrix3f G; // jacobian of g (motion model)
	// Eigen::Matrix3f H; // Jacobian of h ()

	// Eigen::Matrix3f K; // Kalman gain

	// Eigen::Matrix3f R; // Measurement noice
	// Eigen::Matrix3f Q; // Process noice

	// Eigen::Matrix3f I;

	// EKF variablesx
	Matrix<float,nos,1> mu;
	Matrix<float,nos,1> mu_t_1;
	Matrix<float,nos,1> mu_bar;
	Matrix<float,nos,1> mu_bar_t_1;

	Matrix<float,nos,nos> sigma;
	Matrix<float,nos,nos> sigma_bar;
	Matrix<float,nos,nos> sigma_t_1;

	Matrix<float,noc,1> u;
	Matrix<float,noc,1> u_t_1;

	Matrix<float,nom,1> z;
	Matrix<float,nom,1> z_hat;
	
	Matrix<float,nos,nos> G; // jacobian of g (motion model)
	Matrix<float,nom,nos> H; // Jacobian of h ()

	Matrix<float,nos,nom> K; // Kalman gain

	Matrix<float,nos,nos> R; // Process noice
	Matrix<float,nom,nom> Q; // Measurement noice

	Matrix<float,nos,nos> I;


	void init(int numberOfStates, int numberOfMeasurements);

	void setR(float processNoise);
	void setQ(float measurementNoise);

	Matrix<float,nos,1> g(Matrix<float,noc,1> u, Matrix<float,nos,1> mu_t_1);
	Matrix<float,nom,1> h(Matrix<float,nos,1> mu_bar, Matrix<float,nos,1> mu_bar_t_1);
	void estimate(amee::Velocity controlSignal, roboard_drivers::Encoder measurement);

	amee::Pose getMu();
	amee::Pose getSigma();
	amee::Pose getPose();

	void setStartPose(amee::Pose pose);

	amee::Pose mPose;	
		
};
#endif