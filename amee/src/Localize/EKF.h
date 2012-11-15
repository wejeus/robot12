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

// default noises
#define PROCESS_NOISE 0.01f
#define MEASUREMENT_NOISE 0.001f

private:

public:

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


	void init();

	void setStartPose(Matrix<float,nos,1> mu, Matrix<float,nos,nos> sigma);
	void setR(float processNoise);
	void setQ(float measurementNoise);

	Matrix<float,nos,1> g(Matrix<float,noc,1> u, Matrix<float,nos,1> mu_t_1);
	Matrix<float,nom,1> h(Matrix<float,nos,1> mu_bar, Matrix<float,nos,1> mu_bar_t_1);
	void estimate(amee::Velocity controlSignal, roboard_drivers::Encoder measurement);

	amee::Pose getMu();
	amee::Pose getSigma();
	amee::Pose getPose();

	amee::Pose mPose;	
		
};
#endif