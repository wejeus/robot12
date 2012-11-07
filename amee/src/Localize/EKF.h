#ifndef EKF_H
#define EKF_H

#include <std_msgs/Int32.h>
#include "amee/Velocity.h"
#include "amee/Odometry.h"
#include "amee/Pose.h"
#include "Eigen/Eigen"

class EKF {

#define X 0.0f

private:

public:

	// EKF variables
	Eigen::Vector3f mu;
	Eigen::Vector3f mu_t_1;
	Eigen::Vector3f mu_bar;
	Eigen::Vector3f mu_bar_t_1;
	
	Eigen::Matrix3f sigma;
	Eigen::Matrix3f sigma_bar;
	Eigen::Matrix3f sigma_t_1;
	
	Eigen::Vector2f u;
	Eigen::Vector2f u_t_1;	
	
	Eigen::Vector3f z;
	//Eigen::Vector3f z_t_1;
	Eigen::Vector3f z_hat;

	Eigen::Matrix3f G; // jacobian of g (motion model)
	Eigen::Matrix3f H; // Jacobian of h ()

	Eigen::Matrix3f K; // Kalman gain

	Eigen::Matrix3f R; // Measurement noice
	Eigen::Matrix3f Q; // Process noice

	Eigen::Matrix3f I;

	//void init();

	void setR(float processNoise);
	void setQ(float measurementNoise);

	Eigen::Vector3f g(Eigen::Vector2f u, Eigen::Vector3f mu_t_1);
	Eigen::Vector3f h(Eigen::Vector3f mu_bar, Eigen::Vector3f mu_bar_t_1);
	void estimate(amee::Pose pose, amee::Velocity controlSignal, amee::Odometry measurement);

	amee::Pose getMu();
	amee::Pose getSigma();
	amee::Pose getPose();

	amee::Pose pose;


protected:
	
		
};
#endif