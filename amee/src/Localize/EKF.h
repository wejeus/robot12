#ifndef EKF_H
#define EKF_H

#include <std_msgs/Int32.h>
#include "amee/Velocity.h"
#include "amee/Odometry.h"
#include "amee/Pose.h"
#include "Eigen/Eigen"
#include "roboard_drivers/Encoder.h"

//using namespace Eigen;

namespace amee {

	class EKF {

	#define X 0.0f

	// number of states, measurements and controls (s,m,c)
	#define nos 3
	#define nom 6
	#define noc 2

	private:

	public:

		// EKF variablesx
		Eigen::Matrix<float,nos,1> mu;
		Eigen::Matrix<float,nos,1> mu_t_1;
		Eigen::Matrix<float,nos,1> mu_bar;
		//Eigen::Matrix<float,nos,1> mu_bar_t_1;

		Eigen::Matrix<float,nos,nos> sigma;
		Eigen::Matrix<float,nos,nos> sigma_bar;
		Eigen::Matrix<float,nos,nos> sigma_t_1;

		Eigen::Matrix<float,noc,1> u;
		Eigen::Matrix<float,noc,1> u_t_1;

		Eigen::Matrix<float,nom,1> z;
		Eigen::Matrix<float,nom,1> z_hat;
		Eigen::Matrix<float,nom,1> z_t_1;
		
		Eigen::Matrix<float,nos,nos> G; // jacobian of g (motion model)
		Eigen::Matrix<float,nom,nos> H; // Jacobian of h ()

		Eigen::Matrix<float,nos,nom> K; // Kalman gain

		Eigen::Matrix<float,nos,nos> R; // Process noice
		Eigen::Matrix<float,nom,nom> Q; // Measurement noice

		Eigen::Matrix<float,nos,nos> I;


		void init();

		void setStartPose(Eigen::Matrix<float,nos,1> mu, Eigen::Matrix<float,nos,nos> sigma);

		void setR(Eigen::Matrix<float,nos,1> processNoise);
		void setQ(Eigen::Matrix<float,nom,1> measurementNoise);

		void setG(Eigen::Matrix<float,nos,nos> gJacobian);
		void setH(Eigen::Matrix<float,nom,nos> hJacobian);

		Eigen::Matrix<float,nos,1> g(Eigen::Matrix<float,noc,1> u, Eigen::Matrix<float,nos,1> mu_t_1);
		Eigen::Matrix<float,nom,1> h(Eigen::Matrix<float,nos,1> mu_bar, Eigen::Matrix<float,nos,1> mu_t_1, Eigen::Matrix<float,nom,1> z_t_1);

		Eigen::Matrix<float,nom,nos> hJac(Eigen::Matrix<float,nos,1> mu_bar, Eigen::Matrix<float,nos,1> mu_t_1);
		
		void estimate(amee::Velocity controlSignal, amee::Pose measurement1, amee::Pose measuremnent2);
		void setMeasurement1(amee::Pose measurement1);
		void setMeasurement2(amee::Pose measurement2);

		amee::Pose getMu();
		amee::Pose getSigma();
		amee::Pose getPose();
		int getNos();
		int getNom();
		int getNoc();

		amee::Pose mPose;	
		amee::Pose mMeasurement1;
		amee::Pose mMeasurement2;

		// used in motion model (g-function)
		double lastTime;
		Eigen::Matrix<float,nos,1> movement;	

	};
}
#endif