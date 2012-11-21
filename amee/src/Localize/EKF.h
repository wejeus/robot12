#ifndef EKF_H
#define EKF_H

#include <std_msgs/Int32.h>
#include "amee/Velocity.h"
#include "amee/Odometry.h"
#include "amee/Pose.h"
#include "Eigen/Eigen"
#include "roboard_drivers/Encoder.h"

using namespace Eigen;

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
		Matrix<float,nos,1> mu;
		Matrix<float,nos,1> mu_t_1;
		Matrix<float,nos,1> mu_bar;
		//Matrix<float,nos,1> mu_bar_t_1;

		Matrix<float,nos,nos> sigma;
		Matrix<float,nos,nos> sigma_bar;
		Matrix<float,nos,nos> sigma_t_1;

		Matrix<float,noc,1> u;
		Matrix<float,noc,1> u_t_1;

		Matrix<float,nom,1> z;
		Matrix<float,nom,1> z_hat;
		Matrix<float,nom,1> z_t_1;
		
		Matrix<float,nos,nos> G; // jacobian of g (motion model)
		Matrix<float,nom,nos> H; // Jacobian of h ()

		Matrix<float,nos,nom> K; // Kalman gain

		Matrix<float,nos,nos> R; // Process noice
		Matrix<float,nom,nom> Q; // Measurement noice

		Matrix<float,nos,nos> I;


		void init();

		void setStartPose(Matrix<float,nos,1> mu, Matrix<float,nos,nos> sigma);

		void setR(Matrix<float,nos,1> processNoise);
		void setQ(Matrix<float,nom,1> measurementNoise);

		void setG(Matrix<float,nos,nos> gJacobian);
		void setH(Matrix<float,nom,nos> hJacobian);

		Matrix<float,nos,1> g(Matrix<float,noc,1> u, Matrix<float,nos,1> mu_t_1);
		Matrix<float,nom,1> h(Matrix<float,nos,1> mu_bar, Matrix<float,nos,1> mu_t_1, Matrix<float,nom,1> z_t_1);

		Matrix<float,nom,nos> hJac(Matrix<float,nos,1> mu_bar, Matrix<float,nos,1> mu_t_1);
		
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
	};
}
#endif