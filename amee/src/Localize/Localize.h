#ifndef LOCALIZE_H
#define LOCALIZE_H

#include <std_msgs/Int32.h>
#include "amee/Velocity.h"
#include "amee/Odometry.h"
#include "amee/Pose.h"
#include <Eigen>

class Loclasize {

#define X 0.0f

private:

	amee::Velocity speed;
	amee::Odometry encoder;
	amee::Odometry lastEncoder;
	amee::Velocity controlSignal;
	amee::Pose pose;
 	amee::Pose lastPose;

 	ros::Subscriber enc_sub;
	ros::Subscriber motorSpeed_sub
	ros::Publisher pose_pub;

	public:
		void receiveEncoder(const roboard_drivers::Encoder::ConstPtr &msg);
		void receiveMotorSpeed(const amee::Velocity::ConstPtr &v);
		void receiveControlSignal(const amee::Velocity::ConstPtr &v);
		void publishPose(float x, float y, float theta, float x_var, float y_var, float theta_var);
		void init();

	protected:
		// what is protected?
	private:
		// why two private?		
		void publishPose();
};

class EKF {

	Vector3f mu;
	Vector3f mu_bar;
	Vector3f mu_t_1;

	Matrix3f sigma;
	Matrix3f sigma_bar;
	Matrix3f sigma_t_1;
	
	Vector2f u;
	Vector2f u_t_1;	
	
	Vector2f z;
	Vector2f z_t_1;
	Vector2f z_hat;

	Matrix3f::Identity G; // jacobian of g (motion model)
	Matrix3f::Identity H; // Jacobian of h ()

	float K; // Kalman gain

	Matrix2f R; // Measurement noice
	Matrix2f Q; // Process noice

	amee::Pose pose;
	amee::Pose poseVariance;
	
	private:

		public:
				void init();

				void g(u, mu_t_1); // calc expected mean
				void calcExpCov(G, sigma_t_1, R);
				void calcK(sigma_bar, H, Q);
				void h(mu_bar);
				void calcMean(mu_bar, K, H, sigma_bar);
				void calcCov(K, H, sigma_bar);

				void setR(float processNoise);
				void setQ(float measurementNoise);

				amee::Pose getMu();
				amee::Pose getSigma();

};
#endif
