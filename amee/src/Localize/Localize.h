#ifndef LOCALIZE_H
#define LOCALIZE_H

#include <std_msgs/Int32.h>
#include "amee/Velocity.h"
#include "amee/Odometry.h"
#include "amee/Pose.h"

class Loclasize {

#define X 0.0f

private:

	amee::Velocity mVelocity;
	amee::Odometry mOdometry;
	amee::Pose mPose;
 
 	ros::Subscriber enc_sub;
	ros::Subscriber motorSpeed_sub
	ros::Publisher pose_pub;
	
	public:
		void receiveEncoder(const roboard_drivers::Encoder::ConstPtr &msg);
		void receiveMotorSpeed(const amee::Velocity::ConstPtr &v);
		void receiveControlSignal(const amee::Pose::ConstPtr &v);
		void publishPose(float x, float y, float theta, float x_var, float y_var, float theta_var);
		void init();

	protected:
		// what is protected?
	private:
		// why two private?		
		void publishPose();
};
#endif
