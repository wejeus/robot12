#ifndef LOCALIZE_H
#define LOCALIZE_H

#include <std_msgs/Int32.h>
#include "amee/Velocity.h"
#include "amee/Odometry.h"
#include "amee/FollowWallStates.h"
#include "amee/Pose.h"
#include "roboard_drivers/Encoder.h"
#include "roboard_drivers/Motor.h"

class Localize {

#define X 0.0f

private:

public:

	amee::Odometry mOdometry;
	amee::Odometry mLastOdometry;
	amee::Pose mPose; // set to zero as default
	amee::FollowWallStates followWallState;

	void init();

	void receiveOdometry(const amee::Odometry::ConstPtr &msg);
	void receiveFollowWallState(const amee::FollowWallStates::ConstPtr &msg);
	void publishPose(ros::Publisher pose_pub);

protected:
	
		
};
#endif