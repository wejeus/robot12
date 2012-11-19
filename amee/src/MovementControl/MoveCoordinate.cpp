#include <iostream>
#include "MoveCoordinate.h"
#include <cmath>
#include "amee/Velocity.h"

using namespace amee;

MoveCoordinate::MoveCoordinate(ros::Publisher& pub) {
	mSpeedPub = pub;
	mIsRotating = false;
}

MoveCoordinate::~MoveCoordinate() {
}

void MoveCoordinate::init(const SensorData& data) {
	// New rotation requested, init rotation procedure

	mCurrentRelativeAngle = 0.0f;
	mStartingAngle = data.odometry.angle;
	mIsRotating = true;
	mTargetAngle = 90.0f;
	publishSpeeds(0.0f, 0.0f);
}

void MoveCoordinate::init(const SensorData& data, const float& degrees) {
	init(data);
	mTargetAngle = degrees;
}

bool MoveCoordinate::isRunning() const {
	return mIsRotating;
}

void MoveCoordinate::doControl(const SensorData& data) {
/* from python code
	ref_vec = (1.0, 0.0) # Reference vector, set to coordinate axis in direction of robot
        point = (x, y)
        distance = norm(point)
        angle = math.acos(dot_product(ref_vec, point) / (norm(ref_vec)*norm(point)))
        self.move_rotate(angle * (180/math.pi))
        self.move_straight(distance)
*/

	float MAX_ROTATION_SPEED = 0.1;
	float MIN_ROTATION_SPEED = 0.06;

	mCurrentRelativeAngle = data.odometry.angle - mStartingAngle;
	// std::cout << mCurrentRelativeAngle << std::endl;
	// std::cout << data.odometry << std::endl;
	// std::cout << "target=" << mTargetAngle << std::endl;
	if (fabs(mCurrentRelativeAngle - mTargetAngle) > 0.8) {
		float K_p = 1.0/200.0; // starts to slow down 20 degrees before final angle
		float angleError = mTargetAngle - mCurrentRelativeAngle;

		// lower speed as we come closer to "degreesToTravel"
		float rotationSpeed = K_p * angleError;
		float speedSign = rotationSpeed > 0.0f ? 1.0 : -1.0f;

		if (fabs(rotationSpeed) > MAX_ROTATION_SPEED) {
			// saturate speed to ROTATION_SPEED if too high
			rotationSpeed = speedSign * MAX_ROTATION_SPEED;
		} else {
			// saturate speed to ROTATION_SPEED if too low
			rotationSpeed = speedSign * MIN_ROTATION_SPEED;
		}

		publishSpeeds(-rotationSpeed, rotationSpeed);
		
	} else {
		// Rotation done! Reset current angle movement
		// std::cout << "TARGET ANGLE REACHED" << std::endl;
		mIsRotating = false;
		publishSpeeds(0.0f, 0.0f);
	}
}

void MoveCoordinate::publishSpeeds(float left, float right) {
	Velocity v;
	v.left = left;
	v.right = right;
	mSpeedPub.publish(v);
}
