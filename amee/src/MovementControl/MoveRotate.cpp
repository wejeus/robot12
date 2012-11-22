#include <iostream>
#include "MoveRotate.h"
#include <cmath>
#include "amee/Velocity.h"

using namespace amee;

MoveRotate::MoveRotate(ros::Publisher& pub) {
    mSpeedPub = pub;
    mIsRotating = false;
}

MoveRotate::~MoveRotate() {
}

void MoveRotate::init(const SensorData& data) {
    // New rotation requested, init rotation procedure
    mCurrentRelativeAngle = 0.0f;
    mStartingAngle = data.odometry.angle;
    mIsRotating = true;
    mTargetAngle = 90.0f;
    publishSpeeds(0.0f, 0.0f);
}

void MoveRotate::init(const SensorData& data, const float& degrees) {
    init(data);
    mTargetAngle = degrees;
}

bool MoveRotate::isRunning() const {
    return mIsRotating;
}

bool startedRotation = false;
void MoveRotate::doControl(const SensorData& data) {
    float MAX_ROTATION_SPEED = 0.2;
    float MIN_ROTATION_SPEED = 0.06;

    mCurrentRelativeAngle = data.odometry.angle - mStartingAngle;
    std::cout << "relative angle" << mCurrentRelativeAngle << std::endl;
    std::cout << "odometry angle" << data.odometry.angle << std::endl;
    std::cout << "target=" << mTargetAngle << std::endl;
    if (fabs(mCurrentRelativeAngle - mTargetAngle) > 0.2) {
        float K_p = 1.0/200.0; // starts to slow down 20 degrees before final angle
        float angleError = mTargetAngle - mCurrentRelativeAngle;

        // lower speed as we come closer to "degreesToTravel"
        float rotationSpeed = K_p * angleError;
        float speedSign = rotationSpeed > 0.0f ? 1.0 : -1.0f;
        if ((fabs(mCurrentRelativeAngle) < 300.0f) && !startedRotation) {
            rotationSpeed = speedSign * (mCurrentRelativeAngle + 1.0f) * MIN_ROTATION_SPEED; // make a smooth acceleration
            std::cout << "ACC LIMITER" << std::endl;
        } else {
            std::cout << "NO ACC LIMITER" << std::endl;
            startedRotation = true;
        }


        if (fabs(rotationSpeed) > MAX_ROTATION_SPEED) {
            // saturate speed to ROTATION_SPEED if too high
            rotationSpeed = speedSign * MAX_ROTATION_SPEED;
        } else if (fabs(rotationSpeed) < MIN_ROTATION_SPEED) {
            // saturate speed to ROTATION_SPEED if too low
            rotationSpeed = speedSign * MIN_ROTATION_SPEED;
        }

        publishSpeeds(-rotationSpeed, rotationSpeed);
        
    } else {
        // Rotation done! Reset current angle movement
        std::cout << "TARGET ANGLE REACHED" << std::endl;
        std::cout << "ROTATED: " << mCurrentRelativeAngle << std::endl;
        mIsRotating = false;
        publishSpeeds(0.0f, 0.0f);
    }
}

void MoveRotate::publishSpeeds(float left, float right) {
    Velocity v;
    v.left = left;
    v.right = right;
    mSpeedPub.publish(v);
}
