#include <iostream>
#include "MoveAlignWall.h"
#include <cmath>
#include "amee/Velocity.h"

using namespace amee;

MoveAlignWall::MoveAlignWall(ros::Publisher& pub) {
    mSpeedPub = pub;
    mIsRunning = false;
}

MoveAlignWall::~MoveAlignWall() {
}

void MoveAlignWall::init(const SensorData& data) {
    // New align requested, init align procedure
    mWallSide = ALIGN_RIGHT_WALL;
    mIsRunning = true;
    mStartingAngle = data.odometry.angle;
    publishSpeeds(0.0f, 0.0f);
}

void MoveAlignWall::init(const SensorData& data, const int& side) {
    init(data);
    mWallSide = side;
}

bool MoveAlignWall::isRunning() const {
    return mIsRunning;
}

void MoveAlignWall::doControl(const SensorData& data) {
    float MAX_ROTATION_SPEED = 0.1;
    float MIN_ROTATION_SPEED = 0.06;

    float frontDist = data.irdistances.rightFront;
    float backDist = data.irdistances.rightBack;

    float wallSign = 1.0f;

    if (mWallSide == ALIGN_LEFT_WALL) {
        frontDist = data.irdistances.leftFront;
        backDist = data.irdistances.leftBack;
        wallSign = -1.0f;
    }

    if (mWallSide == ALIGN_FRONT_WALL) {
        frontDist = data.irdistances.wheelLeft;
        backDist = data.irdistances.wheelRight;
    }

    if (fabs(mStartingAngle - data.odometry.angle) > 180.0f) { 
        // make sure we don't end up rotating all the time when there is something wrong
        // std::cout << "There seems to be no wall. Stop!" << std::endl;
        stop();
    }

    // std::cout << mCurrentRelativeAngle << std::endl;
    // std::cout << data.odometry << std::endl;
    // std::cout << "target=" << mTargetAngle << std::endl;
    if (fabs(frontDist - backDist) > 0.004) {
        float K_p = 1.0/2.0; // starts to slow down before final angle
        float error = backDist - frontDist;

        // lower speed as we come closer to 0 error
        float rotationSpeed = K_p * error;
        float speedSign = rotationSpeed > 0.0f ? 1.0 : -1.0f;

        if (fabs(rotationSpeed) > MAX_ROTATION_SPEED) {
            // saturate speed to ROTATION_SPEED if too high
            rotationSpeed = speedSign * MAX_ROTATION_SPEED;
        } else if (fabs(rotationSpeed) < MIN_ROTATION_SPEED) {
            // saturate speed to ROTATION_SPEED if too low
            rotationSpeed = speedSign * MIN_ROTATION_SPEED;
        }

        publishSpeeds(-wallSign * rotationSpeed, wallSign * rotationSpeed);
        
    } else {
        // Aligning done!
        // std::cout << "TARGET ANGLE REACHED" << std::endl;
        stop();
    }
}

void MoveAlignWall::stop() {
    mIsRunning = false;
    publishSpeeds(0.0f, 0.0f);
}

void MoveAlignWall::publishSpeeds(float left, float right) {
    Velocity v;
    v.left = left;
    v.right = right;
    mSpeedPub.publish(v);
}
