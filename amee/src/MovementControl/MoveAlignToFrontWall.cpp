#include <iostream>
#include "MoveAlignToFrontWall.h"
#include <cmath>
#include "amee/Velocity.h"
#include "MoveAlignWall.h"

using namespace amee;

MoveAlignToFrontWall::MoveAlignToFrontWall(ros::Publisher& pub) {
    mSpeedPub = pub;
    mIsRunning = false;
    mRefDistReached = false;
    mWallAligner = new MoveAlignWall(pub);
}

MoveAlignToFrontWall::~MoveAlignToFrontWall() {
    delete mWallAligner;
}

void MoveAlignToFrontWall::init(const SensorData& data) {
    init(data, 0.025f);
}

void MoveAlignToFrontWall::init(const SensorData& data, float refDist) {
    // New align requested, init align procedure
    mIsRunning = true;
    mRefDist = refDist;
    mRefDistReached = fabs((data.irdistances.wheelLeft + data.irdistances.wheelRight) / 2.0f - refDist) <= 0.008f;
    mWallAligner->init(data, ALIGN_FRONT_WALL);
    publishSpeeds(0.0f, 0.0f);
}

bool MoveAlignToFrontWall::isRunning() const {
    return mIsRunning;
}

void MoveAlignToFrontWall::doControl(const SensorData& data) {
    float MAX_SPEED = 0.1;
    float MIN_SPEED = 0.08;

    if (mRefDistReached) {
        if (mWallAligner->isRunning()) {
            mWallAligner->doControl(data);
        } else {
            mIsRunning = false;
        }
    } else {
        // float angle_to_wall = tan((mSensorData.irdistances.rightBack - mSensorData.irdistances.rightFront) / IR_BASE_RIGHT);
        // float distance_to_wall = cos(angle_to_wall) * ir_right_mean;
        float error = (data.irdistances.wheelLeft + data.irdistances.wheelRight) / 2.0f - mRefDist;

        if (fabs(error) > 0.004) {
            float K_p = 1.0/2.0; // starts to slow down before final angle
            
            // lower speed as we come closer to 0 error
            float speed = K_p * error;
            float sign = speed > 0.0f ? 1.0 : -1.0f;

            if (fabs(speed) > MAX_SPEED) {
                speed = sign * MAX_SPEED;
            } else if (fabs(speed) < MIN_SPEED) {
                speed = sign * MIN_SPEED;
            }

            publishSpeeds(speed, speed);
            
        } else {
            mRefDistReached = true;
            publishSpeeds(0.0f, 0.0f);
        }    
    }    
}

void MoveAlignToFrontWall::publishSpeeds(float left, float right) {
    Velocity v;
    v.left = left;
    v.right = right;
    mSpeedPub.publish(v);
}
