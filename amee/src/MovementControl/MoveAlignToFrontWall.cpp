#include <iostream>
#include "MoveAlignToFrontWall.h"
#include <cmath>
#include "amee/Velocity.h"
#include "MoveAlignWall.h"
#include "MovementControl.h"
#include "MoveStraight.h"

using namespace amee;

MoveAlignToFrontWall::MoveAlignToFrontWall(ros::Publisher& pub) {
    mSpeedPub = pub;
    mIsRunning = false;
    mRefDistReached = false;
    mCanAlign = false;
    mWallAligner = new MoveAlignWall(pub);
    mStraightMover = new MoveStraight(pub);
}

MoveAlignToFrontWall::~MoveAlignToFrontWall() {
    delete mWallAligner;
    delete mStraightMover;
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
    mStraightMover->init(data, -0.01f, 0.09f);
    mCanAlign = frontAlignmentPossible(data);

    publishSpeeds(0.0f, 0.0f);
}

bool MoveAlignToFrontWall::isRunning() const {
    return mIsRunning;
}

bool MoveAlignToFrontWall::frontAlignmentPossible(const SensorData& data) {
    std::cout << data.irdistances.wheelRight << " " << data.irdistances.wheelLeft << std::endl;
    bool rightOk = data.irdistances.wheelRight >= -0.03f && data.irdistances.wheelRight <= 0.12;
    bool leftOk = data.irdistances.wheelLeft >= -0.03f && data.irdistances.wheelLeft <= 0.12f;
    return leftOk && rightOk;
}

void MoveAlignToFrontWall::doControl(const SensorData& data) {
    float MAX_SPEED = 0.1;
    float MIN_SPEED = 0.08;
    std::cout << "Can align: " << mCanAlign << std::endl;
    if(mCanAlign) {
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
    } else {
        // PLACE AMEE IN A GOOD DISTANCE TO AN EVIL WALL WITHOUT COLLIDING AT THE TAIL
        if (mStraightMover->isRunning() && MovementControl::wallInFront(data)) {
            std::cout << "move backwards!" << std::endl;
            mStraightMover->doControl(data);
        } else {
            mIsRunning = false;
            publishSpeeds(0.0f,0.0f);
        }
    }
    
}

void MoveAlignToFrontWall::publishSpeeds(float left, float right) {
    Velocity v;
    v.left = left;
    v.right = right;
    mSpeedPub.publish(v);
}
