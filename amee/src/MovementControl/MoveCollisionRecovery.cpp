#include <iostream>
#include "MoveCollisionRecovery.h"
#include <cmath>
#include "amee/Velocity.h"
#include "MoveAlignWall.h"
#include "MovementControl.h"
#include "MoveStraight.h"
#include "MoveRotate.h"
#include "amee/MovementEvent.h"

using namespace amee;

MoveCollisionRecovery::MoveCollisionRecovery(ros::Publisher& pub, ros::Publisher& eventPub) {
    mSpeedPub = pub;
    mEventPub = eventPub;
    mIsRunning = false;
    mWallAligner = new MoveAlignWall(pub);
    mStraightMover = new MoveStraight(pub);
    mRotater = new MoveRotate(pub);
}

MoveCollisionRecovery::~MoveCollisionRecovery() {
    delete mWallAligner;
    delete mStraightMover;
    delete mRotater;
}

void MoveCollisionRecovery::init(const SensorData& data) {
    // New recovery requested, init recovery procedure
    mIsRunning = true;
    mRotaterInitialized = false;
    mAlignerInitialized = false;

    mStraightMover->init(data, -1.0f * BACKWARDS_DIST, 0.09f);

    publishSpeeds(0.0f, 0.0f);
}

bool MoveCollisionRecovery::isRunning() const {
    return mIsRunning;
}

bool MoveCollisionRecovery::collisionAhead(const SensorData& data) {
    return (data.irdistances.wheelRight <= 0.08f && data.irdistances.wheelRight >= -0.04f)
        || (data.irdistances.wheelLeft <= 0.08f && data.irdistances.wheelLeft >= -0.04f)
        || (data.sonarDistance <= 0.1f);
}

void MoveCollisionRecovery::doControl(const SensorData& data) {
    
    if (mStraightMover->isRunning()) { // first move back
        // std::cout << "Moving back!" << std::endl;
        mStraightMover->doControl(data);
    } else {
        // std::cout << "Moving back done!" << std::endl;
        if (collisionAhead(data)) { // as long as there is a wall in front rotate left (max 180 degrees)
            // std::cout << "collision ahead, so rotate!" << std::endl;
            if (!mRotaterInitialized) {
                // std::cout << "Rotate init!" << std::endl;
                mRotater->init(data, 1.0f * MAX_ANGLE); // 1.0f because of linker c++ stuff
                mRotaterInitialized = true;
            }
            if (mRotater->isRunning()) {
                // std::cout << "rotating!" << std::endl;
                mRotater->doControl(data);
                return;
            }
        } else { // if no collision ahead, try to align to wall on the right
            // std::cout << "No collision ahead!" << std::endl;
            if (!mAlignerInitialized && wallOnRight(data)) { // init aligner
                // std::cout << "init Aligner!" << std::endl;
                mWallAligner->init(data,ALIGN_RIGHT_WALL);
                mAlignerInitialized = true;
            } else if (mAlignerInitialized && mWallAligner->isRunning()) { // do aligning
                // std::cout << "do aligning!" << std::endl;
                mWallAligner->doControl(data);
            } else { // we are done
                // std::cout << "collision recovery done!" << std::endl;
                mIsRunning = false;
                publishSpeeds(0.0f, 0.0f);
                MovementEvent me;
                me.type = MovementControl::MOVEMENT_EVENT_TYPE_RECOVERY_DONE;
                mEventPub.publish(me);
            }
        }
    }
    
}

bool MoveCollisionRecovery::wallOnRight(const SensorData& data) {
    bool distFrontOk = data.irdistances.rightFront >= 0.0f && data.irdistances.rightFront <= 0.14f;
    bool distBackOk = data.irdistances.rightBack >= 0.0f && data.irdistances.rightBack <= 0.14f;
    // bool noEdge = fabs(data.irdistances.rightFront - data.irdistances.rightBack) <= 0.06f;
    return distBackOk && distFrontOk;// && noEdge;
}


void MoveCollisionRecovery::publishSpeeds(float left, float right) {
    Velocity v;
    v.left = left;
    v.right = right;
    mSpeedPub.publish(v);
}
