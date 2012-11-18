#include "MoveFollowWall.h"
#include "amee/Velocity.h"
#include "MoveRotate.h"
#include "MoveStraight.h"
#include "MoveAlignWall.h"
#include "MoveAlignToFrontWall.h"
#include "amee/FollowWallStates.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace amee;

    MoveFollowWall::MoveFollowWall(ros::Publisher& pub, ros::Publisher& statesPub) {
        mVelPub = pub;
        mFollowWallStatesPub = statesPub;

        mRotater = new MoveRotate(pub);
        mStraightMove = new MoveStraight(pub);
        mWallAligner = new MoveAlignWall(pub);
        mFrontWallAligner = new MoveAlignToFrontWall(pub);
    }

    MoveFollowWall::~MoveFollowWall() {
        delete mRotater;
        delete mStraightMove;
        delete mWallAligner;
        delete mFrontWallAligner;
    }

    void MoveFollowWall::init(const SensorData& data) {
        
        error_sum = 0.0f;
        last_error = 0.0f;

        mRunning = true;
        mState.set(FollowWall);

        ros::Publisher pub_follewWallStates;
    }

    void MoveFollowWall::publishSpeeds(float left, float right) {
        Velocity v;
        v.left = left;
        v.right = right;
        mVelPub.publish(v);
    }

    // enum WallFollowState {foundFrontWall, followSideWall, doNothing, rotating, movingStraight};
    // struct SensorData {
    //     Odometry odometry;
    //     IRDistances irdistances;
    // };
    void MoveFollowWall::doControl(const SensorData& data) {

        mSensorData = data;

        switch (mState.mState) {
            case FollowWall:
               followWallState();
                break;
            case LookForEndOfWall:
                lookForEndOfWallState();
                break;
            case MoveTail:
                moveTailState();
                break;
            case RotateRight:
                rotateRightState();
                break;
            case RotateLeft:
                rotateLeftState();
                break;
            case LookForBeginningOfWall:
                lookForBeginningOfWallState();
                break;
            case AlignToWall:
                alignToWallState();
                break;
            case HandleEvilWalls:
                handleEvilWallsState();
                break;
            case AlignToFrontWall:
                alignToFrontWallState();
                break;
            case TIntersectionHandling:
                tIntersectionHandlingState();
                break;
            case EdgeOfWall:
                edgeOfWallState();
                break;
            default: std::cout << "UNKNOWN STATE" << std::endl;
        }
    }

    void MoveFollowWall::tIntersectionHandlingState() {
              
        if (!mState.initialized) {
            std::cout << "T Intersection: init" << std::endl;  
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;

            if (frontAlignmentPossible()) { 
                std::cout << "T Intersection: ALIGNMENT POSSIBLE" << std::endl;  
                mFrontWallAligner->init(mSensorData, MIN_WALL_DISTANCE);    
            } else {
                std::cout << "T Intersection: CAN'T ALIGN -> ROTATE RIGHT NOW" << std::endl;  
                mState.set(RotateRight);
                MoveFollowWall::publishState(T_INTERSECTION_HANDLED);
                return;
            }
        }

        // if we see a wall reset the gapDistance (because there is no gap!!)
        if (seesWall(mSensorData.irdistances.rightFront) || seesWall(mSensorData.irdistances.rightBack)) {
            mState.gapStartDist = mSensorData.odometry.distance;
        }

        // if we have seen a big enough gap on the right side, we don't need to align to the front wall anymore
        if (mSensorData.odometry.distance - mState.gapStartDist >= TAIL_LENGTH) {
            mState.set(RotateRight);
            MoveFollowWall::publishState(T_INTERSECTION_HANDLED);
            return;
        }

        if (mFrontWallAligner->isRunning()) {
            mFrontWallAligner->doControl(mSensorData);
        } else {
            mState.set(RotateRight);
            MoveFollowWall::publishState(T_INTERSECTION_HANDLED);
        }
    }

    // only go this state if you're sure there is a wall it can align to. If there is no wall it will dance or rotate at most 180 degrees, but the 
    // robot will be lost, so make sure there is a wall before going into this state!
    // After aligning it changes to FollowWall.
    void MoveFollowWall::alignToWallState() {
        
        if (!mState.initialized) {
            std::cout << "AlignToWall init" << std::endl;
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            mWallAligner->init(mSensorData);
        }

        if (mWallAligner->isRunning()) {
            mWallAligner->doControl(mSensorData);
        } else {
            
            MoveFollowWall::publishState(ALIGNED_TO_WALL);
            mState.set(FollowWall);
        }

    }

    void MoveFollowWall::alignToFrontWallState() {
        
        if (!mState.initialized) {
            std::cout << "AlignToFrontWall init" << std::endl;
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            // TODO WE HAVE TO DISTINGUISH WHETHER WE ARE IN FRONT OF A EVIL WALL OR NOT
            // DO THIS BY CHECKING THE DISTANCES MEASURED BY THE IRS UNDERNEATH AMEE
            if (frontAlignmentPossible()) { 
                mFrontWallAligner->init(mSensorData, MIN_WALL_DISTANCE);    
            } else {
                mState.set(RotateLeft);
                return;
            }
        }

        if (mFrontWallAligner->isRunning()) {
            mFrontWallAligner->doControl(mSensorData);
        } else {
            mState.set(RotateLeft);
            MoveFollowWall::publishState(ALIGNED_TO_FRONT_WALL);
        }
    }

    void MoveFollowWall::lookForBeginningOfWallState() {
        
        if (!mState.initialized) {
            std::cout << "LookForBeginningOfWall init" << std::endl;
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            mFoundWallRightBack = false;
            mFoundWallRightFront = false;
            mStraightMove->init(SLOW_MOVEMENT_SPEED);
        }

        if (wallInFront()) { // in case there is for whatever evil reason a wall in front
            mState.set(AlignToFrontWall);
            return;
        }
        bool rF = seesWall(mSensorData.irdistances.rightFront);
        bool rB = seesWall(mSensorData.irdistances.rightBack);
        mFoundWallRightFront = mFoundWallRightFront || rF;
        mFoundWallRightBack = mFoundWallRightBack || rB;

        if (nextToWall()) {
            mState.set(AlignToWall);
            MoveFollowWall::publishState(FOUND_BEGINNING_OF_WALL);
            return;
        }

        if (mFoundWallRightBack && mFoundWallRightFront) {
            // both sensors have seen the wall, but it's not continues, so go to evil wall handling
            mState.set(HandleEvilWalls);
            mState.handleWallInitDistance = TAIL_LENGTH; // make sure it moves far enough for a u-turn
        } else {
            mStraightMove->doControl(mSensorData);
        }
    }

    void MoveFollowWall::rotateLeftState() {
        
          if (!mState.initialized) {
            std::cout << "RotateLeft init" << std::endl;
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            mRotater->init(mSensorData, 90.0f);
          }
          if (mRotater->isRunning()) { 
            mRotater->doControl(mSensorData);
          } else {
            if (wallInFront()){
                mState.set(AlignToFrontWall);
            } else {
                mState.set(HandleEvilWalls);
            }
            publishState(ROTATED_LEFT);
          }
    }

    void MoveFollowWall::rotateRightState() {
        
          if (!mState.initialized) {
            std::cout << "RotateRight init" << std::endl;
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            mRotater->init(mSensorData, -90.0f);
          }
          if (mRotater->isRunning()) { 
            mRotater->doControl(mSensorData);
          } else {
            mState.set(LookForBeginningOfWall);   
            MoveFollowWall::publishState(ROTATED_RIGHT);
          }
    }

    void MoveFollowWall::moveTailState() {
        
        if (wallInFront()) {
            mState.set(TIntersectionHandling);
        } else { 
            if (!mState.initialized) {
                std::cout << "MoveTail init" << std::endl;
                // publishSpeeds(0.0f,0.0f);
                mState.gapStartDist = mSensorData.odometry.distance; // to measure the size of the seen gap
                mState.initialized = true;
                mStraightMove->init(mSensorData, TAIL_LENGTH,SLOW_MOVEMENT_SPEED);
            }
            if (mStraightMove->isRunning()) {
                mStraightMove->doControl(mSensorData);
            } else {
                MoveFollowWall::publishState(MOVED_TAIL);
                mState.set(RotateRight);   
            }
        }
    }

    void MoveFollowWall::lookForEndOfWallState() {
              
        if (!mState.initialized) {
            std::cout << "LookForEndOfWall init" << std::endl;
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            mStraightMove->init(SLOW_MOVEMENT_SPEED);
        }

        if (wallInFront()) {
            // publishSpeeds(0.0f,0.0f);
            mState.set(TIntersectionHandling);
            return;
        }

        if (seesWall(mSensorData.irdistances.rightFront)) {
            mState.set(LookForBeginningOfWall); 
        } else if (!seesWall(mSensorData.irdistances.rightFront) 
            && !seesWall(mSensorData.irdistances.rightBack)) {
            MoveFollowWall::publishState(FOUND_END_OF_WALL);
            mState.set(MoveTail);
        } else {
            mStraightMove->doControl(mSensorData);
        }
    }

    void MoveFollowWall::edgeOfWallState() {
        
        if (!mState.initialized) {
            std::cout << "edgeOfWallState init" << std::endl;
            mStraightMove->init(SLOW_MOVEMENT_SPEED);
            mState.initialized = true;
        }

        if(wallInFront()) {
            mState.set(TIntersectionHandling);
            publishSpeeds(0.0f,0.0f);
        }

        float error = fabs(mSensorData.irdistances.rightBack - mSensorData.irdistances.rightFront);

        if (error <= IR_ERROR_THRESHOLD) {
            mState.set(FollowWall);
        } else if (!seesWall(mSensorData.irdistances.rightFront)) {
            mState.set(LookForEndOfWall);
        } else {
            mStraightMove->doControl(mSensorData);
        }
    }

    void MoveFollowWall::followWallState() {
        if (!mState.initialized) {
            std::cout << "FollowWall init" << std::endl;    
            mState.initialized = true;
        }
        
        // std::cout << mSensorData.irdistances << std::endl;
        if (wallInFront()) {
            mState.set(AlignToFrontWall);
            publishState(FOLLOWED_WALL);
            return;
        }

        if (seesWall(mSensorData.irdistances.rightBack) && seesWall(mSensorData.irdistances.rightFront)) {
            
            float error = fabs(mSensorData.irdistances.rightBack - mSensorData.irdistances.rightFront);
            if (error > IR_ERROR_THRESHOLD) {
                mState.set(EdgeOfWall);
            } else {
                followWall();    
            } 

         } else if (!seesWall(mSensorData.irdistances.rightFront) && seesWall(mSensorData.irdistances.rightBack)) {
            publishState(FOLLOWED_WALL);
            mState.set(LookForEndOfWall);
         } else if (seesWall(mSensorData.irdistances.rightFront) && !seesWall(mSensorData.irdistances.rightBack)) {
            mState.set(LookForBeginningOfWall);
         } else {
           mState.set(HandleEvilWalls);
        }
    }

    void MoveFollowWall::handleEvilWallsState() {
        if (!mState.initialized) {
            std::cout << "handleEvilWallsState init" << std::endl;
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            // initialize this value so that nextToWall() can work
            mSeenWallStartDist = mSensorData.odometry.distance;
            mStraightMove->init(mSensorData, mState.handleWallInitDistance);
        }
        // stop if wall in front
        if (wallInFront()) {
            if (seesWall(mSensorData.irdistances.rightFront)) {
                mState.set(AlignToFrontWall);
            } else {
                mState.set(TIntersectionHandling);    
            }
            return;
        }

        // continue wall following if we see a wall
        if (nextToWall()) {
            mState.set(AlignToWall);
            publishSpeeds(0.0f,0.0f);
            return;
        }

        // if rightFront sees a wall, we have to travel further!
        if (seesWall(mSensorData.irdistances.rightFront)) {
            mStraightMove->init(mSensorData, IR_BASE_RIGHT + TAIL_LENGTH);
        }

        if (mStraightMove->isRunning()) {
            mStraightMove->doControl(mSensorData);
        } else {
            if (seesWall(mSensorData.irdistances.rightBack)) {
                mStraightMove->init(mSensorData, TAIL_LENGTH);
            } else {
                mState.set(RotateRight);    
            }
        }
    }

    void MoveFollowWall::publishState(int state) { // publishes the state

        amee::FollowWallStates followWallState;
        followWallState.state = state;
        struct timeval time;
        gettimeofday(&time, NULL);
        followWallState.timestamp = time.tv_sec+double(time.tv_usec)/1000000.0;          

        mFollowWallStatesPub.publish(followWallState);

    }

    bool MoveFollowWall::isRunning() const {
        return mRunning;
    }

    bool MoveFollowWall::seesWall(float distance) {
        //TODO set as constants
        return distance >= 0.0f && distance <= 0.14f;
    }

    bool MoveFollowWall::nextToWall() {
        bool rF = seesWall(mSensorData.irdistances.rightFront);
        bool rB = seesWall(mSensorData.irdistances.rightBack);
        if (!rF) {
            mSeenWallStartDist = mSensorData.odometry.distance;
        }
        float wallLength = mSensorData.odometry.distance - mSeenWallStartDist;
        return rB && (wallLength >= IR_BASE_RIGHT);
    }

    bool MoveFollowWall::frontAlignmentPossible() {
        bool rightOk = mSensorData.irdistances.wheelRight >= -0.03f && mSensorData.irdistances.wheelRight <= 0.12;
        bool leftOk = mSensorData.irdistances.wheelLeft >= -0.03f && mSensorData.irdistances.wheelLeft <= 0.12f;
        return leftOk && rightOk;
    }

    bool MoveFollowWall::wallInFront() {
        // TODO use constants
        // return (mSensorData.irdistances.frontShortRange <= 0.11f && mSensorData.irdistances.frontShortRange >= 0.0f)
        return mSensorData.irdistances.obstacleInFront 
            || (mSensorData.irdistances.wheelRight <= 0.05f && mSensorData.irdistances.wheelRight >= -0.03f)
            || (mSensorData.irdistances.wheelLeft <= 0.05f && mSensorData.irdistances.wheelLeft >= -0.03f)
            || (mSensorData.sonarDistance <= 0.13f);
    }


    void MoveFollowWall::followWall() {
           
        float ir_right_mean = (mSensorData.irdistances.rightBack + mSensorData.irdistances.rightFront)/2.0f;
        float rotationSpeed = 0.0f;
        float error = mSensorData.irdistances.rightBack - mSensorData.irdistances.rightFront;
        error_sum += error;
        rotationSpeed = K_p_keepRef * error + K_i_keepRef * error_sum; 
        
        float sign = error_sum >= 0 ? 1.0f : -1.0f;
        error_sum = fabs(error_sum) > MAX_ERROR_SUM ? sign * MAX_ERROR_SUM : error_sum;
        last_error = error;
        

        if ((mSensorData.irdistances.leftFront <= LEFT_WALL_TOO_CLOSE) && (mSensorData.irdistances.leftBack <= LEFT_WALL_TOO_CLOSE)) {
            
            float ir_left_mean = (mSensorData.irdistances.leftFront + mSensorData.irdistances.leftBack) / 2.0f;
            rotationSpeed += K_p_reachRef * (ir_left_mean - ir_right_mean);

        } else {
            if(ir_right_mean < MIN_WALL_DISTANCE) {
                rotationSpeed += K_p_reachRef * (MIN_WALL_DISTANCE - ir_right_mean);
            } else if (ir_right_mean > MAX_WALL_DISTANCE) {
                rotationSpeed += K_p_reachRef * (MAX_WALL_DISTANCE - ir_right_mean);
            }    
        }
        

        publishSpeeds(MOVEMENT_SPEED - rotationSpeed, MOVEMENT_SPEED + rotationSpeed);
        
    }