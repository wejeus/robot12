#include "MoveFollowWall.h"
#include "amee/Velocity.h"
#include "MoveRotate.h"
#include "MoveStraight.h"
#include "MoveAlignWall.h"
#include "MoveAlignToFrontWall.h"
#include "MovementControl.h"
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

        publishState(INIT);
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

        // always call nextToWall(), so that it is always initialized and always describes the true situation
        nextToWall();

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
            MoveFollowWall::publishState(T_INTERSECTION_HANDLING_IN);
            mFrontWallAligner->init(mSensorData, MIN_WALL_DISTANCE);    
        }

        // if we see a wall reset the gapDistance (because there is no gap!!)
        if (seesWall(mSensorData.irdistances.rightFront) || seesWall(mSensorData.irdistances.rightBack)) {
            mState.gapStartDist = mSensorData.odometry.distance;
        }

        // if we have seen a big enough gap on the right side, we don't need to align to the front wall anymore
        if (mSensorData.odometry.distance - mState.gapStartDist >= TAIL_LENGTH) {
            mState.set(RotateRight);
            MoveFollowWall::publishState(T_INTERSECTION_HANDLING_OUT);
            return;
        }

        if (mFrontWallAligner->isRunning()) {
            mFrontWallAligner->doControl(mSensorData);
        } else {
            mState.set(RotateRight);
            MoveFollowWall::publishState(T_INTERSECTION_HANDLING_OUT);
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
            MoveFollowWall::publishState(ALIGN_TO_WALL_IN);
        }

        if (mWallAligner->isRunning()) {
            mWallAligner->doControl(mSensorData);
        } else {
            MoveFollowWall::publishState(ALIGN_TO_WALL_OUT);
            mState.set(FollowWall);
        }

    }

    void MoveFollowWall::alignToFrontWallState() {
        
        if (!mState.initialized) {
            publishState(ALIGN_TO_FRONT_WALL_IN);
            std::cout << "AlignToFrontWall init" << std::endl;
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            mFrontWallAligner->init(mSensorData, MIN_WALL_DISTANCE);    
        }

        if (mFrontWallAligner->isRunning()) {
            mFrontWallAligner->doControl(mSensorData);
        } else {
            mState.set(RotateLeft);
            publishState(ALIGN_TO_FRONT_WALL_OUT);
        }
    }

    void MoveFollowWall::lookForBeginningOfWallState() {
        
        if (!mState.initialized) {
            std::cout << "LookForBeginningOfWall init" << std::endl;
            publishState(LOOK_FOR_BEGINNING_OF_WALL_IN);
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            mFoundWallRightBack = false;
            mFoundWallRightFront = false;
            mStraightMove->init(SLOW_MOVEMENT_SPEED);
        }

        if (wallInFront()) { // in case there is for whatever evil reason a wall in front
            publishState(LOOK_FOR_BEGINNING_OF_WALL_OUT);
            mState.set(AlignToFrontWall);
            return;
        }

        bool rF = seesWall(mSensorData.irdistances.rightFront);
        bool rB = seesWall(mSensorData.irdistances.rightBack);
        mFoundWallRightFront = mFoundWallRightFront || rF;
        mFoundWallRightBack = mFoundWallRightBack || rB;

        if (nextToWall()) {
            mState.set(AlignToWall);
            publishState(LOOK_FOR_BEGINNING_OF_WALL_OUT);
            return;
        }

        if (mFoundWallRightBack && mFoundWallRightFront) {
            // both sensors have seen the wall, but it's not continues, so go to evil wall handling
            mState.set(HandleEvilWalls);
            publishState(LOOK_FOR_BEGINNING_OF_WALL_OUT);
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
            publishState(ROTATE_LEFT_IN);
          }
          if (mRotater->isRunning()) { 
            mRotater->doControl(mSensorData);
          } else {
            if (wallInFront()){
                mState.set(AlignToFrontWall);
            } else {
                mState.set(HandleEvilWalls);
            }
            publishState(ROTATE_LEFT_OUT);
          }
    }

    void MoveFollowWall::rotateRightState() {
        
          if (!mState.initialized) {
            std::cout << "RotateRight init" << std::endl;
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            mRotater->init(mSensorData, -90.0f);
            publishState(ROTATE_RIGHT_IN);
          }
          if (mRotater->isRunning()) { 
            mRotater->doControl(mSensorData);
          } else {
            mState.set(LookForBeginningOfWall);
            publishState(ROTATE_RIGHT_OUT);   
          }
    }

    void MoveFollowWall::moveTailState() {
        
        if (wallInFront()) {
            if (mState.initialized) {
                publishState(MOVE_TAIL_OUT);
            }
            mState.set(TIntersectionHandling);
        } else { 
            if (!mState.initialized) {
                std::cout << "MoveTail init" << std::endl;
                // publishSpeeds(0.0f,0.0f);
                publishState(MOVE_TAIL_IN);
                mState.gapStartDist = mSensorData.odometry.distance; // to measure the size of the seen gap
                mState.initialized = true;
                mStraightMove->init(mSensorData, TAIL_LENGTH,SLOW_MOVEMENT_SPEED);
            }
            if (mStraightMove->isRunning()) {
                mStraightMove->doControl(mSensorData);
            } else {
                publishState(MOVE_TAIL_OUT);
                mState.set(RotateRight);   
            }
        }
    }

    void MoveFollowWall::lookForEndOfWallState() {
              
        if (!mState.initialized) {
            std::cout << "LookForEndOfWall init" << std::endl;
            publishSpeeds(0.0f,0.0f);
            publishState(LOOK_FOR_END_OF_WALL_IN);
            mState.initialized = true;
            mStraightMove->init(SLOW_MOVEMENT_SPEED);
        }

        if (wallInFront()) {
            // publishSpeeds(0.0f,0.0f);
            publishState(LOOK_FOR_END_OF_WALL_OUT);
            mState.set(TIntersectionHandling);
            return;
        }

        if (seesWall(mSensorData.irdistances.rightFront)) {
            publishState(LOOK_FOR_END_OF_WALL_OUT);
            mState.set(LookForBeginningOfWall); 
        } else if (!seesWall(mSensorData.irdistances.rightFront) 
            && !seesWall(mSensorData.irdistances.rightBack)) {
            publishState(LOOK_FOR_END_OF_WALL_OUT);
            mState.set(MoveTail);
        } else {
            mStraightMove->doControl(mSensorData);
        }
    }

    void MoveFollowWall::edgeOfWallState() {
        
        if (!mState.initialized) {
            std::cout << "edgeOfWallState init" << std::endl;
            publishState(EDGE_OF_WALL_IN);
            mStraightMove->init(SLOW_MOVEMENT_SPEED);
            mState.initialized = true;
        }

        if(wallInFront()) {
            mState.set(TIntersectionHandling);
            publishState(EDGE_OF_WALL_OUT);
            publishSpeeds(0.0f,0.0f);
        }

        float error = fabs(mSensorData.irdistances.rightBack - mSensorData.irdistances.rightFront);

        if (error <= IR_ERROR_THRESHOLD) {
            mState.set(FollowWall);
            publishState(EDGE_OF_WALL_OUT);
        } else if (!seesWall(mSensorData.irdistances.rightFront)) {
            mState.set(LookForEndOfWall);
            publishState(EDGE_OF_WALL_OUT);
        } else {
            mStraightMove->doControl(mSensorData);
        }
    }

    void MoveFollowWall::followWallState() {
        if (!mState.initialized) {
            publishState(FOLLOW_WALL_IN);
            std::cout << "FollowWall init" << std::endl;    
            mState.initialized = true;
        }
        
        // std::cout << mSensorData.irdistances << std::endl;
        if (wallInFront()) {
            mState.set(AlignToFrontWall);
            publishState(FOLLOW_WALL_OUT);
            return;
        }

        if (seesWall(mSensorData.irdistances.rightBack) && seesWall(mSensorData.irdistances.rightFront)) {
            
            float error = fabs(mSensorData.irdistances.rightBack - mSensorData.irdistances.rightFront);
            if (error > IR_ERROR_THRESHOLD) {
                mState.set(EdgeOfWall);
                publishState(FOLLOW_WALL_OUT);
            } else {
                followWall();    
            } 

         } else if (!seesWall(mSensorData.irdistances.rightFront) && seesWall(mSensorData.irdistances.rightBack)) {
            publishState(FOLLOW_WALL_OUT);
            mState.set(LookForEndOfWall);
         } else if (seesWall(mSensorData.irdistances.rightFront) && !seesWall(mSensorData.irdistances.rightBack)) {
            mState.set(LookForBeginningOfWall);
            publishState(FOLLOW_WALL_OUT);
         } else {
           mState.set(HandleEvilWalls);
           publishState(FOLLOW_WALL_OUT);
        }
    }

    void MoveFollowWall::handleEvilWallsState() {
        if (!mState.initialized) {
            std::cout << "handleEvilWallsState init" << std::endl;
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            // initialize this value so that nextToWall() can work
            // mSeenWallStartDist = mSensorData.odometry.distance;
            publishState(HANDLE_EVIL_WALLS_IN);
            mStraightMove->init(mSensorData, mState.handleWallInitDistance);
        }
        // stop if wall in front
        if (wallInFront()) {
            if (seesWall(mSensorData.irdistances.rightFront)) {
                mState.set(AlignToFrontWall);
                publishState(HANDLE_EVIL_WALLS_OUT);
            } else {
                publishState(HANDLE_EVIL_WALLS_OUT);
                mState.set(TIntersectionHandling);    
            }
            return;
        }

        // continue wall following if we see a wall
        if (nextToWall()) {
            publishState(HANDLE_EVIL_WALLS_OUT);
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
                publishState(HANDLE_EVIL_WALLS_OUT);
                mState.set(RotateRight);    
            }
        }
    }

    void MoveFollowWall::publishState(int state) { // publishes the state change

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

    bool MoveFollowWall::wallInFront() {
        return MovementControl::wallInFront(mSensorData);
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