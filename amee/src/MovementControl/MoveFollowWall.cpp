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
        linearSpeed = 0.15f;
        K_p_keepRef = 0.8f;
        K_p_reachRef = 0.3f;
        K_i_keepRef = 0.0f;
        K_i_reachRef = 0.0f;

        K_d = 0.0f;

        maxErrorSum = 100.0f;
        refDistance = 0.04f;
        noWallDistance = 0.15f;
        wallDistTol = 0.01f;
        error_sum = 0.0f;
        last_error = 0.0f;

        mRunning = true;
        mState.set(FollowWall);

        ros::param::set("/linearSpeed",(double)linearSpeed);
        ros::param::set("/K_p_keepRef", (double)K_p_keepRef);
        ros::param::set("/K_p_reachRef", (double)K_p_reachRef);
        ros::param::set("/K_i_keepRef", (double)K_i_keepRef);
        ros::param::set("/K_i_reachRef", (double)K_i_reachRef);
        ros::param::set("/K_d", (double)K_d);
        ros::param::set("/refDistance", (double)refDistance);
        ros::param::set("/maxErrorSum", (double)maxErrorSum);

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
            default: std::cout << "UNKNOWN STATE" << std::endl;
        }
    }

    void MoveFollowWall::tIntersectionHandlingState() {
        std::cout << "T Intersection" << std::endl;       
        if (!mState.initialized) {
            std::cout << "T Intersection: INIT" << std::endl;  
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;

            if (frontAlignmentPossible()) { 
                std::cout << "T Intersection: ALIGNMENT POSSIBLE" << std::endl;  
                mFrontWallAligner->init(mSensorData, MIN_WALL_DISTANCE);    
            } else {
                std::cout << "T Intersection: CAN'T ALIGN -> ROTATE" << std::endl;  
                mState.set(RotateLeft);
                return;
            }
        }

        if (mFrontWallAligner->isRunning()) {
            mFrontWallAligner->doControl(mSensorData);
        } else {
            mState.set(RotateRight);
        }
    }

    // only go this state if you're sure there is a wall it can align to. If there is no wall it will dance or rotate at most 180 degrees, but the 
    // robot will be lost, so make sure there is a wall before going into this state!
    // After aligning it changes to FollowWall.
    void MoveFollowWall::alignToWallState() {
        std::cout << "AlignToWall" << std::endl;
        if (!mState.initialized) {
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            mWallAligner->init(mSensorData);
        }

        if (mWallAligner->isRunning()) {
            mWallAligner->doControl(mSensorData);
        } else {
            
            MoveFollowWall::PublishState(ALIGNED_TO_WALL);
            mState.set(FollowWall);
        }

    }

    void MoveFollowWall::alignToFrontWallState() {
        std::cout << "AlignToFrontWall" << std::endl;
        if (!mState.initialized) {
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
        }
    }

    void MoveFollowWall::lookForBeginningOfWallState() {
        std::cout << "LookForBeginningOfWall" << std::endl;
        if (!mState.initialized) {
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            mFoundWallRightBack = false;
            mFoundWallRightFront = false;
            mStraightMove->init();
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
            MoveFollowWall::PublishState(FOUND_BEGINNING_OF_WALL);
            return;
        }

        if (mFoundWallRightBack && mFoundWallRightFront) {
            MoveFollowWall::PublishState(FOUND_BEGINNING_OF_WALL);
            // both sensors have seen the wall, but it's not continues, so go to evil wall handling
            mState.set(HandleEvilWalls);
        } else {
            mStraightMove->doControl(mSensorData);
        }
    }

    void MoveFollowWall::rotateLeftState() {
        std::cout << "RotateLeft" << std::endl;
          if (!mState.initialized) {
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            mRotater->init(mSensorData, 90.0f);
          }
          if (mRotater->isRunning()) { 
            mRotater->doControl(mSensorData);
          } else {
            PublishState(ROTATED_LEFT);
            mState.set(HandleEvilWalls);    
          }
    }

    void MoveFollowWall::rotateRightState() {
        std::cout << "RotateRight" << std::endl;
          if (!mState.initialized) {
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            mRotater->init(mSensorData, -90.0f);
          }
          if (mRotater->isRunning()) { 
            mRotater->doControl(mSensorData);
          } else {
            MoveFollowWall::PublishState(ROTATED_RIGHT);
            mState.set(LookForBeginningOfWall);
          }
    }

    void MoveFollowWall::moveTailState() {
        std::cout << "MoveTail" << std::endl;
        if (wallInFront()) {
            mState.set(TIntersectionHandling);
        } else { 
            if (!mState.initialized) {
                publishSpeeds(0.0f,0.0f);
                mState.initialized = true;
                mStraightMove->init(mSensorData, TAIL_LENGTH);
            }
            if (mStraightMove->isRunning()) {
                mStraightMove->doControl(mSensorData);
            } else {
                MoveFollowWall::PublishState(MOVED_TAIL);
                mState.set(RotateRight);   
            }
        }
    }

    void MoveFollowWall::lookForEndOfWallState() {
        std::cout << "LookForEndOfWall" << std::endl;
        if (wallInFront()) {
            publishSpeeds(0.0f,0.0f);
            mState.set(TIntersectionHandling);
            return;
        }

        if (seesWall(mSensorData.irdistances.rightFront)) {
            mState.set(LookForBeginningOfWall); 
        } else if (!seesWall(mSensorData.irdistances.rightFront) 
            && !seesWall(mSensorData.irdistances.rightBack)) {
            MoveFollowWall::PublishState(FOUND_END_OF_WALL);
            mState.set(MoveTail);
        } else {
            if (!mState.initialized) {
                publishSpeeds(0.0f,0.0f);
                mState.initialized = true;
                mStraightMove->init();
            }
            mStraightMove->doControl(mSensorData);
        }
    }

    void MoveFollowWall::followWallState() {
        std::cout << "FollowWall" << std::endl;
        // std::cout << mSensorData.irdistances << std::endl;
        if (wallInFront()) {
            mState.set(AlignToFrontWall);
            PublishState(FOLLOWED_WALL);
            return;
        }

        if (seesWall(mSensorData.irdistances.rightBack)
            && seesWall(mSensorData.irdistances.rightFront)) {
                followWall();    
         } else if (!seesWall(mSensorData.irdistances.rightFront) && seesWall(mSensorData.irdistances.rightBack)) {
            PublishState(FOLLOWED_WALL);
            mState.set(LookForEndOfWall);
         } else if (seesWall(mSensorData.irdistances.rightFront) && !seesWall(mSensorData.irdistances.rightBack)) {
            mState.set(LookForBeginningOfWall);
         } else {
           mState.set(HandleEvilWalls);
        }
    }

    void MoveFollowWall::handleEvilWallsState() {
        std::cout << "handleEvilWallsState" << std::endl;
        if (!mState.initialized) {
            publishSpeeds(0.0f,0.0f);
            mState.initialized = true;
            mStraightMove->init(mSensorData, TAIL_LENGTH);
        }
        // stop if wall in front
        if (wallInFront()) {
            mState.set(AlignToFrontWall);
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

    void MoveFollowWall::PublishState(int state) { // publishes the state

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
        bool rightOk = mSensorData.irdistances.wheelRight >= -0.03f && mSensorData.irdistances.wheelLeft <= 0.08f;
        bool leftOk = mSensorData.irdistances.wheelLeft >= -0.03f && mSensorData.irdistances.wheelRight <= 0.08f;
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
        // std::cout << "Following wall" << std::endl;
        //TODO make this nice (use floats and not double)
        // double temp;    
        // ros::param::getCached("/linearSpeed",temp);
        // linearSpeed = (float) temp;
        // ros::param::getCached("/K_p_keepRef", temp);
        // K_p_keepRef = (float) temp;
        // ros::param::getCached("/K_p_reachRef", temp);
        // K_p_reachRef = (float) temp;
        // ros::param::getCached("/K_i_keepRef", temp);
        // K_i_keepRef = (float) temp;
        // ros::param::getCached("/K_i_reachRef", temp);
        // K_i_reachRef = (float) temp;

        // ros::param::getCached("/K_d", temp); // added derivative control
        // K_d = (float) temp;

        // ros::param::getCached("/refDistance", temp);
        // refDistance = (float) temp;
        // ros::param::getCached("/maxErrorSum", temp);
        // maxErrorSum = (float) temp;
         
        float ir_right_mean = (mSensorData.irdistances.rightBack + mSensorData.irdistances.rightFront)/2.0f;
        // float angle_to_wall = tan((mSensorData.irdistances.rightBack - mSensorData.irdistances.rightFront) / IR_BASE_RIGHT);
        // float distance_to_wall = cos(angle_to_wall) * ir_right_mean;
        
        // std::cout << "DISTANCE TO WALL: " << distance_to_wall << "\n";
        // std::cout << "ANGLE TO WALL: " << angle_to_wall << std::endl;


        float rotationSpeed = 0.0f;
        //if (fabs(refDistance - distance_to_wall) < wallDistTol) { // if were at distance_to_wall +- 5cm
            // std::cout << "KEEPING REFERENCE" << std::endl;

            float error = mSensorData.irdistances.rightBack - mSensorData.irdistances.rightFront;
            error_sum += error;
            rotationSpeed = K_p_keepRef * error + K_i_keepRef * error_sum; 
            // std::cout << "ERROR_SUM:" << error_sum << std::endl;
        // } else {
            // std::cout << "REACHING REFERENCE" << std::endl;
            // std::cout << "FOLLOWING RIGHT WALL" << std::endl;
            // float error = refDistance - distance_to_wall;
            // float error_rate = (last_error - error); //  / delta_t;  assuming time step = 1
            // error_sum += error;
            // rotationSpeed = K_p_reachRef * error + K_i_reachRef * error_sum + K_d * error_rate;// # TODO: add integrating control if needed
            
            // if (fabs(angle_to_wall*180/M_PI) > 30.0f) {
            //     float sign = angle_to_wall >= 0 ? 1.0f : -1.0f;
            //     float angle_correction = 0.2f*sign;
            //     rotationSpeed = rotationSpeed + angle_correction;
            //     std::cout << "HIGH ANGLE, CORRECTION NEEDED: " << angle_to_wall << std::endl;
            // }

            // std::cout << "ERROR_SUM:" << error_sum << std::endl;
            // std::cout << "ERROR:" << error << std::endl;
            // std::cout << "ERROR_RATE:" << error_rate << std::endl;


       // }
        float sign = error_sum >= 0 ? 1.0f : -1.0f;
        error_sum = fabs(error_sum) > maxErrorSum ? sign * maxErrorSum : error_sum;
        last_error = error;
        
        if(ir_right_mean < MIN_WALL_DISTANCE) {
            rotationSpeed += K_p_reachRef * (MIN_WALL_DISTANCE - ir_right_mean);
        } else if (ir_right_mean > MAX_WALL_DISTANCE) {
            rotationSpeed += K_p_reachRef * (MAX_WALL_DISTANCE - ir_right_mean);
        }

       publishSpeeds(linearSpeed - rotationSpeed, linearSpeed + rotationSpeed);
        
    }