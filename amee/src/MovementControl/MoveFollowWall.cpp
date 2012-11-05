#include "MoveFollowWall.h"
#include "amee/Velocity.h"
#include "MoveRotate.h"
#include "MoveStraight.h"
#include "MoveAlignWall.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace amee;

    MoveFollowWall::MoveFollowWall(ros::Publisher& pub) {
        mVelPub = pub;
        mRotater = new MoveRotate(pub);
        mStraightMove = new MoveStraight(pub);
        mWallAligner = new MoveAlignWall(pub);
    }

    MoveFollowWall::~MoveFollowWall() {
        delete mRotater;
        delete mStraightMove;
        delete mWallAligner;
    }

    void MoveFollowWall::init(const SensorData& data) {
        linearSpeed = 0.2f;
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
                alignToWall();
                break;
            default: std::cout << "UNKNOWN STATE" << std::endl;
        }
    }

    // only go this state if you're sure there is a wall it can align to. If there is no wall it will dance or rotate at most 180 degrees, but the 
    // robot will be lost, so make sure there is a wall before going into this state!
    // After aligning it changes to FollowWall.
    void MoveFollowWall::alignToWall() {
        std::cout << "AlignToWall" << std::endl;
        if (!mState.initialized) {
            mState.initialized = true;
            mWallAligner->init(mSensorData);
        }

        if (mWallAligner->isRunning()) {
            mWallAligner->doControl(mSensorData);
        } else {
            mState.set(FollowWall);
        }
    }

    void MoveFollowWall::lookForBeginningOfWallState() {
        std::cout << "LookForBeginningOfWall" << std::endl;
        if (!mState.initialized) {
            mState.initialized = true;
            mFoundWallRightBack = false;
            mFoundWallRightFront = false;
            mStraightMove->init();
        }

        if (wallInFront()) { // in case there is for whatever evil reason a wall in front
            mState.set(RotateLeft);
            return;
        }
        mFoundWallRightFront = mFoundWallRightFront || seesWall(mSensorData.irdistances.rightFront);
        mFoundWallRightBack = mFoundWallRightBack || seesWall(mSensorData.irdistances.rightBack);

        if (mFoundWallRightBack && mFoundWallRightFront) {
            // both sensors have seen the wall, check if the wall is still there
            if (seesWall(mSensorData.irdistances.rightBack) 
                && seesWall(mSensorData.irdistances.rightFront)) {
                mState.set(AlignToWall);
            } else {
                mState.set(MoveTail);
            }
        } else {
            mStraightMove->doControl(mSensorData);
        }
    }

    void MoveFollowWall::rotateLeftState() {
        std::cout << "RotateLeft" << std::endl;
          if (!mState.initialized) {
            mState.initialized = true;
            mRotater->init(mSensorData, 90.0f);
          }
          if (mRotater->isRunning()) { 
            mRotater->doControl(mSensorData);
          } else {
            if (seesWall(mSensorData.irdistances.rightBack) 
                && seesWall(mSensorData.irdistances.rightFront)) { // in most cases there will be a wall, but maybe there is a small hole
                mState.set(AlignToWall);
            } else { // if there is no wall go to FollowWall. It will imediately detect that there is something wrong and deal with it.
                mState.set(FollowWall);    
            }  
          }
    }

    void MoveFollowWall::rotateRightState() {
        std::cout << "RotateRight" << std::endl;
          if (!mState.initialized) {
            mState.initialized = true;
            mRotater->init(mSensorData, -90.0f);
          }
          if (mRotater->isRunning()) { 
            mRotater->doControl(mSensorData);
          } else {
            mState.set(LookForBeginningOfWall);
          }
    }

    void MoveFollowWall::moveTailState() {
        std::cout << "MoveTail" << std::endl;
        if (wallInFront()) {
            mState.set(RotateLeft);
        } else { 
            if (!mState.initialized) {
                mState.initialized = true;
                mStraightMove->init(mSensorData, 0.11f);//TODO set as constant
            }
            if (mStraightMove->isRunning()) {
                mStraightMove->doControl(mSensorData);
            } else {
                mState.set(RotateRight);   
            }
        }
    }

    void MoveFollowWall::lookForEndOfWallState() {
        std::cout << "LookForEndOfWall" << std::endl;
        if (seesWall(mSensorData.irdistances.rightFront)) {
            mState.set(LookForBeginningOfWall); 
        } else if (wallInFront()) {
            mState.set(RotateLeft);
        } else if (!seesWall(mSensorData.irdistances.rightFront) 
            && !seesWall(mSensorData.irdistances.rightBack)) {
            mState.set(MoveTail);
        } else {
            if (!mState.initialized) {
                mState.initialized = true;
                mStraightMove->init();
            }
            mStraightMove->doControl(mSensorData);
        }
    }

    void MoveFollowWall::followWallState() {
        std::cout << "FollowWall" << std::endl;
        std::cout << mSensorData.irdistances << std::endl;
        if (seesWall(mSensorData.irdistances.rightBack)
            && seesWall(mSensorData.irdistances.rightFront)
            && !wallInFront()) {
                followWall();    
         } else if (!seesWall(mSensorData.irdistances.rightFront)) {
            mState.set(LookForEndOfWall);
         } else if (wallInFront()) {
            mState.set(RotateLeft);
         } else {
           std::cout << "HELP ME!!!! I'M LOST!!!!!" << std::endl;
        }
    }

    bool MoveFollowWall::isRunning() const {
        return mRunning;
    }

    bool MoveFollowWall::seesWall(float distance) {
        //TODO set as constants
        return distance >= 0.0f && distance <= 0.14f;
    }

    bool MoveFollowWall::wallInFront() {
        // TODO use constants
        return (mSensorData.irdistances.frontShortRange <= 0.09f && mSensorData.irdistances.frontShortRange >= 0.0f)
            || (mSensorData.irdistances.wheelRight <= 0.09f && mSensorData.irdistances.wheelRight >= 0.0f);
    }

    void MoveFollowWall::followWall() {
        // std::cout << "Following wall" << std::endl;
        //TODO make this nice (use floats and not double)
        double temp;    
        ros::param::getCached("/linearSpeed",temp);
        linearSpeed = (float) temp;
        ros::param::getCached("/K_p_keepRef", temp);
        K_p_keepRef = (float) temp;
        ros::param::getCached("/K_p_reachRef", temp);
        K_p_reachRef = (float) temp;
        ros::param::getCached("/K_i_keepRef", temp);
        K_i_keepRef = (float) temp;
        ros::param::getCached("/K_i_reachRef", temp);
        K_i_reachRef = (float) temp;

        ros::param::getCached("/K_d", temp); // added derivative control
        K_d = (float) temp;

        ros::param::getCached("/refDistance", temp);
        refDistance = (float) temp;
        ros::param::getCached("/maxErrorSum", temp);
        maxErrorSum = (float) temp;
         
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
        
        if(ir_right_mean < TOO_CLOSE_TO_WALL) {
            rotationSpeed += K_p_reachRef * (TOO_CLOSE_TO_WALL - ir_right_mean);
        } else if (ir_right_mean > TOO_FAR_FROM_WALL) {
            rotationSpeed += K_p_reachRef * (TOO_FAR_FROM_WALL - ir_right_mean);
        }

       publishSpeeds(linearSpeed - rotationSpeed, linearSpeed + rotationSpeed);
        
    }