#include "MoveFollowWall.h"
#include "amee/Velocity.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace amee;

    MoveFollowWall::MoveFollowWall(ros::Publisher& pub) {
        mVelPub = pub;
        mRotater = new MoveRotate(pub);
        mStraightMove = new MoveStraight(pub);
    }

    MoveFollowWall::~MoveFollowWall() {
        delete mRotater;
        delete mStraightMove;
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
        mState = followSideWall;

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

        float FRONT_DISTANCE_CUTOFF = 0.1f;
        float RIGHT_DISTANCE_CUTOFF = 0.22f;
        mSensorData = data;
        
        // For debug
        // followWall();
        // std::cout << "FRONT SENSOR: " << mSensorData.irdistances.frontShortRange << std::endl;

        // FIXME (MAYBE) misses one iteration due to state change

        switch (mState) {
            case foundFrontWall:
                if (mRotater->isRunning()) {
                    mRotater->doControl(mSensorData); // use MoveRotate for that
                } else {
                    std::cout << "GOING TO STATE: followSideWall" << std::endl;
                    mState = followSideWall;
                }
                break;
            case endWallHandlingBeforeRotation:
                if (mStraightMove->isRunning()) {
                    mStraightMove->doControl(mSensorData);
                } else {
                    // Now we finished moving a bit forward, rotate around the wall then go back to wall following
                    std::cout << "GOING TO STATE: endWallHandlingRotation" << std::endl;
                    mState = endWallHandlingRotation;
                    mRotater->init(mSensorData,-90);
                }
                break;
            case endWallHandlingRotation:
                if (mRotater->isRunning()) {
                    mRotater->doControl(mSensorData);
                } else {
                    std::cout << "GOING TO STATE: endWallHandlingAfterRotation" << std::endl;
                    mState = endWallHandlingAfterRotation;
                    mStraightMove->init(mSensorData, 2 * (wallDistanceError + 0.12f) + 0.02); //wallDistanceError
                }
                break;
            case endWallHandlingAfterRotation:
                if (mStraightMove->isRunning()) {
                    mStraightMove->doControl(mSensorData);
                } else {
                    std::cout << "GOING TO STATE: checkForUTurn" << std::endl;
                    mState = checkForUTurn;
                }
                break;
            case checkForUTurn:
                {
                    float rightFront = mSensorData.irdistances.rightFront;
                    float rightBack = mSensorData.irdistances.rightBack;
                    if (rightFront < RIGHT_DISTANCE_CUTOFF) { //rightBack < RIGHT_DISTANCE_CUTOFF &&
                        std::cout << "GOING TO STATE: followSideWall" << std::endl;
                        mState = followSideWall;
                    } else {
                        std::cout << "GOING TO STATE: endWallHandlingRotation" << std::endl;
                        mRotater->init(mSensorData, -90);
                        wallDistanceError = 0.0f;
                        mState = endWallHandlingRotation;
                    }
                }
                break;
            case followSideWall: {
                   followWall();
                   bool obstacleOnTheRight = mSensorData.irdistances.wheelRight < FRONT_DISTANCE_CUTOFF && mSensorData.irdistances.wheelRight >= 0;
                   if (mSensorData.irdistances.frontShortRange < FRONT_DISTANCE_CUTOFF || obstacleOnTheRight) { 
                        float angle_to_wall = tan((mSensorData.irdistances.rightBack - mSensorData.irdistances.rightFront) / IR_BASE_RIGHT) * (180/M_PI);
                        float sign = angle_to_wall >= 0 ? 1.0f : -1.0f; // compensate for current location
                        float angle_correction = angle_to_wall*sign;
                        std::cout << "GOING TO STATE: foundFrontWall" << std::endl;
                        mRotater->init(mSensorData, (90 - angle_correction));
                        mState = foundFrontWall;
                    }  else if (mSensorData.irdistances.rightFront > RIGHT_DISTANCE_CUTOFF) {
                        std::cout << "GOING TO STATE: endWallHandlingBeforeRotation" << std::endl;
                        mStraightMove->init(mSensorData, 0.165f);
                        wallDistanceError = mSensorData.irdistances.rightBack; // to be used later
                        mState = endWallHandlingBeforeRotation;
                    } 
                }
                break;
            default: std::cout << "UNKNOWN STATE" << std::endl;
        }
    }

    bool MoveFollowWall::isRunning() const {
        return mRunning;
    }

    void MoveFollowWall::followWall() {
        std::cout << "Following wall" << std::endl;
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
        float angle_to_wall = tan((mSensorData.irdistances.rightBack - mSensorData.irdistances.rightFront) / IR_BASE_RIGHT);
        float distance_to_wall = cos(angle_to_wall) * ir_right_mean;
        
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