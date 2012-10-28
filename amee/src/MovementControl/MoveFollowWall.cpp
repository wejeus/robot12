#include "MoveFollowWall.h"
#include "amee/Velocity.h"

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

using namespace amee;

    MoveFollowWall::MoveFollowWall(ros::Publisher& pub) {
        mVelPub = pub;
        mRotater = new MoveRotate(pub);
    }

    MoveFollowWall::~MoveFollowWall() {
        delete mRotater;
    }

    void MoveFollowWall::init(const SensorData& data) {
        linearSpeed = 0.2f;
        K_p_keepRef = 0.15f;
        K_p_reachRef = 0.3f;
        K_i_keepRef = 0.0f;
        K_i_reachRef = 0.0f;
        maxErrorSum = 100.0f;
        refDistance = 0.04f;
        noWallDistance = 0.15f;
        wallDistTol = 0.01f;
        error_sum = 0.0f;

        mRunning = true;
        mState = followSideWall;

        ros::param::set("/linearSpeed",(double)linearSpeed);
        ros::param::set("/K_p_keepRef", (double)K_p_keepRef);
        ros::param::set("/K_p_reachRef", (double)K_p_reachRef);
        ros::param::set("/K_i_keepRef", (double)K_i_keepRef);
        ros::param::set("/K_i_reachRef", (double)K_i_reachRef);
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

        float FRONT_DISTANCE_CUTOFF = 0.2f;
        float RIGHT_DISTANCE_CUTOFF = 0.4f;
        mSensorData = data;
        
        

        if (mState != foundFrontWall && mSensorData.irdistances.frontShortRange < FRONT_DISTANCE_CUTOFF) {
            float angleToWall = tan((mSensorData.irdistances.rightBack - mSensorData.irdistances.rightFront) / IR_BASE_RIGHT) * (180/M_PI);
            angleToWall = (-1.0f) * angleToWall; // compensate for current location
            std::cout << "GOING TO STATE: foundFrontWall" << std::endl;
            mRotater->init(mSensorData, (90 + angleToWall));
            mState = foundFrontWall;
        } else if (mSensorData.irdistances.rightFront > RIGHT_DISTANCE_CUTOFF) {
            // TODO: MoveStraight needs to be implemented to get this to work...
            // std::cout << "GOING TO STATE: foundEndOfWall" << std::endl;
            // mStraightMover->init(mSensorData, 0.1f);
            // mState = foundEndOfWall;
        }


        switch (mState) {
            case foundFrontWall:
                if (mRotater->isRunning()) {
                    mRotater->doControl(mSensorData); // use MoveRotate for that
                } else {
                    std::cout << "GOING TO STATE: followSideWall" << std::endl;
                    mState = followSideWall;
                }
                break;
            case foundEndOfWall:
                // Warning: this is a bit ugly state control...
                if (mStraightMover->isRunning()) {
                    mStraightMover->doControl(mSensorData);
                } else {
                    // Now we finished moving a bit forward, rotate around the wall then go back to wall following
                    if ( ! mRotater->isRunning()) {
                        mRotater->init(mSensorData, 90);
                    } else {
                        mRotater->doControl(mSensorData);
                        // We need to check if finished here so we can go to a new state
                        if ( ! mRotater->isRunning()) {
                            mState = followWall;
                        }
                    }
                }
            case followSideWall:
                followWall();
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
        ros::param::getCached("/refDistance", temp);
        refDistance = (float) temp;
        ros::param::getCached("/maxErrorSum", temp);
        maxErrorSum = (float) temp;
         
        float ir_right_mean = (mSensorData.irdistances.rightBack + mSensorData.irdistances.rightFront)/2.0f;
        float angle_to_wall = tan((mSensorData.irdistances.rightBack - mSensorData.irdistances.rightFront) / IR_BASE_RIGHT);
        float distance_to_wall = cos(angle_to_wall) * ir_right_mean;
                
        std::cout << "DISTANCE TO WALL: " << distance_to_wall << "\n";
        std::cout << "ANGLE TO WALL: " << angle_to_wall << std::endl;


        float rotationSpeed = 0.0f;
        if (fabs(refDistance - distance_to_wall) < wallDistTol) { // if were at distance_to_wall +- 5cm
            std::cout << "KEEPING REFERENCE" << std::endl;

            float error = mSensorData.irdistances.rightBack - mSensorData.irdistances.rightFront;
            error_sum += error;
            rotationSpeed = K_p_keepRef * error + K_i_keepRef * error_sum; 
            std::cout << "ERROR_SUM:" << error_sum << std::endl;
        } else {
            std::cout << "REACHING REFERENCE" << std::endl;

            float error = refDistance - ir_right_mean;
            error_sum += error;
            rotationSpeed = K_p_reachRef * error + K_i_reachRef * error_sum;// # TODO: add integrating control if needed
            std::cout << "ERROR_SUM:" << error_sum;
        }
        float sign = error_sum >= 0 ? 1.0f : -1.0f;
        error_sum = fabs(error_sum) > maxErrorSum ? sign * maxErrorSum : error_sum;

        publishSpeeds(linearSpeed - rotationSpeed, linearSpeed + rotationSpeed);
        
    }