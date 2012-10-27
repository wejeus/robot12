#include "MoveFollowWall.h"
#include "amee/Velocity.h"

using namespace amee;

	MoveFollowWall::MoveFollowWall(ros::Publisher& pub) {
		mVelPub = pub;
	}

	MoveFollowWall::~MoveFollowWall() {
	}

	void MoveFollowWall::init(const SensorData& data) {
		linearSpeed = 0.1f;
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


	void MoveFollowWall::doControl(const SensorData& data) {

		// float FRONT_DISTANCE_CUTOFF = 0.1f;
		mSensorData = data;
		followWall();

		// if (state != foundFrontWall && mDistances.frontShortRange < FRONT_DISTANCE_CUTOFF) {
		// 	std::cout << "GOING TO STATE: foundFrontWall" << std::endl;
		// 	state = foundFrontWall;
			
		// } else if (mDistances.frontShortRange > FRONT_DISTANCE_CUTOFF) {
		// 	std::cout << "GOING TO STATE: followSideWall" << std::endl;
		// 	state = followSideWall;
		// } else {
		// 	std::cout << "GOING TO STATE: doNothing" << std::endl;
		// 	state = doNothing;
		// }

		// switch (state) {
		// 	case foundFrontWall:
		// 		rotate(); // use MoveRotate for that
		// 		break;
		// 	case followSideWall:
		// 		followWall();
		// 		break;
		// 	case doNothing:
		// 		break;
		// 	default: std::cout << "UNKNOWN STATE" << std::endl;
		// }
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