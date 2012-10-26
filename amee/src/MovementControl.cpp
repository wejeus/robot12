#include "ros/ros.h"
#include "MovementControl.h"
#include "amee/Velocity.h"
#include <iostream>
#include <cmath>

using namespace amee;


MovementControl::MovementControl(ros::Publisher pub) {
	setSpeedPublisher(pub);
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
	publishSpeeds(0.0f,0.0f);


	ros::param::set("/linearSpeed",(double)linearSpeed);
	ros::param::set("/K_p_keepRef", (double)K_p_keepRef);
	ros::param::set("/K_p_reachRef", (double)K_p_reachRef);
	ros::param::set("/K_i_keepRef", (double)K_i_keepRef);
	ros::param::set("/K_i_reachRef", (double)K_i_reachRef);
	ros::param::set("/refDistance", (double)refDistance);
	ros::param::set("/maxErrorSum", (double)maxErrorSum);

	state = doNothing;
}

void MovementControl::setSpeedPublisher(ros::Publisher pub) {
	speed_pub = pub;
}

// Callback for IR distances
void MovementControl::receive_distances(const IRDistances::ConstPtr &msg)
{
	mDistances.rightFront = msg->rightFront;
	mDistances.rightBack = msg->rightBack;
	mDistances.frontShortRange = msg->frontShortRange;
}

void MovementControl::publishSpeeds(float left, float right) {
	Velocity v;
	v.left = left;
	v.right = right;
	speed_pub.publish(v);
}

void MovementControl::followWall() {
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
	 
	float ir_right_mean = (mDistances.rightBack + mDistances.rightFront)/2.0f;
	float angle_to_wall = tan((mDistances.rightBack - mDistances.rightFront) / IR_BASE_RIGHT);
    float distance_to_wall = cos(angle_to_wall) * ir_right_mean;
            
	std::cout << "DISTANCE TO WALL: " << distance_to_wall << "\n";
    std::cout << "ANGLE TO WALL: " << angle_to_wall << std::endl;


    float rotationSpeed = 0.0f;
	if (fabs(refDistance - distance_to_wall) < wallDistTol) { // if were at distance_to_wall +- 5cm
        std::cout << "KEEPING REFERENCE" << std::endl;

        float error = mDistances.rightBack - mDistances.rightFront;
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

void MovementControl::goStraight() {
	publishSpeeds(0.0f, 0.0f);
	std::cout << "GO STRAIGHT" << std::endl;
}


// mTargetAngle
// mIsRotating
// mCurrentRelativeAngle
// mPreviousAngle

// Currentlly hardcoded to use 90 degree turns
void MovementControl::rotate() {
	std::cout << "ROTATING" << std::endl;

	float MAX_ROTATION_SPEED = 0.1;
	float MIN_ROTATION_SPEED = 0.06;

	// New rotation requested, init rotation procedure
	if ( ! mIsRotating) {
		mIsRotating = true;
		mTargetAngle = 90.0f;
		mCurrentRelativeAngle = 0.0f;
	}

	// If here we continuing a previously started rotation
	if (mCurrentRelativeAngle < (mTargetAngle + 0.05f)) {
	    float K_p = 1.0/200.0; // starts to slow down 20 degrees before final angle
	    float angleError = mTargetAngle - mCurrentRelativeAngle;

	    // lower speed as we come closer to "degreesToTravel"
	    float rotationSpeed = K_p * angleError

	    if (abs(rotationSpeed) > MAX_ROTATION_SPEED) {
	    	// saturate speed to ROTATION_SPEED if too high
	    	rotationSpeed = std::copysign(MAX_ROTATION_SPEED, rotationSpeed);
	    } else {
	    	// saturate speed to ROTATION_SPEED if too low
	    	rotationSpeed = std::copysign(MIN_ROTATION_SPEED, rotationSpeed);
	    }

	    publishSpeeds(rotationSpeed, -rotationSpeed);
		
		mCurrentRelativeAngle = abs(mTotalAngle - mPreviousAngle);
	} else {
		// Rotation done! Reset current angle movement
		std::cout << "TARGET ANGLE REACHED" << std::endl;
		mIsRotating = false;
		publishSpeeds(0.0f, 0.0f);
	}
}


void MovementControl::doControl() {

	
	float FRONT_DISTANCE_CUTOFF = 0.1f;

	if (state != foundFrontWall && mDistances.frontShortRange < FRONT_DISTANCE_CUTOFF) {
		std::cout << "GOING TO STATE: foundFrontWall" << std::endl;
		state = foundFrontWall;
		
	} else if (mDistances.frontShortRange > FRONT_DISTANCE_CUTOFF) {
		std::cout << "GOING TO STATE: followSideWall" << std::endl;
		state = followSideWall;
	} else {
		std::cout << "GOING TO STATE: doNothing" << std::endl;
		state = doNothing;
	}

	switch (state) {
		case foundFrontWall:
			rotate();
			break;
		case followSideWall:
			followWall();
			break;
		case doNothing:
			break;
		default: std::cout << "UNKNOWN STATE" << std::endl;
	}
}

int main(int argc, char **argv)
{
	
	ros::init(argc, argv, "MovementControlNode");//Creates a node named "MotorControl"
	ros::NodeHandle n;

	ros::Publisher	vel_pub;
	vel_pub = n.advertise<Velocity>("/amee/motor_control/set_wheel_velocities", 100);

	// create the controller and initialize it
	MovementControl control(vel_pub);

	ros::Subscriber dist_sub;

	// create subscriber for distances
	dist_sub = n.subscribe("/amee/sensors/irdistances", 100, &MovementControl::receive_distances, &control);

	ros::Rate loop_rate(20);
	while(vel_pub.getNumSubscribers() == 0 && ros::ok()) {
		loop_rate.sleep();
	} 
	
	while(ros::ok()){
		
		// go to sleep for a short while
		loop_rate.sleep();

		// call all callbacks
		ros::spinOnce();
		
		// drive!
		control.doControl();
	}

	return 0;
}
