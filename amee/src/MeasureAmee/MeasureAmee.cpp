#include "ros/ros.h"
#include "std_msgs/String.h"
#include <std_msgs/Int32.h>
#include "amee/MovementCommand.h"
#include "amee/Odometry.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <math.h>
#include <ctime>


using namespace amee;

time_t start_t, end_t; /* We need these to let the robot run for a while before stopping her */
bool first = true;
float x_first, x_last, y_first, y_last;
float curDist = 0.0f;
float DIST_ERROR = 0.2f; //in meter
float MIN_RUNNING_TIME = 10.0f; //in seconds

bool distReached(){
	if((curDist = sqrt(pow(x_first - x_last, 2) + pow(y_first - y_last, 2))) < DIST_ERROR)
		return true;
	return false;
}

void calcPos(const Odometry::ConstPtr &odo){
	ROS_INFO("dist (%0.2f, %0.2f) -> (%0.2f, %0.2f) = %0.2f", x_first, y_first, odo->x, odo->y, curDist);
	if(first){
		x_first = odo->x; y_first = odo->y;
		first = false;
		ROS_INFO("#### First x: %0.2f, y: %0.2f ####", x_first, y_first);
	}
	x_last = odo->x; y_last = odo->y;
}


int main(int argc, char ** argv){
	if(argc > 1){
		DIST_ERROR = atof(argv[1]);
	}
	printf("DIST_ERROR is set to %0.2f\n", DIST_ERROR);


	/* Init ROS stuff */
	ros::init(argc, argv, "MeasureAmee");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<MovementCommand>("/MovementControl/MovementCommand", 1);
	ros::Subscriber sub = nh.subscribe("/amee/motor_control/odometry", 100, &calcPos);
	ros::Rate loop_rate(10);

	/* Init the start and stop msg to publish */
	MovementCommand mc, mc_stop;
	mc.type = 4; mc.distance = mc.x = mc.y = mc.angle = 0.0f;
	mc_stop.type = 5;
	x_first = y_first = 0.0f;

	/* wait for subscriber */
	while(ros::ok() && pub.getNumSubscribers() == 0) loop_rate.sleep();

	/* publish follow wall to MovementControl */
	ROS_INFO("Publishing start wallfollowing");
	pub.publish(mc);
	
	time(&start_t);

	while(ros::ok()){
		ros::spinOnce();
		
		time(&end_t);
		if(difftime(end_t, start_t) > MIN_RUNNING_TIME && distReached()){
			pub.publish(mc_stop); /* publishing stop wallfollowing to MovementControl */
			ROS_INFO("Publishing stop wallfollowing");
			ros::spinOnce();
			loop_rate.sleep();
			break;
		}	
		
		loop_rate.sleep();
	}

	puts("Quiting!");
	return 0;
}
