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


using namespace amee;
using namespace roboard_drivers;

float x_first, x_last, y_first, y_last;
float DIST_ERROR = 0.5; //in meter

bool distReached(){
	if(sqrt(abs(x_first - x_last) + abs(y_first - y_last)) < DIST_ERROR)
		return true;
	return false;
}

void calcPos(const Odometry::ConstPtr &odo){
	ROS_INFO("x: %0.2f, y: %0.2f", odo->x, odo->y);
	x_last = odo->x; y_last = odo->y;
}


int main(int argc, char ** argv){
	ros::init(argc, argv, "MeasureAmee");
	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<MovemenCommand>("/MovementControl/MovementCommand", 1);
	ros::Subscriber sub = nh.subscribe("/amee/motor_control/odometry", 100, &calcPos);
	ros::Rate loop_rate(20);

	MovementCommand mc, mc_stop;
	mc.type = 4; mc.distance = mc.x = mc.y = mc.angle = 0.0f;
	mc_stop.type = 5;
	x_first = y_first = 0.0f;


	while(ros::ok() && pub.getNumSubscribers() == 0) loop_rate.sleep(); /* wait for subscriber */

	pub.publish(mc); /* publish follow wall to MovementControl */
	ROS_INFO("Publishing start wallfollowing");


	while(ros::ok()){
		ros::spinOnce();
		
		if(distReached){
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
