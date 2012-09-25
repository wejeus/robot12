#include "ros/ros.h"
#include "std_msgs/String.h"
#include "amee/Velocity2.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>

#define _USE_MATH_DEFINES


/* Hopefully this will work for all types of keyboards with arrows */
enum {KEYCODE_U = 65, KEYCODE_D, KEYCODE_R, KEYCODE_L};

/* This is the rotation angle at each keypress */
const float ANGLE = 10*M_PI/180;

/**
 * This is for handling the nonblocking key presses
 **/
int kbhit(void){
	struct termios oldt, newt;
	int ch;
	int oldf;

	tcgetattr(STDIN_FILENO, &oldt);
	newt = oldt;
	newt.c_lflag &= ~(ICANON | ECHO);
	tcsetattr(STDIN_FILENO, TCSANOW, &newt);
	oldf = fcntl(STDIN_FILENO, F_GETFL, 0);
	fcntl(STDIN_FILENO, F_SETFL, oldf | O_NONBLOCK);
 
	ch = getchar();
 
	tcsetattr(STDIN_FILENO, TCSANOW, &oldt);
	fcntl(STDIN_FILENO, F_SETFL, oldf);
 
	if(ch != EOF)
	{
		ungetc(ch, stdin);
		return 1;
	}
 
	return 0;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "KeyHandler");
	ros::NodeHandle n;

	ros::Publisher keyCom_pub = n.advertise<amee::Velocity2>("key_commands", 1000);

	ros::Rate loop_rate(10);

	/**
	 * A count of how many messages we have sent. This is used to create
	 * a unique Velocity for each publish.
	 */
	int count = 0;
	amee::Velocity2 v;
	char c;
	ROS_INFO("Press the arrow keys to move around.");
	while (ros::ok()){

		if(kbhit()){
			c = getchar();
			if(int(c) == 27){//if we hit an arrow key (or an escape key)
				c = getchar(); c = getchar(); //read 2 more chars
				switch(c){
					case KEYCODE_L:
						ROS_DEBUG("LEFT");
						v.angular = ANGLE;
						v.linear = 0.0f;
						break;
					case KEYCODE_R:
						ROS_DEBUG("RIGHT");
						v.angular = -ANGLE;
						v.linear = 0.0f;
						break;
					case KEYCODE_U:
						ROS_DEBUG("UP");
						v.linear = 1.0f;
						v.angular = 0.0f;
						break;
					case KEYCODE_D:
						ROS_DEBUG("DOWN");
						v.linear = -1.0f;
						v.angular = 0.0f;
						break;
					default:
						ROS_DEBUG("RELEASE");
						v.linear = 0.0f;
						v.angular = 0.0f;
						break;
				}
				keyCom_pub.publish(v);
			}
		}

		ros::spinOnce();

		loop_rate.sleep();
		++count;
	}

	return 0;
}


