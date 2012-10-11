#include "ros/ros.h"
#include "std_msgs/String.h"
#include "amee/Velocity.h"
#include "amee/Motor.h"
//#include <QWSKeyboardHandler>
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


/* Hopefully this will work for all types of keyboards with arrows */
enum {KEYCODE_SPACE = 32, KEYCODE_U = 65, KEYCODE_D, KEYCODE_R, KEYCODE_L};

using namespace amee;

/* The volocity change rate */
float VELO_RATE = 0.5;
//struct timeval start;



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

/**
 * Sets the wheel velocities to given values
 **/
inline void setWheels(Motor &motor, const float L=0.0, const float R=0.0){
	//v.linear = L; v.angular = A;
	//gettimeofday(&start, NULL);
	motor.left  = L;
	motor.right = R;
}


/**
 * Gets input from keyboard and publishes it to "wheel_velocities" topic.
 *
 * Use the keyboard arrows to navigate the robot.
 **/
void startKeyboardHandling(int argc, char **argv){
	ros::init(argc, argv, "KeyHandler");
	ros::NodeHandle n;

	ros::Publisher keyCom_pub = n.advertise<Motor>("/serial/motor_speed", 100000);

	ros::Rate loop_rate(10);

	char c = '0';//, lastKey = '0';
	puts("Press the arrow keys to move around.");
	while (ros::ok()){

		if(kbhit()){
			c = getchar();
			if(int(c) == 27 || int(c) == 32){//if we hit an arrow key or space (or an escape key)
				c = getchar(); c = getchar(); //read 2 more chars
				Motor motor;
				switch(c){
					case KEYCODE_L:
						ROS_DEBUG("LEFT");
						setWheels(motor, -VELO_RATE, VELO_RATE);
						break;
					case KEYCODE_R:
						ROS_DEBUG("RIGHT");
						setWheels(motor, VELO_RATE, -VELO_RATE);
						break;
					case KEYCODE_U:
						ROS_DEBUG("UP");
						setWheels(motor, VELO_RATE, VELO_RATE);
						break;
					case KEYCODE_D:
						ROS_DEBUG("DOWN");						
						setWheels(motor, -VELO_RATE, -VELO_RATE);
						break;
					default:
						ROS_DEBUG("RELEASE");
						setWheels(motor);
						break;
				}
				keyCom_pub.publish(motor);
			}
		}

		/* publish the wheel velocities */
		//if(lastKey != c){
		//	lastKey = c;
		//	keyCom_pub.publish(v);
		//}

		ros::spinOnce();

		loop_rate.sleep();
	}

}


int main(int argc, char **argv){
	
	if(argc > 1){
		VELO_RATE = atof(argv[1]);
	}

	printf("VELO_RATE is set to: %0.2f\n", VELO_RATE);
	startKeyboardHandling(argc, argv);

	return 0;
}


