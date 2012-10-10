#include "ros/ros.h"
#include "std_msgs/String.h"
#include "amee/KeyboardCommand.h"
#include "amee/Encoder.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


/* Hopefully this will work for all types of keyboards with arrows */
enum {KEYCODE_SPACE = 32, KEYCODE_U = 65, KEYCODE_D, KEYCODE_R, KEYCODE_L};

/* Holds the velocities of the (left & right) wheels */
//amee::KeyboardCommand v;
amee::Encoder v;

/* The volocity change rate */
long VELO_RATE = 1l;
struct timeval start;



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
inline void setWheels(const long L=0, const long R=0){
	//v.linear = L; v.angular = A;
	gettimeofday(&start, NULL);
	v.left 	= L;
	v.right = R;
	v.timestamp = start.tv_sec+double(start.tv_usec)/1000000.0;
}


/**
 * Gets input from keyboard and publishes it to "wheel_velocities" topic.
 *
 * Use the keyboard arrows to navigate the robot.
 **/
void startKeyboardHandling(int argc, char **argv){
	ros::init(argc, argv, "KeyHandler");
	ros::NodeHandle n;

	ros::Publisher keyCom_pub = n.advertise<amee::Encoder>("/serial/encoder", 100000);

	ros::Rate loop_rate(10);

	//sets the velocities of the wheels to initial values (0.0f)
	setWheels();
	char c = '0';//, lastKey = '0';
	puts("Press the arrow keys to move around.");
	while (ros::ok()){

		if(kbhit()){
			c = getchar();
			if(int(c) == 27 || int(c) == 32){//if we hit an arrow key or space (or an escape key)
				c = getchar(); c = getchar(); //read 2 more chars
				switch(c){
					case KEYCODE_L:
						ROS_DEBUG("LEFT");
						setWheels(-VELO_RATE, VELO_RATE);
						break;
					case KEYCODE_R:
						ROS_DEBUG("RIGHT");
						setWheels(VELO_RATE, -VELO_RATE);
						break;
					case KEYCODE_U:
						ROS_DEBUG("UP");
						setWheels(VELO_RATE, VELO_RATE);
						break;
					case KEYCODE_D:
						ROS_DEBUG("DOWN");						
						setWheels(-VELO_RATE, -VELO_RATE);
						break;
					default:
						ROS_DEBUG("RELEASE");
						setWheels();
						break;
				}
				keyCom_pub.publish(v);
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
		VELO_RATE = atoi(argv[1]);
	}

	printf("VELO_RATE is set to: %i\n", VELO_RATE);
	startKeyboardHandling(argc, argv);

	return 0;
}


