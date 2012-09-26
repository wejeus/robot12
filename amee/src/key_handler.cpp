#include "ros/ros.h"
#include "std_msgs/String.h"
#include "amee/Velocity.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>
#include <cmath>

#define _USE_MATH_DEFINES


/* Hopefully this will work for all types of keyboards with arrows */
enum {KEYCODE_U = 65, KEYCODE_D, KEYCODE_R, KEYCODE_L};

/* Holds the velocities of the (left & right) wheels */
amee::Velocity v;

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

inline void setWheels(float L=0.0f, float R=0.0f){
	v.left = L; v.right = R;
}

void startKeyboardHandling(int argc, char **argv){
	ros::init(argc, argv, "KeyHandler");
	ros::NodeHandle n;

	ros::Publisher keyCom_pub = n.advertise<amee::Velocity>("wheel_velocities", 1000);

	ros::Rate loop_rate(10);

	//sets the velocities of the wheels to initial values (0.0f)
	setWheels();
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
						setWheels(-1.0f, 1.0f);
						break;
					case KEYCODE_R:
						ROS_DEBUG("RIGHT");
						setWheels(1.0f, -1.0f);
						break;
					case KEYCODE_U:
						ROS_DEBUG("UP");
						setWheels(1.0f, 1.0f);
						break;
					case KEYCODE_D:
						ROS_DEBUG("DOWN");						
						setWheels(-1.0f, -1.0f);
						break;
					default:
						ROS_DEBUG("RELEASE");
						setWheels();
						break;
				}
			}
		}else //if we don't push any key
			setWheels(); //reset

		/* publish the wheel velocities */
		keyCom_pub.publish(v);

		ros::spinOnce();

		loop_rate.sleep();
	}

}

/* This is for handling the input from the terminal */
void terminalHandler(int argc, char **argv){
	bool inOK = false;
	int in;

	while(!inOK){
		printf("\033[2J\033[1;1H");
		puts("###################################################################\n#");
		puts("#  1. Navigate the robot with the keyboard arrows.");
		puts("#  2. Give the arguments (A,D,x,y) for the robot movement.");
		puts("#  3. Do a random movement");
		puts("#  \n0. Quit");
		puts("###################################################################\n\n");
	
		in = int(getchar()) - 48;

		/* if the input values are in the intervall */
		if(in >= 0 && in <= 3)
			inOK = true;
	}

	printf("The input was %d", in);


	if(in == 0) exit(0);

	if(in == 1)
		startKeyboardHandling(argc, argv);

	if(in == 2){
		float A, D, x, y;
		const float NAN_F = -888888.8f;
		A = D = x = y = NAN_F;

		inOK = false;
		while(!inOK){
			printf("Input values for A D x y: ");
			scanf("%f %f %f %f", &A, &D, &x, &y);

			if(A == NAN_F || D == NAN_F || x == NAN_F || y == NAN_F){		
				printf("\033[2J\033[1;1H"); //clear the screen
				puts("Did you forget to input all the values? (A D x y)");
				puts("Try again");
			}else{
				inOK = true;
			}
		}
		
		puts("I need some code to start the FakeMotor and the MotorControl with the given values.");
		puts("Exiting");
		exit(1);
		//TODO: Start the FakeMotor
		//TODO: Start the Motorcontrol with the values
		
	}	

	if(in == 3){
		puts("Not coded yet! Exiting.");
		exit(0);
	}
	
}


int main(int argc, char **argv){
	
	terminalHandler(argc, argv);// get input from terminal


	return 0;
}


