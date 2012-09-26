#include "ros/ros.h"
#include "std_msgs/String.h"
#include "amee/KeyboardCommand.h"
#include <stdio.h>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>


/* Hopefully this will work for all types of keyboards with arrows */
enum {KEYCODE_SPACE = 32, KEYCODE_U = 65, KEYCODE_D, KEYCODE_R, KEYCODE_L};

/* Holds the velocities of the (left & right) wheels */
amee::KeyboardCommand v;

/* The volocity change rate */
float VELO_RATE = 0.5f;



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
inline void setWheels(const float L=0.0f, const float A=0.0f){
	v.linear = L; v.angular = A;
}


/**
 * Gets input from keyboard and publishes it to "wheel_velocities" topic.
 *
 * Use the keyboard arrows to navigate the robot.
 **/
void startKeyboardHandling(int argc, char **argv){
	ros::init(argc, argv, "KeyHandler");
	ros::NodeHandle n;

	ros::Publisher keyCom_pub = n.advertise<amee::KeyboardCommand>("/KeyboardControl/KeyboardCommand", 1000);

	ros::Rate loop_rate(10);

	//sets the velocities of the wheels to initial values (0.0f)
	setWheels();
	char c = '0', lastKey = '0';
	puts("Press the arrow keys to move around.");
	while (ros::ok()){

		if(kbhit()){
			c = getchar();
			if(int(c) == 27){//if we hit an arrow key (or an escape key)
				c = getchar(); c = getchar(); //read 2 more chars
				switch(c){
					case KEYCODE_L:
						ROS_DEBUG("LEFT");
						setWheels(0, 1);
						break;
					case KEYCODE_R:
						ROS_DEBUG("RIGHT");
						setWheels(0, -1);
						break;
					case KEYCODE_U:
						ROS_DEBUG("UP");
						setWheels(1, 0);
						break;
					case KEYCODE_D:
						ROS_DEBUG("DOWN");						
						setWheels(-1, 0);
						break;
					default:
						ROS_DEBUG("RELEASE");
						setWheels();
						break;
				}
			}else if(int(c) == 32){//we hit the space
				setWheels();
			}
		}

		/* publish the wheel velocities */
		if(lastKey != c){
			lastKey = c;
			keyCom_pub.publish(v);
		}

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
		puts("#  3. Do a random movement.");
		puts("#");
		puts("#  0. Quit\n#");
		puts("###################################################################\n\n");
	
		in = int(getchar()) - 48;

		/* if the input values are in the intervall */
		if(in >= 0 && in <= 3)
			inOK = true;
	}


	if(in == 0) exit(0);

	// Navigate with keyboard arrows
	if(in == 1)
		startKeyboardHandling(argc, argv);

	// Start the MotorControl with some given values
	if(in == 2){
		float A, D, x, y;
		const float NAN_F = -888888.8f;
		A = D = x = y = NAN_F;

		inOK = false;
		while(!inOK){
			printf("Input values for A D x y: ");
			scanf("%f %f %f %f", &A, &D, &x, &y);

			if(A == NAN_F || D == NAN_F || x == NAN_F || y == NAN_F){//Bad code!
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

	// Start the MotorControl with some random values
	if(in == 3){
		//TODO: code me
		puts("Not coded yet! Exiting.");
		exit(0);
	}
	
}


int main(int argc, char **argv){
	
	if(argc > 1){
		VELO_RATE = atof(argv[1]);
	}

	printf("VELO_RATE is set to: %0.2f\n", VELO_RATE);
	//terminalHandler(argc, argv);// get input from terminal
	startKeyboardHandling(argc, argv);

	return 0;
}


