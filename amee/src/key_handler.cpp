#include "key_handler.h"

using namespace amee;

namespace amee{


	/**
	 * This is for handling the nonblocking key presses
	 **/
	inline int KeyHandler::kbhit(bool put_char_back = true) const{
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
			if(put_char_back)
				ungetc(ch, stdin);
			return 1;
		}
	 
		return 0;
	}

	/**
	 * Sets the wheel velocities to given values
	 **/
	inline void KeyHandler::setWheels(Motor &motor, const float L=0.0, const float R=0.0) const{
		motor.left  = L;
		motor.right = R;
	}

	/**
	 * Gets input from keyboard and publishes it to "wheel_velocities" topic.
	 *
	 * Use the keyboard arrows to navigate the robot.
	 **/
	void KeyHandler::handleKeyStroke() const{
		char c = '0';
		if(kbhit()){
			c = getchar();
			int ascii_i = int(c);
			if(ascii_i == 27 || ascii_i == 32 || ascii_i == LEFT_YAW || ascii_i == RIGHT_YAW){//if we hit an arrow key or space (or an escape key)
				Motor motor;
				if(ascii_i == 27){//if it is an escape key
					c = getchar(); c = getchar(); //read 2 more chars
					ascii_i = int(c);
				}

				switch(ascii_i){
					case KEYCODE_L:
						ROS_DEBUG("LEFT");
						setWheels(motor, VELO_RATE_L, -VELO_RATE_R);
						break;
					case KEYCODE_R:
						ROS_DEBUG("RIGHT");
						setWheels(motor, -VELO_RATE_L, VELO_RATE_R);
						break;
					case KEYCODE_U:
						ROS_DEBUG("UP");
						setWheels(motor, -VELO_RATE_L, -VELO_RATE_R);
						break;
					case KEYCODE_D:
						ROS_DEBUG("DOWN");						
						setWheels(motor, VELO_RATE_L, VELO_RATE_R);
						break;
					case LEFT_YAW:
						ROS_DEBUG("LEFT_YAW");						
						setWheels(motor, 0, -VELO_RATE_R);
						break;
					case RIGHT_YAW:
						ROS_DEBUG("RIGHT_YAW");						
						setWheels(motor, -VELO_RATE_L, 0);
						break;
					default:
						ROS_DEBUG("RELEASE");
						setWheels(motor);
						break;
				}
				keyCom_pub.publish(motor);
			}
		}
	}

	void KeyHandler::setKeyComPublisher(ros::Publisher & pub){keyCom_pub = pub;}

	/**
	 * Sets the velocity rates to the default values
	 **/
	void KeyHandler::initDefVeloRate(){
		VELO_RATE_L = DEF_VELO_RATE_L;
		VELO_RATE_R = DEF_VELO_RATE_R;
	}

	/**
	 *
	 **/
	void KeyHandler::setVeloRate(float left, float right){
		VELO_RATE_L = -left;
		VELO_RATE_R = -right;
	}

	inline const float & KeyHandler::getVeloRateL() const{return VELO_RATE_L;}
	inline const float & KeyHandler::getVeloRateR() const{return VELO_RATE_R;}

}//namespace amee




int main(int argc, char **argv){
	KeyHandler keyHandler;
	
	if(argc > 1){//if only one velocity is given, set both motors to given value
		float velo_rate = atof(argv[1]);
		keyHandler.setVeloRate(velo_rate, velo_rate);
	}else if(argc > 2){//if both velocities are given, set each to the corresponding value
		float velo_rate_L = atof(argv[1]);
		float velo_rate_R = atof(argv[2]);
		keyHandler.setVeloRate(velo_rate_L, velo_rate_R);
	}else{//no value is given so set the volicities to the default values
		keyHandler.initDefVeloRate();
	}

	printf("VELO_RATE is set to: [%0.2f, %0.2f]\n", keyHandler.getVeloRateL(), keyHandler.getVeloRateR());

	ros::init(argc, argv, "KeyHandler");
	ros::NodeHandle nodeHandle;
	ros::Publisher keyCom_pub = nodeHandle.advertise<Motor>("/serial/motor_speed", 1);
	keyHandler.setKeyComPublisher(keyCom_pub);

	ros::Rate loop_rate = ros::Rate(6);

	puts("Press the arrow keys to move around.");
	while (ros::ok()){

		keyHandler.handleKeyStroke();
		ros::spinOnce();

		while(keyHandler.kbhit(false));//clear the rest
		loop_rate.sleep();
	}

	return 0;
}


