#include "PublishAmee.h"

using namespace amee;

namespace amee{


}//namespace amee


void printError(const char * m){
	printf("Bad number of arguments, should be: %s\n", m); fflush(stdout);
}

void wait(ros::Publisher &p){
	while(p.getNumSubscribers() == 0)
		usleep(200);
}

int main(int argc, char **argv){
	if(argc <= 3){
//		puts("Usage: ./PublishAmee /topic_name std_msgs/String hello");
		puts("Usage: ./PublishAmee /serial/motor_speed roboard_drivers/Motor 0.2 0.3");
		return 0;
	}

	ros::init(argc, argv, "PublishAmee");
	ros::NodeHandle nodeHandle;
	ros::Publisher pub;
	ros::Rate loop_rate = ros::Rate(10);

	
	if(ros::ok()){
		if(strcmp(argv[2], "roboard_drivers/Motor") == 0){
			pub = nodeHandle.advertise<Motor>(argv[1], 1);
			if(argc < 5){ printError("left(f32) right(f32)"); return 0;}
			Motor m; m.left = atof(argv[3]); m.right = atof(argv[4]);
			wait(pub);
			pub.publish(m);
		}else if(strcmp(argv[2], "amee/Velocity") == 0){
			pub = nodeHandle.advertise<Velocity>(argv[1], 1);
			if(argc < 5){ printError("right(f32) left(f32)"); return 0;}
			Velocity v; v.right = atof(argv[3]); v.left = atof(argv[4]);
			wait(pub);
			pub.publish(v);
		}else if(strcmp(argv[2], "amee/MovementCommand") == 0){
			pub = nodeHandle.advertise<MovementCommand>(argv[1], 1);
			if(argc < 8){ printError("type(uint8) distance(f32) angle(f32) x(f32) y(f32)"); return 0;}
			MovementCommand mc;
			mc.type = atoi(argv[3]); mc.distance = atof(argv[4]); mc.angle = atof(argv[5]);
			mc.x = atof(argv[6]); mc.y = atof(argv[7]);
			wait(pub);
			pub.publish(mc);
		}else{
			ROS_INFO("Don't know %s", argv[2]);
		}

		//TODO we should wait till the message is delivered
		ros::spinOnce();
		loop_rate.sleep();
	}

	return 0;
}


