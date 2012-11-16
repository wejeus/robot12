#include "PublishAmee.h"

using namespace amee;

namespace amee{


}//namespace amee


ros::Rate * loop_rate;

void printError(const char * m){
	printf("Bad number of arguments, should be: %s\n", m); fflush(stdout);
}

void wait(ros::Publisher &p){
	while(ros::ok() && p.getNumSubscribers() == 0)
		loop_rate->sleep();
}

int main(int argc, char **argv){
	if(argc < 2){
		puts("Usage:\n\t./PublishAmee forward 1.0");
		puts("or:\n\t./PublishAmee /serial/motor_speed roboard_drivers/Motor 0.2 0.3");
		return 0;
	}

	ros::init(argc, argv, "PublishAmee");
	ros::NodeHandle nodeHandle;
	ros::Publisher pub;
	loop_rate = new ros::Rate(10);

	
	if(ros::ok()){
		if(argc > 2 && strcmp(argv[2], "roboard_drivers/Motor") == 0){
			pub = nodeHandle.advertise<Motor>(argv[1], 1);
			if(argc < 5){ printError("left(f32) right(f32)"); return 0;}
			Motor m; m.left = atof(argv[3]); m.right = atof(argv[4]);
			wait(pub);
			pub.publish(m);
		}else if(argc > 2 && strcmp(argv[2], "amee/Velocity") == 0){
			pub = nodeHandle.advertise<Velocity>(argv[1], 1);
			if(argc < 5){ printError("right(f32) left(f32)"); return 0;}
			Velocity v; v.right = atof(argv[3]); v.left = atof(argv[4]);
			wait(pub);
			pub.publish(v);
		}else if(argc > 2 && strcmp(argv[2], "amee/MovementCommand") == 0){
			pub = nodeHandle.advertise<MovementCommand>(argv[1], 1);
			if(argc < 8){ printError("type(uint8) distance(f32) angle(f32) x(f32) y(f32)"); return 0;}
			MovementCommand mc;
			mc.type = atoi(argv[3]); mc.distance = atof(argv[4]); mc.angle = atof(argv[5]);
			mc.x = atof(argv[6]); mc.y = atof(argv[7]);
			wait(pub);
			pub.publish(mc);
		}else if(strcmp(argv[1], "forward") == 0){
			if(argc < 3){printError("forward [distance]"); return 0;}
			pub = nodeHandle.advertise<MovementCommand>("/MovementControl/MovementCommand", 1);
			MovementCommand mc;
			mc.type = 1; mc.distance = atof(argv[2]);
			wait(pub);
			pub.publish(mc);
		}else if(strcmp(argv[1], "rotate") == 0){
			if(argc < 3){printError("rotate [angle]"); return 0;}
            pub = nodeHandle.advertise<MovementCommand>("/MovementControl/MovementCommand", 1);
            MovementCommand mc;
            mc.type = 2; mc.angle = atof(argv[2]);
            wait(pub);
            pub.publish(mc);
        }else if(strcmp(argv[1], "point") == 0){
			if(argc < 4){printError("point [x y]"); return 0;}
            pub = nodeHandle.advertise<MovementCommand>("/MovementControl/MovementCommand", 1);
            MovementCommand mc;
            mc.type = 3; mc.x = atof(argv[2]); mc.y = atof(argv[3]);
            wait(pub);
            pub.publish(mc);
        }else if(strcmp(argv[1], "wall") == 0){
            pub = nodeHandle.advertise<MovementCommand>("/MovementControl/MovementCommand", 1);
            MovementCommand mc;
            mc.type = 4;
            wait(pub);
            pub.publish(mc);
        }else if(strcmp(argv[1], "stopwall") == 0){
            pub = nodeHandle.advertise<MovementCommand>("/MovementControl/MovementCommand", 1);
            MovementCommand mc;
            mc.type = 5;
            wait(pub);
            pub.publish(mc);
        }else if(strcmp(argv[1], "alignWall") == 0){
            pub = nodeHandle.advertise<MovementCommand>("/MovementControl/MovementCommand", 1);
            MovementCommand mc;
            mc.type = 6;
            wait(pub);
            pub.publish(mc);
        }else if(strcmp(argv[1], "alignFront") == 0){
            pub = nodeHandle.advertise<MovementCommand>("/MovementControl/MovementCommand", 1);
            MovementCommand mc;
            mc.type = 7;
            wait(pub);
            pub.publish(mc);
        }else if(strcmp(argv[1], "reset") == 0){
			pub = nodeHandle.advertise<Motor>("/serial/motor_speed", 1);
			Motor m; m.right = 0.0f; m.left = 0.0f;
			wait(pub);
			pub.publish(m);
        }else{
			ROS_INFO("Don't know %s", argv[2]); return 0;
		}

		//TODO we should wait till the message is delivered
		ros::spinOnce();
		loop_rate->sleep();
	}

	delete loop_rate;
	return 0;
}

