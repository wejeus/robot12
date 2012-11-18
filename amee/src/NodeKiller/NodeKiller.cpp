#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <stdio.h>
#include <sys/types.h>
#include <signal.h>
#include <ctime>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string.h>

using namespace std;

void killNodes(const std_msgs::Int32::ConstPtr& msg){
	pid_t pid = (int)msg->data;

	ostringstream pName;
	pName << "/proc/" << pid << "/comm";

	ifstream file; file.open(pName.str().c_str(), ios::in);
	string out; file >> out; file.close();

	printf("killing %s (%d)...", out.c_str(), pid); fflush(stdout);
	if(kill(pid, 9) == 0)/* System call to kill with signal 9 */
		printf("%s killed!", out.c_str());
	else
		printf("could not kill %s!", out.c_str());
	puts("\n\n"); fflush(stdout);
}

int main(int argc, char ** argv){

	ros::init(argc, argv, "NodeKiller");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/amee/nodekiller", 20, &killNodes);
	ros::Rate loop_rate(10);

	puts("\nWaiting for publish on /amee/nodekiller (pid)");
	while(ros::ok()){
		ros::spinOnce();

		loop_rate.sleep();
	}

	puts("Quiting!");
	return 0;
}
