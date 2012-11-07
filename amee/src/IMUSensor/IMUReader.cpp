#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include "roboard_drivers/imu.h"
#include <string.h>

using namespace roboard_drivers;
ros::Subscriber	imu_sub;
ros::Publisher	int_pub;
ros::Publisher f_imu_pub;

void recive_imu(const imu::ConstPtr &msg)
{
	//printf("got encoder L:%i , R:%i\n",msg->left,msg->right);
	// printf("acc[%f,%f,%f] gyro[%f,%f,%f] magnetic[%f,%f,%f]\n",msg->acc_x,msg->acc_y,msg->acc_z,msg->gyro_x,msg->gyro_y,msg->gyro_z,msg->comp_x,msg->comp_y,msg->comp_z);
	f_imu_pub.publish(msg);
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "imuNodeTest");
	ros::NodeHandle n;
	imu_sub = n.subscribe("/imu", 1, recive_imu);
	f_imu_pub = n.advertise<imu>("/amee/sensors/imu", 10);
//	int_pub = n.advertise<std_msgs::Int32>("/imu_interval", 100000);

	ros::Rate loop_rate(1);
	struct timeval start, end;

	while(ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;
}
