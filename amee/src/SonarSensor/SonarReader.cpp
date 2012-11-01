#include "ros/ros.h"
#include "sta_msgs/Int32.h"
#include <stdio.d>
#include <unistd.h>


int main(int argc, char **argv){

	ros::init(argc, argv, "SonarReader");
	ros::NodeHandle nodeHandle;
	ros::Publisher sonarPub;
	sonarPub = nodeHandle.advertise<std_msgs::Int32>("/amee/sensors/sonardistance", 10);
	puts("Publishing on /amee/sensors/sonardistance");


	//init
	unsigned char buf[10];
	buf[0] = 0x00;  //byte 1: set SRF08 command register
	buf[1] = 0x51;  //byte 2: set ranging Mode - result in cm
	i2c_Send(0xe0>>1, buf, 2);  // write 2 bytes to SRF08 whose I2C addr = 0xe0 >> 1
	// Note: the SRF08 I2C address 0xe0 actually contains the I2C r/w bit, so we shift it to get the correct I2C address



	ros::Rate loop_rate(1000);
	struct timeval start, end;

	while (ros::ok()){
		gettimeofday(&start, NULL);
                double timestamp = start.tv_sec+double(start.tv_usec)/1000000.0;
                unsigned b1, b2;
                i2c0master_StartN(0xe0>>1, I2C_WRITE, 2);
                i2c0master_WriteN(0);
                i2c0master_WriteN(81);
                usleep(100000);
                i2c0master_StartN(0xe0>>1, I2C_WRITE, 1);
                i2c0master_SetRestartN(I2C_READ, 2);
                i2c0master_WriteN(2);       //set 1st SRF range register
                b1 = i2c0master_ReadN();    //read echo high byte
                b2 = i2c0master_ReadN(); //read echo low byte
                unsigned int dist = b1*256 + b2;
                printf("sonar: %f->%i\n",timestamp,dist);
		sonarPub.publish(dist);
                ros::spinOnce();
                gettimeofday(&end, NULL);
                while((end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec)) < sonar_interval*1000){
                        loop_rate.sleep();
                        gettimeofday(&end, NULL);
                }
                sleep_stop = false;
                ros::spinOnce();

	}
	i2c_Close();

	return 0;
}


