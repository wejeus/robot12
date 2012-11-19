#include <cv.h>
#include <highgui.h>
#include "ros/ros.h"

#include <std_msgs/Int32.h>
#include <std_msgs/Int8MultiArray.h>

#include <linux/videodev2.h>
#include <libv4l2.h>
#include <string.h>
#include <signal.h>
#include "V4L2Camera.cc"



const float max_speed 	= 0.03f;
int camera_interval 	= 1000;
int camera_width 	= 320;
int camera_height	= 240;

bool cam0_on		= false;
bool cam1_on		= false;
V4L2Camera cam0;
V4L2Camera cam1;

bool run = true;
void sigtrap(int) {run = false;}

void camera_change_freq(const std_msgs::Int32::ConstPtr& msg)
{
	ROS_INFO("camera change adc interval to %i", int(msg->data));
	camera_interval = int(msg->data);
}

/*
void change_camera_params(const CameraParams::ConstPtr &msg)
{
	sleep_stop = true;
	camera_interval = msg->interval;
	camera_width 	= msg->width;
	camera_height	= msg->height;
	printf("camera_interval: %i\n",camera_interval);
}
*/
int main(int argc, char **argv)
{
	printf("Starting CameraNode\n");
	ros::init(argc, argv, "CameraNode");
	ros::NodeHandle n;

	ros::Publisher img0_pub = n.advertise<std_msgs::Int8MultiArray>("/camera0_img", 10);
	ros::Publisher img1_pub = n.advertise<std_msgs::Int8MultiArray>("/camera1_img", 10);
	ros::Rate loop_rate(1000);

	cam0.connect("/dev/video0");
	if (cam0.init(V4L2Camera::SIZE_320_240, 1 )){
		cam0_on = true;
		printf("cam0 on\n");
	}else{printf("cam0 fail\n");}
	
	cam1.connect("/dev/video1");	
	if (cam1.init(V4L2Camera::SIZE_320_240, 1 )){
		cam1_on = true;
		printf("cam1 on\n");
	}else{printf("cam1 fail\n");}

	ros::Subscriber camera_sub	= n.subscribe("cameras_interval", 10, camera_change_freq);
	
	const bool debugg_send = false;
	struct timeval start, end;
	while(ros::ok() && run){
		
		gettimeofday(&start, NULL);
		double time = start.tv_sec+double(start.tv_usec)/1000000.0;
		printf("time: %f\n",time);
		if(cam0_on){
			if(debugg_send){printf("cam0.getIplImg()\n");}
			char * cam0_img = cam0.getIplImg();
			std_msgs::Int8MultiArray array;
			array.data.clear();
			int nr_elements = camera_width*camera_height*3;
			array.data.resize(nr_elements);
			for(int i = 0; i < nr_elements; i++)
			{
				array.data.at(i)=cam0_img[i];
			}
			img0_pub.publish(array);
		}
		if(cam1_on){
			if(debugg_send){printf("cam1.getIplImg()\n");}
			char * cam1_img = cam1.getIplImg();
			std_msgs::Int8MultiArray array;
			array.data.clear();
			int nr_elements = camera_width*camera_height*3;
			array.data.resize(nr_elements);
			for(int i = 0; i < nr_elements; i++)
			{
				array.data.at(i)=cam1_img[i];
			}
			img1_pub.publish(array);
		}
		
		ros::spinOnce();
		gettimeofday(&end, NULL);
		while(double(end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec)) < camera_interval*1000.0){
			loop_rate.sleep();
			gettimeofday(&end, NULL);
		}
		ros::spinOnce();
	}
	if(cam0_on){
		printf("cam0 off\n");
		cam0.disconnect();
		cam0_on = false;
	}
	if(cam1_on){
		printf("cam1 off\n");
		cam1.disconnect();
		cam1_on = false;
	}

	return 0;
}
