/*
 * CameraHack.cpp
 *
 *  Created on: Nov 22, 2012
 *      Author: Rasmus Göransson
 */

#include <linux/videodev2.h>
#include <libv4l2.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <sys/time.h>
#include <sys/mman.h>
#include <iostream>
#include <sstream>
#include <cstdio>

#include <opencv2/opencv.hpp>

#include "CameraHack.h"

namespace cyclops
{

CameraHack::CameraHack(const uint cam, const uint width, const uint height)
	: cam(cam)
	, width(width)
	, height(height)
	, buffer_data(0)
	, cv_hack_old_data(cv_hack.data)
{
	// Create device file name
	std::stringstream ss;
	ss << "/dev/video" << cam;

	// Open the camera in blocking mode
	file_descriptor = v4l2_open(ss.str().c_str(), O_RDWR /*| O_NONBLOCK*/, 0);

	if (file_descriptor < 0)
	{
		std::cerr << "CameraHack: Cannot open device" << std::endl;
		throw "CameraHack: Cannot open device";
	}

	// Set video format (width, height, pixel format)
	v4l2_format format;
	memset(&format, 0, sizeof(format));
	format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	format.fmt.pix.width = width;
	format.fmt.pix.height = height;
	format.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
	format.fmt.pix.field = V4L2_FIELD_INTERLACED;
	if (!sendIoctlRequest(VIDIOC_S_FMT, &format))
	{
		std::cerr << "CameraHack: Failed to set format" << std::endl;
		throw "CameraHack: Failed to set format";
	}

	if (format.fmt.pix.pixelformat != V4L2_PIX_FMT_BGR24)
	{
		std::cerr << "CameraHack::init v4l2 did not accept BGR24 format." << std::endl;
		throw "CameraHack::init v4l2 did not accept BGR24 format.";
	}

	if (format.fmt.pix.width != width || format.fmt.pix.height != height)
	{
		std::cerr << "CameraHack::init did not get desired image size. Got "
				<< format.fmt.pix.width << "x" << format.fmt.pix.height
				<< " instead of " << width << "x" << height << std::endl;
		throw "CameraHack: Failed to set resolution";
	}

	// Set buffer number of buffers to 1
	v4l2_requestbuffers requestbuffers;
	memset(&requestbuffers, 0, sizeof(requestbuffers));
	requestbuffers.count = 1;
	requestbuffers.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	requestbuffers.memory = V4L2_MEMORY_MMAP;
	if (!sendIoctlRequest(VIDIOC_REQBUFS, &requestbuffers))
	{
		std::cerr << "CameraHack: Failed to request buffers" << std::endl;
		throw "CameraHack: Failed to request buffers";
	}
	std::cout << "num buffers: " << requestbuffers.count << std::endl;

	// Setup buffer data and start streaming
	startBuffering();
}

CameraHack::~CameraHack()
{
	// Restore cv_hack
	cv_hack.data = cv_hack_old_data;

	// Unmap buffer
	if (buffer_data)
		v4l2_munmap(buffer_data, buffer.length);

	// Stop stream
	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	sendIoctlRequest(VIDIOC_STREAMOFF, &type);

	// Close file
	if (file_descriptor > 0)
		v4l2_close (file_descriptor);
}

void CameraHack::startBuffering()
{
	memset(&buffer, 0, sizeof(buffer));
	buffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	buffer.memory = V4L2_MEMORY_MMAP;
	buffer.index = 0;

	if (!sendIoctlRequest(VIDIOC_QUERYBUF, &buffer))
	{
		std::cerr << "CameraHack: Failed to query buffer" << std::endl;
		throw "CameraHack: Failed to query buffer";
	}

	// Memory map the buffer, this maps the buffer to where the kernel stores the image data
	buffer_data = v4l2_mmap(NULL, buffer.length,
			PROT_READ | PROT_WRITE, MAP_SHARED, file_descriptor, buffer.m.offset);

	if (MAP_FAILED == buffer_data)
	{
		std::cerr << "CameraHack: mmap error " << errno << ", " << strerror(errno) << std::endl;
		throw "CameraHack: mmap error";
	}

	// Hack the cv_image to point into the buffer
	cv_hack.create(cv::Size(width, height), CV_8UC3);
	cv_hack_old_data = cv_hack.data;
	cv_hack.data = (uchar*)buffer_data;

	// Start streaming
	v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	if (!sendIoctlRequest(VIDIOC_STREAMON, &type))
	{
		std::cerr << "CameraHack: Failed to start video stream" << std::endl;
		throw "CameraHack: Failed to start video stream";
	}
	return;
}

bool CameraHack::getFreshImg(cv::Mat &img, uint32_t &sec, uint32_t &nsec)
{
	// Add the buffer to the queue
	if (!sendIoctlRequest(VIDIOC_QBUF, &buffer))
		return false;

	// Wait for the buffer to get filled
	if (!sendIoctlRequest(VIDIOC_DQBUF, &buffer))
		return false;

	// Make the image point to our hacked image
	img = cv_hack;

	// Set timestamp
	sec = buffer.timestamp.tv_sec;
	nsec = buffer.timestamp.tv_usec*1000;

	return true;
}

bool CameraHack::sendIoctlRequest(unsigned long int request, void *arg)
{
	int ret = 0;

	// Keep requesting until we succeed or get an error
	do
		ret = v4l2_ioctl(file_descriptor, request, arg);
	while (ret == -1 && ((errno == EINTR) || (errno == EAGAIN)));

	if (ret == -1)
	{
		std::cerr << "CameraHack::sendIoctlRequest error " << errno << ", " << strerror(errno) << std::endl;
		return false;
	}

	return true;
}

} /* namespace cyclops */
