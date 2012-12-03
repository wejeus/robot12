/*
 * CameraHack.h
 *
 *  Created on: Nov 22, 2012
 *      Author: Rasmus Göransson
 */

#ifndef CAMERAHACK_H_
#define CAMERAHACK_H_

#include <linux/videodev2.h>
#include <libv4l2.h>
#include <opencv2/opencv.hpp>

namespace cyclops
{

class CameraHack
{
public:
	CameraHack(uint cam, uint width, uint height);
	~CameraHack();

	// getFreshImg redirects the img to point directly into the fresh buffer.
	// This means that no copy is made and that the next call to getFreshImg
	// will overwrite the data. This also means that the image should be
	// destroyed or pointed somewhere else before the CameraHack instance is
	// destroyed. A clean copy of the image can be done with img.clone().
	bool getFreshImg(cv::Mat &img, uint32_t &sec, uint32_t &nsec);

protected:
	void startBuffering();
	bool sendIoctlRequest(unsigned long int request, void *arg);

	// Requested camera and resolution
	const int cam;
	const int width;
	const int height;

	// File descriptor to device
	int file_descriptor;

	// Buffer info and pointer to memory mapped buffer data
	v4l2_buffer buffer;
	void *buffer_data;

	// This image will actually point directly to the memory mapped buffer
	cv::Mat cv_hack;
	uchar *cv_hack_old_data;
};

} /* namespace cyclops */
#endif /* CAMERAHACK_H_ */
