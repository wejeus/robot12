#include <linux/videodev2.h>
#include <libv4l2.h>
#include <cv.h>
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

#include "V4L2Camera.hh"

V4L2Camera::V4L2Camera()
{
  m_Fd = 0; // Make sure to initialze the file descriptor to 0
}

V4L2Camera::~V4L2Camera()
{
  disconnect();
}

// Custom Functions: Akshaya Thippur
// -----------------------------------------------------------------------------

std::string V4L2Camera::getCamType(){
  //const std::string CAMERATYPE = "/dev/video1";
	const std::string CAMERATYPE = "/dev/video0";
	return CAMERATYPE;
}

// **********************************************************************************************************
/*
cv::Mat V4L2Camera::getMatImg(){
	cv::Mat ImgMat(getIplImg());
	return ImgMat;
}
*/
char* V4L2Camera::getIplImg(){
	int index = grabBlocking();
	char* img = (char*) getDataBuffer(index)->data;
	//printf("index = %i\n",index);

	int w = m_Format.fmt.pix.width;
	int h = m_Format.fmt.pix.height;
	//printf("w: %i h: %i\n",w,h);

	//IplImage* frame = cvCreateImage(cvSize(w,h ), 8, 3);
/*
	frame->imageData       = img;
	return frame;
*/
	return img;
}

// **********************************************************************************************************

bool V4L2Camera::setCamRes(enum ImageSize is){
	disconnect();
	if(!connect(m_device))
		return false;
	return init(is, m_NumDataBuffers);
}

// **********************************************************************************************************
int V4L2Camera::getWidth(){
	int Temp   = m_Format.fmt.pix.width;
	return Temp;
}

// **********************************************************************************************************
int V4L2Camera::getHeight(){
	int Temp   = m_Format.fmt.pix.height;
	return Temp;
}

// ------------------------------------------------------------------------------

bool
V4L2Camera::sendIoctlRequest(unsigned long int request, void *arg)
{
  int ret = 0;

  do {
    ret = v4l2_ioctl(m_Fd, request, arg);
  } while (ret == -1 && ((errno == EINTR) || (errno == EAGAIN)));
  
  if (ret == -1) {
    fprintf(stderr, "V4L2Camera::sendIoctlRequest error %d, %s\n", 
	    errno, strerror(errno));
    return false;
  }

  return true;
}

// **********************************************************************************************************
bool
V4L2Camera::connect(const std::string &device)
{
	m_device = device;
  if (m_Fd > 0) {
    std::cerr << "V4L2Camera::connect ALREADY CONNECTED!!\n";
    return false;
  }

  m_Fd = v4l2_open(device.c_str(), O_RDWR | O_NONBLOCK, 0);
  if (m_Fd < 0) {
    perror("V4L2Camera::connect Cannot open device");
    return false;
  }

  return true;
}

// **********************************************************************************************************
bool
V4L2Camera::init(enum ImageSize is, int numBuffers)
{
  if (m_Fd <= 0) {
    std::cerr << "V4L2Camera::init NOT CONNECTED need to call connect\n";
    return false;
  }
  
  switch (is) {
  case SIZE_1600_1200:
    return init(1600,1200,numBuffers); 
  case SIZE_1400_1050:
    return init(1400,1050,numBuffers);
  case SIZE_1280_960:
    return init(1280,960,numBuffers); 
  case SIZE_1152_864:
    return init(1152,864,numBuffers);
  case SIZE_1024_768:
    return init(1024,768,numBuffers);
  case SIZE_800_600:
    return init(800,600,numBuffers);
  case SIZE_640_480:
    return init(640,480,numBuffers);
  case SIZE_320_240:
    return init(320,240,numBuffers);
  case SIZE_160_120:
  default:
    return init(160,120,numBuffers);
  }
}

// **********************************************************************************************************
bool
V4L2Camera::init(unsigned int width, unsigned int height, int numBuffers)
{
  memset(&m_Format, 0, sizeof(m_Format));
  m_Format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  m_Format.fmt.pix.width       = width;
  m_Format.fmt.pix.height      = height;
  m_Format.fmt.pix.pixelformat = V4L2_PIX_FMT_BGR24;
  m_Format.fmt.pix.field       = V4L2_FIELD_INTERLACED;
  if (!sendIoctlRequest(VIDIOC_S_FMT, &m_Format)) return false;

  if (m_Format.fmt.pix.pixelformat != V4L2_PIX_FMT_BGR24) {
    std::cerr << "V4L2Camera::init v4l2 did not accept BGR24 format.\n";
    return false;
  }

  if (m_Format.fmt.pix.width != width ||
      m_Format.fmt.pix.height != height) {
    std::cerr << "V4L2Camera::init did not get desired image size. Got "
	      << m_Format.fmt.pix.width << "x" 
	      << m_Format.fmt.pix.height << " instead of "
	      << width << "x" << height << std::endl;
    return false;
  }

  memset(&m_ReqBuffer, 0, sizeof(m_ReqBuffer));
  m_ReqBuffer.count = numBuffers;
  m_ReqBuffer.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  m_ReqBuffer.memory = V4L2_MEMORY_MMAP;
  if (!sendIoctlRequest(VIDIOC_REQBUFS, &m_ReqBuffer)) return false;

  m_DataBuffers = (DataBuffer*)calloc(m_ReqBuffer.count, sizeof(DataBuffer));
  m_NumDataBuffers = m_ReqBuffer.count;
  for (int i = 0; i < m_NumDataBuffers; i++) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (!sendIoctlRequest(VIDIOC_QUERYBUF, &buf)) return false;

    m_DataBuffers[i].length = buf.length;
    m_DataBuffers[i].data = v4l2_mmap(NULL, buf.length,
				       PROT_READ | PROT_WRITE, MAP_SHARED,
				       m_Fd, buf.m.offset);
    m_DataBuffers[i].width = m_Format.fmt.pix.width;
    m_DataBuffers[i].height = m_Format.fmt.pix.height;
    
    if (MAP_FAILED == m_DataBuffers[i].data) {
      perror("mmap");
      return false;
    }
  }

  for (int i = 0; i < m_NumDataBuffers; ++i) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (!sendIoctlRequest(VIDIOC_QBUF, &buf)) return false;
  }

  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;	  
  return sendIoctlRequest(VIDIOC_STREAMON, &type);
}

// **********************************************************************************************************

int
V4L2Camera::grabBlocking(double maxSecs)
{
  
  double t0 = getCurrentTime();
  int ret;
  do {
    fd_set fds;
    FD_ZERO(&fds);
    FD_SET(m_Fd, &fds);
    
    // Timeout. 
    struct timeval tv;
    double tLeft = maxSecs - (getCurrentTime() - t0);

    if (tLeft < 0) {
      std::cerr << "V4L2Camera:grabBlocking timed out\n";
      return -1;
    }
    tv.tv_sec = (int)tLeft;
    tv.tv_usec = (int)(1e6*(tLeft - tv.tv_sec));
    ret = select(m_Fd + 1, &fds, NULL, NULL, &tv);
  } while ((ret == -1 && (errno = EINTR)));

  if (ret == -1) {
    perror("V4L2Camera select");
    return errno;
  }

  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof(buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (!sendIoctlRequest(VIDIOC_DQBUF, &buf)) return -1;

  int index = buf.index;

  if (!sendIoctlRequest(VIDIOC_QBUF, &buf)) return -1;

  return index;
}

// **********************************************************************************************************
bool
V4L2Camera::writeBufferToPPM(int index, const std::string &filename)
{
	FILE *fout = fopen(filename.c_str(), "w");
	if (!fout) {
	 perror("V4L2Camera::writeBufferToPGM Cannot open file");
	 return false;
	}

	fprintf(fout, "P6\n%d %d 255\n",
	   m_Format.fmt.pix.width, m_Format.fmt.pix.height);
	fwrite(m_DataBuffers[index].data, m_Format.fmt.pix.sizeimage, 1, fout);
	fclose(fout);

	return true;
}

// **********************************************************************************************************

void
V4L2Camera::disconnect()
{
	if (m_Fd<=0) return;

	enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	sendIoctlRequest(VIDIOC_STREAMOFF, &type);

	for (int i = 0; i < m_NumDataBuffers; ++i){v4l2_munmap(m_DataBuffers[i].data, m_DataBuffers[i].length);}

	v4l2_close(m_Fd);
	m_Fd = -1;
}

// **********************************************************************************************************
