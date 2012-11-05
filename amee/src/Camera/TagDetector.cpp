
#include <cv.h>
#include <highgui.h>
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <ctype.h>
#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Int8MultiArray.h>


using namespace std;
using namespace cv;

void streamCamera(VideoCapture &capture);
void render(Mat &frame);
void initWindows();
void initROS(int argc, char *argv[]);
void initLocalInput(int argc, char *argv[]);
void filterRedTag(Mat &srcImg, Mat &destImg);
void onSaturationColorHighChange(int value);
void onSaturationColorLowChange(int value);
void onCannyHighChange(int value);
void onCannyLowChange(int value);
void *onButtonDoBlur(int state, void *pointer);

char windowResult[] = "result";
char windowThresh[] = "tresh";
int SATURATION_COLOR_HIGH = 139;
int SATURATION_COLOR_LOW = 83;
int CANNY_LOW = 77;
int CANNY_HIGH = 92;


// Flags
bool SMOOTH_IMAGE = true;
bool FILTER_RED_TAG = true;
bool EQUALIZE_HISTOGRAM = true;

void streamCamera(VideoCapture &capture) {
    Mat frame;

    while (true) {
        capture >> frame;
        render(frame);
        if(waitKey(30) >= 0) break;
    }
}

void render(Mat &frame) {
    // void cvSmooth(const CvArr* src, CvArr* dst, int smoothtype=CV_GAUSSIAN, int param1=3, int param2=0, double param3=0, double param4=0)
    if (SMOOTH_IMAGE) GaussianBlur(frame, frame, Size(7,7), 1.5, 1.5);
    if (FILTER_RED_TAG) filterRedTag(frame, frame);

    imshow(windowResult, frame);
    waitKey(3);
}

void initWindows() {
    namedWindow(windowResult, CV_WINDOW_AUTOSIZE);
    namedWindow(windowThresh, CV_WINDOW_AUTOSIZE);

    // int cvCreateButton(const char* button_name=NULL, CvButtonCallback on_change=NULL, void* userdata=NULL, int button_type=CV_PUSH_BUTTON, int initial_button_state=0 )
    // cvCreateButton("DoBlur", onButtonDoBlur, NULL, CV_CHECKBOX, 0); 

    // int cvCreateTrackbar(const char* trackbarName, const char* windowName, int* value, int count, CvTrackbarCallback onChange)
    cvCreateTrackbar("SaturationColorHigh", windowResult, &SATURATION_COLOR_HIGH, 360,  onSaturationColorHighChange);
    cvCreateTrackbar("SaturationColorLow", windowResult, &SATURATION_COLOR_LOW, 360,  onSaturationColorLowChange);
    cvCreateTrackbar("CannyHigh", windowResult, &CANNY_HIGH, 360,  onCannyHighChange);
    cvCreateTrackbar("CannyLow", windowResult, &CANNY_LOW, 360,  onCannyLowChange);
}

void filterRedTag(Mat &srcImg, Mat &destImg) {
    
    CvSize imgSize = srcImg.size();
    Mat hsvImg = cvCreateImage(imgSize, IPL_DEPTH_8U, 3);
    Mat binImg = cvCreateImage(imgSize, IPL_DEPTH_8U, 1);

    // void cvtColor(InputArray src, OutputArray dst, int code, int dstCn=0)
    cvtColor(srcImg, hsvImg, CV_RGB2HSV); // TODO: Change to BGR in ROS?
    
    // vector<cv::Mat> hsvChannels;
    // split(hsvImg, hsvChannels);
    // huePlane = hsvChannels[0];
    // if (EQUALIZE_HISTOGRAM) equalizeHist(hsvImg, hsvImg);

    // void inRange(InputArray src, InputArray lowerb, InputArray upperb, OutputArray dst)
    inRange(hsvImg, Scalar(SATURATION_COLOR_LOW, 100, 100), Scalar(SATURATION_COLOR_HIGH, 255, 255), binImg);
    
    // void Canny(InputArray image, OutputArray edges, double threshold1, double threshold2, int apertureSize=3, bool L2gradient=false )
    Canny(binImg, binImg, CANNY_LOW, CANNY_HIGH);
    imshow(windowThresh, binImg); // For debug output

    vector<vector<Point> > contours;
    // void findContours(InputOutputArray image, OutputArrayOfArrays contours, OutputArray hierarchy, int mode, int method, Point offset=Point())
    findContours(binImg, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    for(int i = 0; i < contours.size(); ++i) {
        RotatedRect rect = minAreaRect(contours[i]);
        if (contourArea(contours[i]) > 1000) {
            Point2f vtx[4];
            rect.points(vtx);
            for(int j = 0; j < 4; ++j) {
                line(destImg, vtx[j], vtx[(j+1)%4], Scalar(0, 255, 0), 1, CV_AA);
            }

            // if rect is in horizonal center of image:
            //     mark tag on map
        }
    }

}


void onSaturationColorHighChange(int value) {
    cout << "SATURATION_COLOR_HIGH: " << value << endl;
    SATURATION_COLOR_HIGH = value;
}

void onSaturationColorLowChange(int value) {
    cout << "SATURATION_COLOR_LOW: " << value << endl;
    SATURATION_COLOR_LOW = value;
}

void onCannyHighChange(int value) {
    cout << "CANNY_HIGH: " << value << endl;
    CANNY_HIGH = value;
}

void onCannyLowChange(int value) {
    cout << "CANNY_LOW: " << value << endl;
    CANNY_LOW = value;
}

void *onButtonDoBlur(int state, void *pointer) {
    printf("ok");
}

// Mat mat;

void cam0_cb(const std_msgs::Int8MultiArray::ConstPtr& array) {
    IplImage *img               = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);
    char * data                 = img->imageData;
    
    for(int i = 0; i < 320*240*3; i++)
    {
        data[i] = char(array->data.at(i));
    }
    // cvSaveImage("test.jpg" ,img);
    // cvReleaseImage(&img);
    // Mat mat = cvCreateMat(img->height,img->width,CV_32FC3 );
    // cvConvert(img, mat);
    Mat mat(img);
    render(mat);
    // Maybe release
}

void initROS(int argc, char *argv[]) {
    printf("Starting TagDetection using ROS\n");
    ros::init(argc, argv, "TagDetection");
    ros::NodeHandle n;
    ros::Subscriber img0_sub = n.subscribe("/camera0_img", 1, cam0_cb);
    // ros::Subscriber img1_sub = n.subscribe("/camera1_img", 1, cam1_cb);
    ros::spin();
}

void initLocalInput(int argc, char *argv[]) {

    cout << "Starting TagDetection using local input" << endl;

    VideoCapture capture;

    if (isdigit(*argv[2])) {
        cout << "Initializing camera: " << *argv[1] << endl;
        capture.open(atoi(argv[2]));
        if (!capture.isOpened()) {
            fprintf(stderr, "ERROR: capture is NULL\n");
            return;
        }
    }

    if (capture.isOpened()) {
        streamCamera(capture);
        // cvReleaseCapture(&capture);
    } else {
        Mat image = imread(argv[2], 1);
        render(image);
        waitKey();
    }
}

int main(int argc, char *argv[]) {

    initWindows();

    if (argc >= 2 && *argv[1] == 'l') {
        initLocalInput(argc, argv);
    } else {
        initROS(argc, argv);
    }

    cout << "Quitting ..." << endl;
    cvDestroyWindow(windowResult);
    return 0;
}

/* TODO's */
// Better camera capture pipeline (capture new frame then copy to next?)
// Tuning of parameters
// Get video stream from ROS
// Publish detected tag to map
// Try LAB color space instead of HSV

/* NOTES */

// OpenCV's RGB-to-HSV and HSV-to-RGB conversion only stores the Hue component as an 8-bit integer in the range 0 to 179, when it could have easily used 0 to 255. This means that if you use OpenCV's cvCvtColor() function to obtain a HSV image, you will loose some of the color resolution, since it is basically storing the Hue as a 7-bit number instead of an 8-bit number. Also, if you process this HSV image based on color values you found from other software or libraries, it is likely to cause you problems, since most systems use a range of 0 to 255. For example, if you are writing a skin detector and want to find some good HSV thresholds,

// When you see in the reference manual or in OpenCV source code a function that takes InputArray, it means that you can actually pass Mat, Matx, vector<T> etc. (see above the complete list).

// Do Opencv (current release) use 180 or 360 for hsv colors?