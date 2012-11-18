
#include <cv.h>
#include <highgui.h>
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/core/core.hpp"
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <fstream>
#include <ctype.h>
#include <string>
#include <unistd.h>


using namespace std;
using namespace cv;


void stream(VideoCapture &capture, VideoWriter &writer) {
   while (true) {
        
        Mat frame;

        if ( ! capture.grab()) { 
            cout << "Could not grab new frame!" << endl;
        } else {
            if (!capture.retrieve(frame)) {
                cout << "Could not decode and return new frame!" << endl;
            } else {
                //imshow("out", frame);
                writer << frame;
            }
        }
        
        // if (waitKey(10) > 0) {
        //     return;
        // }
    }
}

int main(int argc, char *argv[]) {
    
    //namedWindow("out", CV_WINDOW_AUTOSIZE);

    VideoCapture capture(CV_CAP_DSHOW);

    if (isdigit(*argv[1])) {
        capture.open(atoi(argv[1]));
    }
    
    if (!capture.isOpened()) {
        return -1;
    }

    // double fps = capture.get(CV_CAP_PROP_FPS);
    double fps = 2.0;
    Size frameSize = Size((int) capture.get(CV_CAP_PROP_FRAME_WIDTH), (int) capture.get(CV_CAP_PROP_FRAME_HEIGHT));
    // Size frameSize = Size(320,240);
    cout << fps << ":" << frameSize.width << ":" << frameSize.height << endl;

    VideoWriter writer;
    writer.open("out.avi", CV_FOURCC('M','J','P','G'), fps, frameSize, true);

    if (!writer.isOpened()) {
        return -1;
    }

    stream(capture, writer);

    cout << "done" << endl;
    writer.release();
    capture.release();

    return 0;
}
