
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

#define ROS

#ifdef ROS
#include "ros/ros.h"
#include <std_msgs/Int32.h>
#include <std_msgs/Int8MultiArray.h>
#include "amee/Tag.h"
#include "amee/MovementCommand.h"
#include <ros/console.h>
using namespace amee;
#endif

using namespace std;
using namespace cv;

enum TAG_CLASS {FAILURE, UNKNOWN, APPLE, BANANA, BOOK, CAMERA, DRYER, GLASS, GLASSES, HAMMER, LAPTOP, MUG, SCISSORS, TEDDY};
enum COLOR {UNKNOWN_COLOR, BLACK, GREEN, BLUE, MAGENTA};

// Flags
bool SMOOTH_IMAGE = false;
bool FILTER_RED_TAG = true;
bool EQUALIZE_HISTOGRAM = true;
bool DISPLAY_GRAPHICAL = false;
bool LOCAL = false;
int CAMERA_INTERVAL = 10;
bool USE_TEMPLATES = false;
bool USE_SURF = false;
bool USE_ROS_CAMERA = false;
bool INIT_ROS = false;
bool CLASSIFICATION_IN_PROGRESS = false;

struct TagData {
    TAG_CLASS object;
    Mat image;
    vector<KeyPoint> keypoints;  
    Mat descriptors;
};

struct TagTemplate {
    TAG_CLASS object;
    Mat image;
};

void streamCamera(VideoCapture &capture);
void render(Mat &frame);
void initWindows();
void show(const string& winname, InputArray mat);
void log(string fmt, ...);
string class2name(TAG_CLASS object);
TAG_CLASS name2class(string name);
string color2string(COLOR object);
void drawText(Mat &image, string text);

void initROS(int argc, char *argv[]);
void publishMovement(int state);
void initLocalInput(int argc, char *argv[]);

void onSaturationColorHighChange(int value);
void onSaturationColorLowChange(int value);
void onCannyHighChange(int value);
void onCannyLowChange(int value);
void onGaussianKernelChange(int value);
void onGaussianSigmaChange(int value);



bool findTagROI(Mat &srcImage, Mat &ROI);
int thresholdRedPixels(Mat &srcImage, Mat &destImage);
bool containsRedBorderPixels(Mat &srcImage);
bool containsRedPixels(Mat &srcImage);
bool prepareROI(Mat &srcROI, Mat &destROI);
COLOR determineTagColor(Mat &ROI);
void thresholdBlackWhite(Mat &srcImage, Mat &destImage);

bool initSURF();
TAG_CLASS classifyTagUsingSURF(Mat &ROI);

bool initTemplates();
TAG_CLASS classifyTagUsingTemplate(Mat &ROI);

char windowResult[] = "result";
char windowThresh[] = "tresh";
char windowTag[] = "tag";
int SATURATION_COLOR_HIGH = 20;
int SATURATION_COLOR_LOW = 0;
int CANNY_LOW = 77;
int CANNY_HIGH = 92;
int GAUSSIAN_KERNEL_SIZE = 7;
int GAUSSIAN_SIGMA = 1.5;
double TAG_MIN_PIXEL_AREA = 1500.0;
char TAG_FILES[] = "tags/files.txt";
int minHessian = 200;
Mat frame;

vector<TagData> sourceTags;
vector<TagTemplate> sourceTemplates;

#ifdef ROS
ros::Publisher mapPublisher;
ros::Publisher movementPublisher;
#endif


void render(Mat &frame) {
    // if (SMOOTH_IMAGE) GaussianBlur(frame, frame, Size(GAUSSIAN_KERNEL_SIZE, GAUSSIAN_KERNEL_SIZE), GAUSSIAN_SIGMA, GAUSSIAN_SIGMA);
    // if (FILTER_RED_TAG) filterRedTag(frame, frame);
    
    if (containsRedPixels(frame)) {
        
        CLASSIFICATION_IN_PROGRESS = true;
        log("Found some object, classification in progress...\n");
        publishMovement(5); // Stop motors

        Mat ROI;
        TAG_CLASS res;

        if (findTagROI(frame, ROI) && prepareROI(ROI, ROI)) {
            // TODO: test if ROI is "good enough" to make a classification (is in center, sharp?)

            
            // TODO: move robot so that tag is in center (1. determine distance from tag center to image normal 2. move(distance))
            
            if (USE_TEMPLATES) res = classifyTagUsingTemplate(ROI);
            if (USE_SURF) res = classifyTagUsingSURF(ROI);


            log("Color: %s, ", color2string(determineTagColor(ROI)).c_str());

            if (res != FAILURE) {
                log("Object: %s\n", class2name(res).c_str());
            }
            
            publishMovement(4); // continue wall following
        }

        log("CLASSIFICATION DONE!\n");
        CLASSIFICATION_IN_PROGRESS = false;        


        
    }

    show(windowResult, frame);
}


/* Thresholds redpixels and saves resulting binary image in destImage, returns number of red pixels in image */
int thresholdRedPixels(Mat &srcImage, Mat &destImage) {
    Mat hsvImage, firstRedSection, secondRedSection;
    cvtColor(srcImage, hsvImage, CV_BGR2HSV);
    
    // Works very good if the red in the tag is not dark. If it's in a position to reflect a little light directly it works very good.
    inRange(hsvImage, Scalar(0, 100, 0), Scalar(40, 255, 255), firstRedSection);
    inRange(hsvImage, Scalar(129, 100, 0), Scalar(179, 255, 255), secondRedSection);
    bitwise_or(firstRedSection, secondRedSection, destImage);

    int numRedPixels = countNonZero(destImage);
    // log("numRedPixels: %d\n", numRedPixels);

    return numRedPixels;
}

bool containsRedPixels(Mat &srcImage) {
    Mat dummy;
    int numRedPixels = thresholdRedPixels(srcImage, dummy);
    // log("Num red pixels in image: %d\n", numRedPixels);
    if (numRedPixels > 500) {
        return true;
    } else {
        return false;
    }
}

bool containsRedBorderPixels(Mat &srcImage) {
    // TODO: remove center part of image and threshold. That would give a good estimate if the (potential) red pixels are located on the border.
    return containsRedPixels(srcImage);
}

/* Looks for a red rectangle of a certain size in the image */
bool findTagROI(Mat &srcImage, Mat &ROI) {

    // GaussianBlur(srcImage, srcImage, Size(GAUSSIAN_KERNEL_SIZE, GAUSSIAN_KERNEL_SIZE), GAUSSIAN_SIGMA, GAUSSIAN_SIGMA);
    Mat binaryImage;
    thresholdRedPixels(srcImage, binaryImage);

    int erosionSize = 2;
    Mat element = getStructuringElement(MORPH_ELLIPSE, Size(2*erosionSize + 1, 2*erosionSize+1));
    erode(binaryImage, binaryImage, element);
    
    Canny(binaryImage, binaryImage, CANNY_LOW, CANNY_HIGH);

    // dilate(binaryImage, binaryImage, element);

    vector< vector<Point> > contours;
    findContours(binaryImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);
    show(windowThresh, binaryImage);

    // Filter out contours that are big enough to hold a tag rectangle
    for(int i = 0; i < contours.size(); ++i) {

        if (contourArea(contours[i]) > TAG_MIN_PIXEL_AREA) {
            // log("Area: %d", contourArea(contours[i]));
            Rect rect = boundingRect(contours[i]);
            // Make sure found ROI do not exeed image boundaries
            if (0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= srcImage.cols && 0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= srcImage.rows) {
                rectangle(srcImage, rect, Scalar(255,0,0), 3);
                ROI = Mat(srcImage, rect);
                return true;
            }
        }
    }

    return false;
}

/* Tries to remove as much as possible of tag borders (if any) and resizes the ROI to a predefined resolution */
// TODO: Adjust size of ROI to match template, maybe rotate/skew to (not sure if possible to make a good estimate..?)
bool prepareROI(Mat &srcROI, Mat &destROI) {

    if (containsRedBorderPixels(srcROI)) {
        // resize to known size with borders
        // crop border, keep center
        // Given know size of tag (with borders) the border of a tag constitue ~13% of the image tag
        float borderRatio = 0.12965957f;
        int width = srcROI.cols;
        int height = srcROI.rows;
        int borderSizeX = width*borderRatio;
        int borderSizeY = height*borderRatio;
        Rect nonBorderROI(borderSizeX, borderSizeY, width-(2*borderSizeX), height-(2*borderSizeY));

        // Check for correct dimensions (new ROI smaller than original)
        if (0 <= nonBorderROI.x && 0 <= nonBorderROI.width && nonBorderROI.x + nonBorderROI.width <= srcROI.cols && 0 <= nonBorderROI.y && 0 <= nonBorderROI.height && nonBorderROI.y + nonBorderROI.height <= srcROI.rows) {
            Mat croppedROI = srcROI(nonBorderROI);
            resize(croppedROI, destROI, Size(200, 200), 0, 0, INTER_LINEAR);
        } else {
            log("Warning: invalid dimensions of ROI when classifying.\n");
            return false;
        }
    } else {
        resize(srcROI, destROI, Size(200, 200), 0, 0, INTER_LINEAR);
    }

    return true;
}

struct PixelSum {
    COLOR color;
    int numPixels;
};

// TODO: Adjust color params to better match colors.
COLOR determineTagColor(Mat &ROI) {
    // Mat r,g,b;
    // vector<Mat> bgr;
    // split(ROI, bgr);
    // log("B %d G %d R %d\n", countNonZero(bgr[0]), countNonZero(bgr[1]), countNonZero(bgr[2]));
    // return UNKNOWN_COLOR;

    Mat hsvImage, binaryImage;
    cvtColor(ROI, hsvImage, CV_BGR2HSV);
    int numColoredPixels;
    vector<PixelSum> elems;
    PixelSum ps;

    // Green
    inRange(hsvImage, Scalar(60, 50, 20), Scalar(90, 255, 255), binaryImage);
    ps.numPixels = countNonZero(binaryImage);
    ps.color = GREEN;
    elems.push_back(ps);

    // Blue
    inRange(hsvImage, Scalar(110, 100, 20), Scalar(130, 255, 255), binaryImage);
    ps.numPixels = countNonZero(binaryImage);
    ps.color = BLUE;
    elems.push_back(ps);

    // Magenta
    inRange(hsvImage, Scalar(150, 0, 0), Scalar(165, 255, 255), binaryImage);
    ps.numPixels = countNonZero(binaryImage);
    ps.color = MAGENTA;
    elems.push_back(ps);

    PixelSum bestMatch;
    bestMatch.color = UNKNOWN_COLOR;
    bestMatch.numPixels = 0;
    for (vector<PixelSum>::iterator it = elems.begin(); it < elems.end(); ++it) {
        log("COLOR: %s VALUE: %d\n", color2string(it->color).c_str(), it->numPixels);
        if (it->numPixels > bestMatch.numPixels) {
            bestMatch.color = it->color;
            bestMatch.numPixels = it->numPixels;
        }
    }

    // if (bestMatch.numPixels < 1000) {
    //     bestMatch.color = BLACK;
    // } else if (elems[0].numPixels > 1000 && elems[1].numPixels > 1000) {
    //     log("asdfasdf\n");
    //     bestMatch.color = BLACK;
    // }

    // Debug: adjustment of saturation params.
    inRange(hsvImage, Scalar(SATURATION_COLOR_LOW, 100, 0), Scalar(SATURATION_COLOR_HIGH, 255, 255), binaryImage);
    log("Match: %d\n", countNonZero(binaryImage));
    show(windowTag, binaryImage);

    return bestMatch.color;
}

bool initTemplates() {
    log("Init templates...\n");

    ifstream infile;

    // Ptr<FeatureDetector>& featureDetector
    SurfFeatureDetector detector(minHessian);
    SurfDescriptorExtractor extractor;

    infile.open(TAG_FILES);
    std::string line;
    if (infile.is_open()) {
        while (std::getline(infile, line)) {
            TagTemplate tag;
            tag.object = name2class(line);
            tag.image = imread(line, CV_LOAD_IMAGE_GRAYSCALE);
    
            resize(tag.image, tag.image, Size(200, 200), 0, 0, INTER_LINEAR);
            thresholdBlackWhite(tag.image, tag.image);
            
            // Debug
            // show(windowTag, tag.image);
            // waitKey();

            if( ! tag.image.data) {
                log("Image not found!\n");
                return false;
            }

            sourceTemplates.push_back(tag);

            log("Read template: %s\n", line.c_str());
        }
    } else {
        log("Could not read tag filenames!\n");
        return false;
    }

    infile.close();
    return true;
}

void thresholdBlackWhite(Mat &srcImage, Mat &destImage) {
    threshold(srcImage, destImage, 128, 255,THRESH_BINARY);
}

/* Assumes borders have been removed */
TAG_CLASS classifyTagUsingTemplate(Mat &ROI) {
    
    Mat tmpROI;    
    cvtColor(ROI, tmpROI, CV_BGR2GRAY);
    thresholdBlackWhite(tmpROI, tmpROI);

    double bestMatch = 100;
    TAG_CLASS obj;

    for (vector<TagTemplate>::iterator it = sourceTemplates.begin(); it < sourceTemplates.end(); ++it) {
        Mat result;
        matchTemplate(tmpROI, it->image, result, CV_TM_SQDIFF_NORMED);
        double min, max;
        minMaxLoc(result, &min, &max);
        // log("%s Min: %f Max: %f\n", class2name(it->object).c_str(), min, max);
        if (min < bestMatch) {
            bestMatch = min;
            obj = it->object;
        }

    }

    log("bestMatch: %f\n", bestMatch);
    return obj;
}

bool initSURF() {
    log("Init surf...\n");

    ifstream infile;

    // Ptr<FeatureDetector>& featureDetector
    SurfFeatureDetector detector(minHessian);
    SurfDescriptorExtractor extractor;

    infile.open(TAG_FILES);
    std::string line;
    if (infile.is_open()) {
        while (std::getline(infile, line)) {
            TagData tag;
            tag.object = name2class(line);
            tag.image = imread(line, CV_LOAD_IMAGE_GRAYSCALE);
    
            if( ! tag.image.data) {
                log("Image not found!\n");
                return false;
            }

            //-- Step 1: Detect the keypoints using SURF Detector            
            detector.detect(tag.image, tag.keypoints);

            //-- Step 2: Calculate descriptors (feature vectors)
            extractor.compute(tag.image, tag.keypoints, tag.descriptors);
            sourceTags.push_back(tag);

            log("Classified tag: %s keypoints: %d\n", line.c_str(), tag.keypoints.size());
        }
    } else {
        log("Could not read tag filenames!\n");
        return false;
    }

    infile.close();

    // Show keypoints in objects
    // for (vector<TagData>::iterator it = sourceTags.begin(); it < sourceTags.end(); ++it) {
    //     for (vector<KeyPoint>::iterator it2 = it->keypoints.begin(); it2 < it->keypoints.end(); it2++) {
    //         KeyPoint keyPoint = *it2;
    //         CvPoint center;
    //         int radius;
    //         center.x = cvRound(keyPoint.pt.x);
    //         center.y = cvRound(keyPoint.pt.y);
    //         radius = cvRound(keyPoint.size*1.2/9.*2);
            
    //         circle(it->image, center, radius, Scalar(255, 0, 0), 2);
    //     }
    //     show(windowThresh, it->image);
    //     log("File: %s keypoints: %d \n", it->filename.c_str(), it->keypoints.size());
    //     waitKey();
    // }


    return true;
}


TAG_CLASS classifyTagUsingSURF(Mat &ROI) {

    Mat binaryImage;
    cvtColor(ROI, binaryImage, CV_BGR2GRAY); 

    //-- Step 1: Detect the keypoints using SURF Detector
    SurfFeatureDetector detector(minHessian);
    vector<KeyPoint> ROIKeypoints;
    detector.detect(binaryImage, ROIKeypoints);

    // If no good keypoints could be found we cant relly do anything interesting.
    // This usually happens if object has a lot of movement in the captured frame.
    if (ROIKeypoints.size() < 2) {
        log("Not enought keypoints in frame..\n");
        return FAILURE;
    } else {
        log("Interesting keypoints in frame: %d\n", ROIKeypoints.size());
    }

    //-- Step 2: Calculate descriptors (feature vectors)
    SurfDescriptorExtractor extractor;
    Mat ROIDescriptors;
    extractor.compute(binaryImage, ROIKeypoints, ROIDescriptors);

    //-- Step 3: Matching descriptor vectors with a brute force matcher
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
    
    // log("---------------\n");

    // Compare against precomputed tags
    double bestMatch = 100.0f;
    TAG_CLASS determinedClass = UNKNOWN;

    for (vector<TagData>::iterator it = sourceTags.begin(); it < sourceTags.end(); ++it) {
        Mat tagImage = it->image;
        vector<KeyPoint> tagKeypoints = it->keypoints;
        Mat tagDescriptors = it->descriptors;
        
        vector< DMatch > matches;
        matcher->match(tagDescriptors, ROIDescriptors, matches);

        // vector< vector< DMatch > > matches;
        // matcher->knnMatch(tagDescriptors, descriptors, matches, 3);

        double max_dist = 0;
        double min_dist = 100;
        //-- Quick calculation of max and min distances between keypoints
        for( int i = 0; i < tagDescriptors.rows; i++ ) { 
            double dist = matches[i].distance;
            if( dist < min_dist ) min_dist = dist;
            if( dist > max_dist ) max_dist = dist;
        }

        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        vector< DMatch > goodMatches;
        double error = 0;
        for( int i = 0; i < ROIDescriptors.rows; i++ ) {
            if( matches[i].distance < 3*min_dist ) {
                goodMatches.push_back(matches[i]);
                error += matches[i].distance;
            }
        }

        // log("%s error: %f\n", it->filename.c_str(), (error/(float)goodMatches.size()));
        
        // if ((error/(float)goodMatches.size()) < bestMatch) {
        //     bestMatch = (error/(float)goodMatches.size());
        //     determinedClass = it->object;
        // }

        //-- Localize the object
        vector<Point2f> obj;
        vector<Point2f> scene;

        // Get the keypoints from the good matches
        for( int i = 0; i < goodMatches.size(); i++ ) {
            obj.push_back(tagKeypoints[goodMatches[i].queryIdx].pt);
            scene.push_back(ROIKeypoints[goodMatches[i].trainIdx].pt);
        }

        Mat H = findHomography(obj, scene, CV_RANSAC);

        //-- Get the corners from the image_1 ( the object to be "detected" )
        vector<Point2f> obj_corners(4);
        obj_corners[0] = cvPoint(0,0);
        obj_corners[1] = cvPoint( tagImage.cols, 0 );
        obj_corners[2] = cvPoint( tagImage.cols, tagImage.rows );
        obj_corners[3] = cvPoint( 0, tagImage.rows );
        vector<Point2f> scene_corners(4);

        Mat transformedROI;
        perspectiveTransform(binaryImage, transformedROI, H);
        show(windowTag, binaryImage);
        
        if (containsRedBorderPixels(transformedROI)) {
            // resize to known size with borders
            // crop border, keep center
            // Given know size of tag (with borders) the border of a tag constitue ~13% of the image tag
            float borderRatio = 0.12965957f;
            int width = ROI.cols;
            int height = ROI.rows;
            int borderSizeX = width*borderRatio;
            int borderSizeY = height*borderRatio;
            Rect nonBorderROI(borderSizeX, borderSizeY, width-(2*borderSizeX), height-(2*borderSizeY));

            // log("width: %d (%d) height: %d (%d) \n", ROI.cols, width, ROI.rows, height);
            // log("x: %d y: %d width: %d height: %d \n", nonBorderROI.x, nonBorderROI.y, nonBorderROI.width, nonBorderROI.height);

            // Check for correct dimensions (new ROI smaller than original)
            if (0 <= nonBorderROI.x && 0 <= nonBorderROI.width && nonBorderROI.x + nonBorderROI.width <= transformedROI.cols && 0 <= nonBorderROI.y && 0 <= nonBorderROI.height && nonBorderROI.y + nonBorderROI.height <= transformedROI.rows) {
                Mat croppedROI = ROI(nonBorderROI);
                resize(croppedROI, transformedROI, Size(200, 200), 0, 0, INTER_LINEAR);
            } else {
                log("Warning: invalid dimensions of ROI when classifying.\n");
                return UNKNOWN;
            }
            
            } else {
                resize(transformedROI, transformedROI, Size(200, 200), 0, 0, INTER_LINEAR);
            }


            // TODO (maybe): adjust histogram of ROI, extract and replace background with white color
        
            cvtColor(transformedROI, transformedROI, CV_BGR2GRAY);

            Mat result;
            matchTemplate(transformedROI, tagImage, result, CV_TM_SQDIFF_NORMED);
            double min, max;
            minMaxLoc(result, &min, &max);
            // log("%s Min: %f Max: %f\n", class2name(it->object).c_str(), min, max);
            if (min < bestMatch) {
                bestMatch = min;
                determinedClass = it->object;
            }
        }

    return determinedClass;
}

#ifdef ROS
void cam0_cb(const std_msgs::Int8MultiArray::ConstPtr& array) {
    log("Got frame from ROS\n");

    if ( ! CLASSIFICATION_IN_PROGRESS) {
        IplImage *img               = cvCreateImage(cvSize(320, 240), IPL_DEPTH_8U, 3);
        char * data                 = img->imageData;

        for(int i = 0; i < 320*240*3; i++) {
            data[i] = char(array->data.at(i));
        }

        // cvSaveImage("test.jpg" ,img);
        // cvReleaseImage(&img);
        // Mat mat = cvCreateMat(img->height,img->width,CV_32FC3 );
        // cvConvert(img, mat);
        Mat mat(img);
        render(mat);
        // waitKey(10); // For some reason we have to wait to get the graphical stuff to show..?
    }
}
#endif

void initROS(int argc, char *argv[]) {
    #ifdef ROS
    log("Initializing ROS...\n");
    ros::init(argc, argv, "TagDetection");
    ros::NodeHandle rosNodeHandle;

    mapPublisher = rosNodeHandle.advertise<Tag>("/amee/tag", 100);
    // wait(mapPublisher);
    movementPublisher = rosNodeHandle.advertise<MovementCommand>("/MovementControl/MovementCommand", 1);
    // wait(movementPublisher);

    if (USE_ROS_CAMERA) {
        ros::Subscriber img0_sub = rosNodeHandle.subscribe("/camera0_img", 1, cam0_cb);
        log("done. Spinning...\n");
        ros::spin();
    } 
    // ros::Subscriber img1_sub = n.subscribe("/camera1_img", 1, cam1_cb);

    
    
    #else
    log("ROS IS NOT DEFINED");
    #endif
}

void initLocalInput(string source) {

    log("Starting TagDetection using local input.\n");

    if (!strcmp(source.c_str(), "")) {
        log("No input source!\n");
        return;
    }

    VideoCapture capture;
    // capture.set(CV_CAP_PROP_FPS, 1.0);

    if (isdigit(*source.c_str())) {
        log("Initializing camera: %s\n", source.c_str());
        capture.open(atoi(source.c_str()));
        // bool VideoCapture::set(int propId, double value)
        
        // cout << capture.set(CV_CAP_PROP_FRAME_WIDTH, 320);
        // cout << capture.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
        
        if (!capture.isOpened()) {
            log("ERROR: capture is NULL\n");
            return;
        }
    }

    if (capture.isOpened()) {
        streamCamera(capture);
        // cvReleaseCapture(&capture);
    } else {
        log("Reading image: %s\n", source.c_str());
        Mat image = imread(source.c_str(), 1);
        render(image);
        waitKey();
    }
}

int main(int argc, char *argv[]) {

    int c;
    string source;

    if (argc == 1) {
        cout << "Options (default):" << endl
             << "l (false) - use local camera on system" << endl
             << "t (false) - loads and classifies using TEMPLATES" << endl
             << "k (false) - loads and classifies using SURF" << endl
             << "c (false) - use camera data using callback listening to rostopic" << endl
             << "d (false) - display results graphically" << endl
             << "r (false) - init ROS" << endl
             << "s (nothing) - which camera source to listen to when using local camera (/dev/videoX)" << endl;
             return 0;
    }

    while ((c = getopt (argc, argv, "ltrckds:")) != -1) {
        switch (c) {
            case 'l':
                LOCAL = true;
                break;
            case 'd':
                DISPLAY_GRAPHICAL = true;
                break;
            case 't':
                USE_TEMPLATES = true;     
                break;
            case 'k':
                USE_SURF = true;     
                break;
            case 'c':
                USE_ROS_CAMERA = true;     
                break;
            case 'r':
                INIT_ROS = true;     
                break;
            case 's':
                source = optarg;     
                break;
            case '?':
                if (optopt == 's')
                    log("Option -%c requires an argument. Image or camera.\n", optopt);
                else if (isprint (optopt))
                    log("Unknown option `-%c'.\n", optopt);
                else
                    log("Unknown option character `\\x%x'.\n", optopt);
                return 1;
            default:
                abort();
        }
    }

    if (DISPLAY_GRAPHICAL) {
        log("Will do graphical display\n");
        initWindows();
    }

    if (USE_TEMPLATES) {
        if (!initTemplates()) {
            log("Failed to initialize templates!\n");
            return -1;
        }
    }

    if (USE_SURF) {
        if (!initSURF()) {
            log("Failed to initialize SURF tags!\n");
            return -1;
        }
    }

    if (INIT_ROS) {
        initROS(argc, argv);
    }

    if (LOCAL) {
        initLocalInput(source);
    } 

    log("Quitting ...");
    
    if (DISPLAY_GRAPHICAL) cvDestroyWindow(windowResult);
    return 0;
}

void drawText(Mat &image, string text) {
    int fontFace = CV_FONT_HERSHEY_SIMPLEX;
    double fontScale = 1;
    int thickness = 1;
    int baseline = 0;
    Size textSize = getTextSize(text, fontFace, fontScale, thickness, &baseline);
    // Point textOrg((destImage.cols - textSize.width)/2,(destImage.rows + textSize.height)/2);
    Point position(0,20);
    Scalar color(0,255,0);
    putText(image, text, position, fontFace, fontScale, color, thickness, 7);
    log("%s\n", text.c_str());
}

void initWindows() {
    namedWindow(windowResult, CV_WINDOW_AUTOSIZE);
    namedWindow(windowThresh, CV_WINDOW_AUTOSIZE);
    namedWindow(windowTag, CV_WINDOW_AUTOSIZE);

    // int cvCreateTrackbar(const char* trackbarName, const char* windowName, int* value, int count, CvTrackbarCallback onChange)
    cvCreateTrackbar("SaturationColorHigh", windowResult, &SATURATION_COLOR_HIGH, 180,  onSaturationColorHighChange);
    cvCreateTrackbar("SaturationColorLow", windowResult, &SATURATION_COLOR_LOW, 180,  onSaturationColorLowChange);
    cvCreateTrackbar("CannyHigh", windowResult, &CANNY_HIGH, 360,  onCannyHighChange);
    cvCreateTrackbar("CannyLow", windowResult, &CANNY_LOW, 360,  onCannyLowChange);
    cvCreateTrackbar("GaussianKernelSize", windowResult, &GAUSSIAN_KERNEL_SIZE, 10,  onGaussianKernelChange);
    cvCreateTrackbar("GaussianSigma", windowResult, &GAUSSIAN_SIGMA, 10,  onGaussianSigmaChange);
}

// Can only handle C types
void log(string fmt, ...) {
    va_list args;
    va_start(args,fmt);

    #ifdef ROS
    // ROS_DEBUG(fmt, args);
    vprintf(fmt.c_str(), args);
    #else
    vprintf(fmt.c_str(), args);
    #endif

    va_end(args);
}

void show(const string& winname, InputArray mat) {
    if (DISPLAY_GRAPHICAL) {
        imshow(winname.c_str(), mat);
        waitKey(30);
    }
}

void onSaturationColorHighChange(int value) {
    log("SATURATION_COLOR_HIGH: %d\n", value);
    SATURATION_COLOR_HIGH = value;
}

void onSaturationColorLowChange(int value) {
    log("SATURATION_COLOR_LOW: %d\n", value);
    SATURATION_COLOR_LOW = value;
}

void onCannyHighChange(int value) {
    log("CANNY_HIGH: %d\n", value);
    CANNY_HIGH = value;
}

void onCannyLowChange(int value) {
    log("CANNY_LOW: %d\n", value);
    CANNY_LOW = value;
}

void onGaussianKernelChange(int value) {
    if ((value % 2) == 2) {
        GAUSSIAN_KERNEL_SIZE = value+1;
    }    
    else {
        GAUSSIAN_KERNEL_SIZE = value;
    }
    log("GAUSSIAN_KERNEL_SIZE: %d\n", GAUSSIAN_KERNEL_SIZE);
}

void onGaussianSigmaChange(int value) {
    log("GAUSSIAN_SIGMA: %d\n", value);
    GAUSSIAN_SIGMA = (value/10) + 1;
}

void streamCamera(VideoCapture &capture) {

    while (true) {
        // gettimeofday(&end, NULL);
        // while(double(end.tv_sec*1000000+end.tv_usec-(start.tv_sec*1000000+start.tv_usec)) < CAMERA_INTERVAL*1000.0) {
        //     oop_rate.sleep();
        //     gettimeofday(&end, NULL);
        // }
        // capture >> frameNext;
        // capture >> frame;
        // render(frame);
        // // flip(frameNext, frame, 0);
        // if(waitKey(50) >= 0) break;
        // usleep(CAMERA_INTERVAL*1000);

        if ( ! CLASSIFICATION_IN_PROGRESS && !capture.grab()) { 
            cout << "Could not grab new frame!" << endl;
        } else {
            if (!capture.retrieve(frame)) {
                cout << "Could not decode and return new frame!" << endl;
            } else {
                log("Got new frame\n");
                render(frame);
            }
        }
        
        if (DISPLAY_GRAPHICAL) {
            if(waitKey(CAMERA_INTERVAL) >= 0) break;
        } else {
            usleep(CAMERA_INTERVAL*1000);
        }
    }
}

void publishMovement(int state) {
    #ifdef ROS
    MovementCommand mc;
    mc.type = state;
    movementPublisher.publish(mc);
    #endif
}

TAG_CLASS name2class(string name) {
    if (name.compare("tags/tag_apple.png") == 0) return APPLE;
    if (name.compare("tags/tag_banana.png") == 0) return BANANA;
    if (name.compare("tags/tag_book.png") == 0) return BOOK;
    if (name.compare("tags/tag_camera.png") == 0) return CAMERA;
    if (name.compare("tags/tag_dryer.png") == 0) return DRYER;
    if (name.compare("tags/tag_glass.png") == 0) return GLASS;
    if (name.compare("tags/tag_glasses.png") == 0) return GLASSES;
    if (name.compare("tags/tag_hammer.png") == 0) return HAMMER;
    if (name.compare("tags/tag_laptop.png") == 0) return LAPTOP;
    if (name.compare("tags/tag_mug.png") == 0) return MUG;
    if (name.compare("tags/tag_scissors.png") == 0) return SCISSORS;
    if (name.compare("tags/tag_teddy.png") == 0) return TEDDY;

    return FAILURE;
}

string class2name(TAG_CLASS object) {
    switch (object) {
        case APPLE: return "Apple";
        case BANANA: return "Banana";
        case BOOK: return "Book";
        case CAMERA: return "Camera";
        case DRYER: return "Dryer";
        case GLASS: return "Glass";
        case GLASSES: return "Glasses";
        case HAMMER: return "Hammer";
        case LAPTOP: return "Laptop";
        case MUG: return "Mug";
        case SCISSORS: return "Scissors";
        case TEDDY: return "Teddy";
        case UNKNOWN: return "Unknown";
        case FAILURE: return "Failure!";
        default: return "Unknown class!";
    }
}

string color2string(COLOR object) {
    switch (object) {
        case BLACK: return "Black";
        case GREEN: return "Green";
        case BLUE: return "Blue";
        case MAGENTA: return "Magenta";
        default: return "Unknown color!";
    }
}
 // draw the ROIKeypoints on the captured frame
    // for (vector<KeyPoint>::iterator it = ROIKeypoints.begin(); it < ROIKeypoints.end(); it++) {
    //     KeyPoint keyPoint = *it;
    //     CvPoint center;
    //     int radius;
    //     center.x = cvRound(keyPoint.pt.x);
    //     center.y = cvRound(keyPoint.pt.y);
    //     radius = cvRound(keyPoint.size*1.2/9.*2);
    //     circle(srcImage, center, radius, Scalar(255, 0, 0), 2);
    // }
// TODO: mug contains very few keypoints -> special treatment?

// OpenCV Error: Unsupported format or combination of formats (type=0
// ) in buildIndex_, file /Users/wejeus/bin/OpenCV-2.4.2/modules/flann/src/miniflann.cpp, line 299
// terminate called after throwing an instance of 'cv::Exception'
//   what():  /Users/wejeus/bin/OpenCV-2.4.2/modules/flann/src/miniflann.cpp:299: error: (-210) type=0
//  in function buildIndex_

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

// parameters for ros::init(argc, argv, "TagDetection");

// Camera update using ROS is really slow! Only 1 sec updates, maybe only due to CamereNode?