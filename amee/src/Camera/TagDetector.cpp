
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
#include <sys/time.h>
#include <sstream>
#include <string>
#include "CameraHack.h"

#define ROS

#ifdef ROS
#include "ros/ros.h"
#include "amee/Tag.h"
#include "amee/Velocity.h"
#include "amee/MovementCommand.h"
#include "roboard_drivers/Motor.h"
#include <ros/console.h>
using namespace amee;
#endif

using namespace std;
using namespace cv;

enum TAG_CLASS {NOTHING, FAILURE, UNKNOWN, APPLE, BANANA, BOOK, CAMERA, DRYER, GLASS, GLASSES, HAMMER, LAPTOP, MUG, SCISSORS, TEDDY};
enum COLOR {UNKNOWN_COLOR, BLACK, GREEN, BLUE, MAGENTA};

struct TagInfo {
    TAG_CLASS type;
    COLOR color;
    ros::Time timestamp;
};

struct PixelSum {
    COLOR color;
    int numPixels;
};

struct TagTemplate {
    TAG_CLASS object;
    Mat image;
};

/* Flags */
bool EQUALIZE_HISTOGRAM = true;
bool DISPLAY_GRAPHICAL = false;
int CAMERA_INTERVAL = 10;
bool USE_TEMPLATES = false;
bool USE_ROS = false;
bool CLASSIFICATION_IN_PROGRESS = false;
bool WAIT_FOR_TIMEOUT = false;
bool SOURCE_IS_SINGLE_IMAGE = false;

/* Constants */
char windowResult[] = "result";
char windowThresh[] = "thresh";
char windowTag[] = "tag";

int SATURATION_COLOR_HIGH = 20;
int SATURATION_COLOR_LOW = 0;
// int CANNY_LOW = 1;
// int CANNY_HIGH = 95;
int CANNY_LOW = 77;
int CANNY_HIGH = 92;
int GAUSSIAN_KERNEL_SIZE = 7;
int GAUSSIAN_SIGMA = 1.5;
double TAG_MIN_PIXEL_AREA = 1500.0;
int TAG_SIZE = 50;
int NEEDED_PIXELS_RED_BORDER = 100;
int NEEDED_PIXELS_RED_THRESHOLD = 200;
char TAG_FILES[] = "tags/files.txt";

void render(Mat &frame, ros::Time timestamp);
void show(const string& winname, InputArray mat);
void log(string fmt, ...);
string class2name(TAG_CLASS object);
TAG_CLASS name2class(string name);
string color2string(COLOR object);
void drawText(Mat &image, string text);
void initROS(int argc, char *argv[]);
void publishMap(ros::Time timestamp, int object, int color);
void stream(string source);
bool captureSingleFrame(Mat &frame);


/* For classification */
int filterRedPixels(Mat &srcImage, Mat &destImage);
bool findTagROI(Mat &srcImage, Mat &thresholdedImage, Mat &ROI);
bool prepareROI(Mat &srcROI, Mat &destROI);
COLOR determineTagColor(Mat &ROI);
void thresholdBlackWhite(Mat &srcImage, Mat &destImage);
bool initTemplates();
TAG_CLASS classifyTagUsingTemplate(Mat &ROI);
bool isReadyForNextTag();
void setNextTagTimout();


void analyse(ros::Time ts);


/* Globals */
double lastTagTimeout;
deque<TagInfo> tagHistory;
int CAMERA_ID = -1;
bool CAPTURE_HIGHRES_IMAGE = false;
vector<TagTemplate> sourceTemplates;
int erosionSize = 2;
Mat erosionElement = getStructuringElement(MORPH_ELLIPSE, Size(2*erosionSize + 1, 2*erosionSize + 1));


#ifdef ROS
ros::Publisher mapPublisher;
ros::Publisher movementPublisher;
ros::Publisher rawMotorPublisher;
#endif

void onTrackbar(int value, void* = NULL) {
    cout << value << endl;
}

// TODO: Unknown color on -> b1, b2, g2, g3, m2, s2
// export LD_PRELOAD=/usr/lib/libv4l/v4l1compat.so
int main(int argc, char *argv[]) {

    int c;
    string source;

    if (argc == 1) {
        cout << "Options (default):" << endl
             << "t (false) - loads and classifies using TEMPLATES" << endl
             << "d (false) - display results graphically" << endl
             << "r (false) - init ROS (setup publishers/subscribers)" << endl
             << "w (false) - use timeout between tag classifications" << endl
             << "s (nothing) - which source to use: (<int> | <path>) representing: (</dev/videoX> | <path of img/video>)" << endl;
             return 0;
    }

    while ((c = getopt (argc, argv, "trwds:")) != -1) {
        switch (c) {
            case 'd':
                DISPLAY_GRAPHICAL = true;
                break;
            case 't':
                USE_TEMPLATES = true;     
                break;
            case 'r':
                USE_ROS = true;     
                break;
            case 'w':
                WAIT_FOR_TIMEOUT = true;     
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
        namedWindow(windowResult, CV_WINDOW_AUTOSIZE);
        namedWindow(windowThresh, CV_WINDOW_AUTOSIZE);
        namedWindow(windowTag, CV_WINDOW_AUTOSIZE);
        createTrackbar("LOW", windowResult, &SATURATION_COLOR_LOW, 20, onTrackbar);
        createTrackbar("HIGH", windowResult, &SATURATION_COLOR_HIGH, 20, onTrackbar);
    }

    if (USE_TEMPLATES && !initTemplates()) {
        log("Failed to initialize tag objects!\n");
        return -1;
    }

    if (USE_ROS) {
        initROS(argc, argv);
    }

    stream(source);
    
    log("Quitting ...");
    
    if (DISPLAY_GRAPHICAL) cvDestroyWindow(windowResult);
    return 0;
}

int id = 0;
char buf[100];
void imout(Mat &frame, string type) {
    std::ostringstream ostr;
    ostr << type << id << ".jpg";
    std::string s = ostr.str();
    //imwrite(strcat(strcat(type.c_str(), s.c_str()), ".jpg"), frame);
    imwrite(s, frame);
    id++;
}

double getTime() {
    struct timeval timestamp;
    gettimeofday(&timestamp, NULL);
    return timestamp.tv_sec+double(timestamp.tv_usec)/1000000.0;
}


// TODO: Goal -> Classification block should not run so often...
void render(Mat &frame, ros::Time timestamp) {

    Mat thresholdedImage, ROI;
    TagInfo tag;
    tag.type = NOTHING;
    tag.color = UNKNOWN_COLOR;
    tag.timestamp = timestamp;
        
    int redPixelsInImage = filterRedPixels(frame, thresholdedImage);

    if (redPixelsInImage > NEEDED_PIXELS_RED_THRESHOLD && isReadyForNextTag()) {

        CLASSIFICATION_IN_PROGRESS = true;

        if (findTagROI(frame, thresholdedImage, ROI)) {

            if (!prepareROI(ROI, ROI)) {
                // imout(frame, "noPrepare_");
                log("Could not prepare ROI!\n");
            }

            if (USE_TEMPLATES) tag.type = classifyTagUsingTemplate(ROI);
            tag.color = determineTagColor(ROI);      
        } else {
            // imout(frame, "noFindRect_");
            // log("\nCould not find (possible) tag rectangle in image\n");
        }
                
        CLASSIFICATION_IN_PROGRESS = false;  
    }

    if (tag.type == NOTHING || tag.type == FAILURE) {
        log("[%d.%d] <nothing found>\n", timestamp.sec, timestamp.nsec);
        // imout(frame, "fail_");
    } else {
        log("[%d.%d] Found: %s with color: %s\n", timestamp.sec, timestamp.nsec, class2name(tag.type).c_str(), color2string(tag.color).c_str());
        tagHistory.push_front(tag);
        analyse(timestamp);
        setNextTagTimout();
    }

    show(windowResult, frame);
}


void analyse(ros::Time ts) {
    vector<int> types(15,0);
    vector<int> colors(5,0);

    for (deque<TagInfo>::iterator it = tagHistory.begin(); it < tagHistory.end(); ++it) {
        if ((ts.sec - it->timestamp.sec) < 4) { // Filter out old measurements
            types[(int)it->type]++;
            colors[(int)it->color]++;
        }
    }

    int bestType = -1;
    int typeID;
    for (int i = 0; i < 15; ++i) {
        if (types[i] > bestType) {
            bestType = types[i];
            typeID = i;
        }
    }

    int bestColor = -1;
    int colorID;
    for (int i = 0; i < 5; ++i) {
        if (colors[i] > bestColor) {
            bestColor = types[i];
            colorID = i;
        }
    }

    if (bestType >= 2) {
        // log("Publish. Type: %d Color: %d\n", typeID, colorID);
        // Publishes the previous ts to get more accurate estimate of position of frame
        publishMap(tagHistory[1].timestamp, typeID, colorID); 
    }

    if (tagHistory.size() > 5) {
        tagHistory.erase(tagHistory.begin()+2, tagHistory.end());
    }
}

bool isReadyForNextTag() {
    if ( ! WAIT_FOR_TIMEOUT) {
        return true;
    }

    struct timeval currentTimestamp;
    gettimeofday(&currentTimestamp, NULL);
    double t = currentTimestamp.tv_sec+double(currentTimestamp.tv_usec)/1000000.0;
    
    if ((t - lastTagTimeout) > 3.0f) {
        // log("timeout TRUE: %f\n", (t - lastTagTimeout));
        return true;
    } else {
        // log("timeout FALSE: %f\n", (t - lastTagTimeout));
        return false;
    }
}

void setNextTagTimout() {
    struct timeval tagTimeout;
    gettimeofday(&tagTimeout, NULL);
    lastTagTimeout = tagTimeout.tv_sec+double(tagTimeout.tv_usec)/1000000.0;
}

/* Thresholds redpixels and saves resulting binary image in destImage, returns number of red pixels in image */
int filterRedPixels(Mat &srcImage, Mat &destImage) {
    resize(srcImage, destImage, Size(100, 100), 0, 0, INTER_LINEAR);

    Mat hsvImage, firstRedSection, secondRedSection;
    cvtColor(srcImage, hsvImage, CV_BGR2HSV);
    
    // Works very good if the red in the tag is not dark. If it's in a position to reflect a little light directly it works very good.
    inRange(hsvImage, Scalar(0, 100, 0), Scalar(10, 255, 255), firstRedSection);
    inRange(hsvImage, Scalar(169, 100, 0), Scalar(179, 255, 255), secondRedSection);
    bitwise_or(firstRedSection, secondRedSection, destImage);

    erode(destImage, destImage, erosionElement);

    // inRange(hsvImage, Scalar(SATURATION_COLOR_LOW, 100, 0), Scalar(SATURATION_COLOR_HIGH, 255, 255), destImage);
    // inRange(hsvImage, Scalar(140, 100, 0), Scalar(179, 255, 255), destImage);
    
    int numRedPixels = countNonZero(destImage);

    return numRedPixels;
}

/* Looks for a red rectangle of a certain size in the image */
bool findTagROI(Mat &srcImage, Mat &thresholdedImage, Mat &ROI) {

    // GaussianBlur(srcImage, srcImage, Size(GAUSSIAN_KERNEL_SIZE, GAUSSIAN_KERNEL_SIZE), GAUSSIAN_SIGMA, GAUSSIAN_SIGMA);
    
    // dilate(thresholdedImage, thresholdedImage, erosionElement);

    Canny(thresholdedImage, thresholdedImage, CANNY_LOW, CANNY_HIGH);

    vector< vector<Point> > contours;
    findContours(thresholdedImage, contours, CV_RETR_EXTERNAL, CV_CHAIN_APPROX_SIMPLE);

    // Filter out contours that are big enough to hold a tag rectangle
    for(int i = 0; i < contours.size(); ++i) {

        if (contourArea(contours[i]) > TAG_MIN_PIXEL_AREA) {
            // log("Area: %d", contourArea(contours[i]));
            Rect rect = boundingRect(contours[i]);
            // Make sure found ROI do not exeed image boundaries
            if (0 <= rect.x && 0 <= rect.width && rect.x + rect.width <= srcImage.cols && 0 <= rect.y && 0 <= rect.height && rect.y + rect.height <= srcImage.rows) {
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
    
    Mat dummy;

    if (filterRedPixels(srcROI, dummy) > NEEDED_PIXELS_RED_BORDER) {
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
            resize(croppedROI, destROI, Size(TAG_SIZE, TAG_SIZE), 0, 0, INTER_LINEAR);
        } else {
            log("Warning: invalid dimensions of ROI when classifying.\n");
            return false;
        }
    } else {
        resize(srcROI, destROI, Size(TAG_SIZE, TAG_SIZE), 0, 0, INTER_LINEAR);
    }

    return true;
}

// TODO: Adjust color params to better match colors.
COLOR determineTagColor(Mat &srcROI) {
    float borderRatio = 0.3f;
    int width = srcROI.cols;
    int height = srcROI.rows;
    int borderSizeX = width*borderRatio;
    int borderSizeY = height*borderRatio;
    Rect nonBorderROI(borderSizeX, borderSizeY, width-(2*borderSizeX), height-(2*borderSizeY));

    Mat ROI;
    // Check for correct dimensions (new ROI smaller than original)
    if (0 <= nonBorderROI.x && 0 <= nonBorderROI.width && nonBorderROI.x + nonBorderROI.width <= srcROI.cols && 0 <= nonBorderROI.y && 0 <= nonBorderROI.height && nonBorderROI.y + nonBorderROI.height <= srcROI.rows) {
        ROI = srcROI(nonBorderROI);
    }

    // show(windowTag, ROI);

    int saturationLow = 50;
    int saturationHigh = 255;
    int valueLow = 0;
    int valueHigh = 150;

    Mat hsvImage, binaryImage;
    cvtColor(ROI, hsvImage, CV_BGR2HSV);

    // vector<Mat> hsv;
    // split(hsvImage, hsv);

    // /// Establish the number of bins
    // int histSize = 180;
    // /// Set the ranges ( for B,G,R) )
    // float range[] = { 0, 180 } ;
    // const float* histRange = { range };
    // bool uniform = true;
    // bool accumulate = false;
    // Mat b_hist;

    // calcHist( &hsv[0], 1, 0, Mat(), b_hist, 1, &histSize, &histRange, uniform, accumulate );
    // int hist_w = 512;
    // int hist_h = 400;
    // int bin_w = cvRound( (double) hist_w/histSize );

    // Mat histImage( hist_h, hist_w, CV_8UC3, Scalar( 0,0,0) );
    // /// Draw for each channel
    // for( int i = 1; i < histSize; i++ ) {
    //     line( histImage, Point( bin_w*(i-1), hist_h - cvRound(b_hist.at<float>(i-1)) ) ,
    //                    Point( bin_w*(i), hist_h - cvRound(b_hist.at<float>(i)) ),
    //                    Scalar( 255, 0, 0), 2, 8, 0  );
    // }
    // //Show histogram
    // show(windowThresh, histImage);
    // waitKey(10);



    vector<PixelSum> elems;
    PixelSum ps;

    // Green
    inRange(hsvImage, Scalar(55, saturationLow, valueLow), Scalar(90, saturationHigh, valueHigh), binaryImage);
    ps.numPixels = countNonZero(binaryImage);
    ps.color = GREEN;
    elems.push_back(ps);

    // Blue
    inRange(hsvImage, Scalar(105, saturationLow, valueLow), Scalar(135, saturationHigh, valueHigh), binaryImage);
    ps.numPixels = countNonZero(binaryImage);
    ps.color = BLUE;
    elems.push_back(ps);

    // Magenta
    inRange(hsvImage, Scalar(150, saturationLow, valueLow), Scalar(179, saturationHigh, valueHigh), binaryImage);
    ps.numPixels = countNonZero(binaryImage);
    ps.color = MAGENTA;
    elems.push_back(ps);

    PixelSum bestMatch;
    bestMatch.color = UNKNOWN_COLOR;
    bestMatch.numPixels = 0;
    for (vector<PixelSum>::iterator it = elems.begin(); it < elems.end(); ++it) {
        // log("COLOR: %s VALUE: %d\n", color2string(it->color).c_str(), it->numPixels);
        if (it->numPixels > bestMatch.numPixels) {
            bestMatch.color = it->color;
            bestMatch.numPixels = it->numPixels;
        }
    }

    PixelSum secondBestMatch;
    secondBestMatch.color = UNKNOWN_COLOR;
    secondBestMatch.numPixels = 0;
    for (vector<PixelSum>::iterator it = elems.begin(); it < elems.end(); ++it) {
        if (it->color != bestMatch.color && it->numPixels > secondBestMatch.numPixels) {
            secondBestMatch.color = it->color;
            secondBestMatch.numPixels = it->numPixels;
        }
    }

    int diff = bestMatch.numPixels - secondBestMatch.numPixels;
    // cout << "Diff: " << diff << endl;

    if (diff < 10) {
        return BLACK;
    }
    // if (bestMatch.numPixels < 1000) {
    //     bestMatch.color = BLACK;
    // } else if (elems[0].numPixels > 1000 && elems[1].numPixels > 1000) {
    //     log("asdfasdf\n");
    //     bestMatch.color = BLACK;
    // }

    // Debug: adjustment of saturation params.
    // inRange(hsvImage, Scalar(SATURATION_COLOR_LOW, saturationLow, valueLow), Scalar(SATURATION_COLOR_HIGH, saturationHigh, valueHigh), binaryImage);
    // log("Match: %d\n", countNonZero(binaryImage));
    // show(windowTag, binaryImage);

    return bestMatch.color;
}

bool initTemplates() {
    cout << "Init templates..." << endl;

    ifstream infile;

    infile.open(TAG_FILES);
    string line;

    if (infile.is_open()) {
        while (std::getline(infile, line)) {
            TagTemplate tag;
            tag.object = name2class(line);
            tag.image = imread(line, CV_LOAD_IMAGE_GRAYSCALE);
    
            resize(tag.image, tag.image, Size(TAG_SIZE, TAG_SIZE), 0, 0, INTER_LINEAR);
            thresholdBlackWhite(tag.image, tag.image);
            
            // Debug
            // show(windowTag, tag.image);
            // waitKey();

            if( ! tag.image.data) {
                cout << "Image not found!" << endl;
                return false;
            }

            sourceTemplates.push_back(tag);

            cout << "Read template: " << line.c_str() << endl;
        }
    } else {
        cout << "Could not read tag filenames!" << endl;
        return false;
    }

    infile.close();
    return true;
}

void thresholdBlackWhite(Mat &srcImage, Mat &destImage) {
    threshold(srcImage, destImage, 110, 255, THRESH_BINARY);
}

/* Assumes borders have been removed */
TAG_CLASS classifyTagUsingTemplate(Mat &ROI) {
    
    Mat tmpROI;    
    
    cvtColor(ROI, tmpROI, CV_BGR2GRAY);

    // Do some processing to easier separate object from background
    GaussianBlur(tmpROI, tmpROI, Size(GAUSSIAN_KERNEL_SIZE, GAUSSIAN_KERNEL_SIZE), GAUSSIAN_SIGMA, GAUSSIAN_SIGMA);
    // equalizeHist(tmpROI, tmpROI);
    thresholdBlackWhite(tmpROI, tmpROI);

    double bestMatch = 10000; // infinity
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

    // log("bestMatch: %f (error: %f)\n", bestMatch, error);
    if (bestMatch > 0.5f) {
        return UNKNOWN;
    }

    return obj;
}

void initROS(int argc, char *argv[]) {
    #ifdef ROS
    log("Initializing ROS...\n");
    ros::init(argc, argv, "TagDetection");
    ros::NodeHandle rosNodeHandle;

    mapPublisher = rosNodeHandle.advertise<Tag>("/amee/tag", 100);
    movementPublisher = rosNodeHandle.advertise<MovementCommand>("/MovementControl/MovementCommand", 1);
//    rawMotorPublisher = rosNodeHandle.advertise<Motor>("/serial/motor_speed", 100);

    #else
    log("ROS IS NOT DEFINED");
    #endif
}

VideoCapture *capture = new VideoCapture();
// VideoCapture *capture;


bool captureSingleFrame(Mat &frame) {
    // VideoCapture capture;
    // capture.open(CAMERA_ID);

    // if (!capture.grab()) { 
    //     cout << "Could not grab new frame!" << endl;
    //     return false;
    // } else {
    //     if (!capture.retrieve(frame)) {
    //        cout << "Could not decode and return new frame!" << endl;
    //        return false;
    //     }
    // }
    // return true;

    //capture->open(CAMERA_ID);

    if (CAPTURE_HIGHRES_IMAGE) {
        capture->set(CV_CAP_PROP_FRAME_WIDTH, 640);
        capture->set(CV_CAP_PROP_FRAME_HEIGHT, 480);
    }

    if (!capture->read(frame)) {
       cout << "Could not decode and return new frame!" << endl;
       return false;
    }

    return true;
}

// TODO: Stream from video file
void stream(string source) {
    cout << "Starting TagDetection using raw input." << endl;

    if (!strcmp(source.c_str(), "")) {
        cout << "No input source!" << endl;
        return;
    }

    if (isdigit(*source.c_str())) {
        cout << "I will now stream from camera: " << source << endl;
        CAMERA_ID = atoi(source.c_str());
    } else {
        SOURCE_IS_SINGLE_IMAGE = true;
    }

    if (SOURCE_IS_SINGLE_IMAGE) {
        cout << "Reading single image: " << source.c_str() << endl;
        Mat image = imread(source.c_str(), 1);
        // render(image);
        waitKey();
    } else {
        cout << "Streaming starting..." << endl;
        Mat frame;
        cyclops::CameraHack ch_capture(CAMERA_ID, 320, 240);
        ros::Time timestamp;

        while (true) {
            ch_capture.getFreshImg(frame, timestamp.sec, timestamp.nsec);
            render(frame, timestamp);
        }
        // cvReleaseCapture(&cv_cap);
    }

}



/* -------------------------------------------------------------------------- */
/* - HELPERS ---------------------------------------------------------------- */
/* -------------------------------------------------------------------------- */


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
    }
}

void publishMap(ros::Time timestamp, int object, int color) {
    #ifdef ROS
    if (USE_ROS) {
        Tag t;
        t.ts_sec = timestamp.sec;
        t.ts_nsec = timestamp.nsec;
        t.side = 1;
        t.object = object;
        t.color = color;
        mapPublisher.publish(t);
        cout << "mapPublisher: " << timestamp << " : " << object << " : " << color << endl;
    }
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