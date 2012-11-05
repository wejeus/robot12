#!/usr/bin/python

import sys
import cv
import time
from optparse import OptionParser
import numpy

def get_options():
    parser = OptionParser(usage = "usage: % prog [options] [filename|camera_index]")
    parser.add_option("-c", "--cascade", action="store", dest="cascade", type="str", help="Haar cascade file, default %default", default = "./haarcascade_frontalface_alt.xml")
    (options, args) = parser.parse_args()

    if len(args) != 1: 
        parser.print_help()
        sys.exit(0)

    return (options, args)

def stream_camera(capture):
    frame_copy = None
    while True:
        frame = cv.QueryFrame(capture)
        if not frame:
            cv.WaitKey(0)
            break
        if not frame_copy:
            frame_copy = cv.CreateImage((frame.width,frame.height), cv.IPL_DEPTH_8U, frame.nChannels)
        if frame.origin == cv.IPL_ORIGIN_TL:
            cv.Copy(frame, frame_copy)
        else:
            cv.Flip(frame, frame_copy, 0)
        
        render(frame_copy)

        if cv.WaitKey(10) >= 0:
            break

# Adds good feature points as an overlay on destination_image.
def add_good_features(source_image, destination_image, MAX_COUNT=50):
    # GoodFeatures..() can only track 1 channel, use grayscale
    image_gray = cv.CreateImage(cv.GetSize(source_image), 8, 1)
    cv.CvtColor(source_image, image_gray, cv.CV_RGB2GRAY)

    # create temps used by algorithm images
    eig = cv.CreateImage(cv.GetSize(source_image), cv.IPL_DEPTH_32F, 1)
    temp = cv.CreateImage(cv.GetSize(source_image), cv.IPL_DEPTH_32F, 1)
    
    # the default parameters
    quality = 0.01
    min_distance = 10
    
    # search the good points
    features = cv.GoodFeaturesToTrack(image_gray, eig, temp, MAX_COUNT, quality, min_distance)

    for (x,y) in features:
        cv.Circle(destination_image, (x, y), 3, (0, 255, 0), -1, 8, 0)

def filter_red(src_img, dest_img):
    cv.Smooth(src_img, src_img)

    # Histogram
    # h = np.zeros((300,256,3))
    # b,g,r = cv.Split(src_img)
    # bins = np.arange(256).reshape(256,1)
    # color = [ (255,0,0),(0,255,0),(0,0,255) ]

    # CalcHist(image, hist, accumulate=0, mask=NULL)
    # histogram = cv.CalcHist(h_plane,[0],None,[256],[0,255])
    # # cv.NormalizeHist(histogram, 1)
    # hist=np.int32(np.around(hist_item))
    # pts = np.column_stack((bins,hist))
    # cv2.polylines(h,[pts],False,col)

    # h_bins = 30
    # s_bins = 32
    # hist_size = [h_bins, s_bins]
    # # hue varies from 0 (~0 deg red) to 180 (~360 deg red again */
    # h_ranges = [0, 180]
    # # saturation varies from 0 (black-gray-white) to
    # # 255 (pure spectrum color)
    # s_ranges = [0, 255]
    # ranges = [h_ranges, s_ranges]
    # scale = 10
    # hist = cv.CreateHist(h_bins, cv.CV_HIST_ARRAY, ranges, 1)
    # cv.CalcHist(src_img, hist)
    # (_, max_value, _, _) = cv.GetMinMaxHistValue(hist)
    # for h in range(h_bins):
    #     bin_val = cv.QueryHistValue_2D(hist, h, s)
    #     intensity = cv.Round(bin_val * 255 / max_value)
    #     cv.Rectangle(hist_img,
    #                  (h*scale, s*scale),
    #                  ((h+1)*scale - 1, (s+1)*scale - 1),
    #                  cv.RGB(intensity, intensity, intensity), 
    #                  cv.CV_FILLED)

    (rows, cols) = cv.GetSize(src_img)

    img_hsv = cv.CreateImage((rows, cols), cv.IPL_DEPTH_8U, 3)
    h_plane = cv.CreateImage((rows, cols), 8, 1)
    s_plane = cv.CreateImage((rows, cols), 8, 1)
    img_binary = cv.CreateImage((rows, cols), 8, 1)
    cv.CvtColor(src_img, img_hsv, cv.CV_RGB2HSV)    
    # cv.Split(img_hsv, h_plane, s_plane, None, None)
    
    # cv.EqualizeHist(h_plane, h_plane) # Gives better result in form of less flickering

    cv.InRangeS(img_hsv, cv.Scalar(COLOR_RANGE_LOW, 100, 100), cv.Scalar(COLOR_RANGE_HIGH, 255,255), h_plane)


    # create temps used by algorithm images
    # eig = cv.CreateImage(cv.GetSize(src_img), cv.IPL_DEPTH_32F, 1)
    # temp = cv.CreateImage(cv.GetSize(src_img), cv.IPL_DEPTH_32F, 1)
    # the default parameters
    # quality = 0.01
    # min_distance = 30 # min distance between detected points
    # search the good points
    # features = cv.GoodFeaturesToTrack(h_plane, eig, temp, 20, quality, min_distance)
    # for (x,y) in features:
    #     cv.Circle(dest_img, (x, y), 3, (0, 255, 0), -1, 8, 0)

    cv.Canny(h_plane, h_plane, CANNY_LOW, CANNY_HIGH)
    cv.ShowImage("thresh", h_plane)

    contours = cv.FindContours(h_plane, cv.CreateMemStorage(), cv.CV_RETR_EXTERNAL, cv.CV_CHAIN_APPROX_SIMPLE)
    while contours:
        rect = cv.MinAreaRect2(contours)
        if cv.ContourArea(contours) > 1000:
            print "FOUND RECT WITH AREA: %s" % cv.ContourArea(contours)
            box = cv.BoxPoints(rect)
            for i in range(4):
                cv.Line(dest_img, box[i], box[(i+1)%4], (0,255,0), 6, 8);
        contours = contours.h_next()

    # HoughLines2(image, storage, method, rho, theta, threshold, param1=0, param2=0) 
    # lines = cv.HoughLines2(h_plane, cv.CreateMemStorage(), cv.CV_HOUGH_PROBABILISTIC, cv.CV_PI/180, 1, 50, 1)
    # for l in lines:
    #     (p1, p2) = l
    #     # Line(img, pt1, pt2, color, thickness=1, lineType=8, shift=0)
    #     cv.Line(dest_img, p1, p2, cv.Scalar(0,0,255), 2, cv.CV_AA)

    cv.ShowImage("result", dest_img)

def plot_contours(src_image, dest_image):
    cv.NamedWindow("debug", cv.CV_WINDOW_AUTOSIZE)

    # Better to use HSV when doing extraction
    (rows, cols) = cv.GetSize(src_image)
    image_hsv = cv.CreateImage((rows, cols), cv.IPL_DEPTH_8U, 3)
    h_plane = cv.CreateImage((rows, cols), 8, 1)
    s_plane = cv.CreateImage((rows, cols), 8, 1)
    image_bin = cv.CreateImage((rows, cols), 8, 1)
    cv.CvtColor(src_image, image_hsv, cv.CV_RGB2HSV)
    cv.Laplace(h_plane, h_plane, 3)
    cv.Threshold(h_plane, h_plane, 40.0, 255.0, cv.CV_THRESH_BINARY)
    cv.Smooth(h_plane, h_plane, cv.CV_BLUR, 5, 5) # Its suggested that smoothing first gives better results
    cv.ShowImage("debug", h_plane)

    # Create binary image to be used for contour detection
    # image_gray = cv.CreateImage(cv.GetSize(src_image), 8, 1)
    # cv.CvtColor(image_hsv, image_gray, cv.CV_BGR2GRAY)
    # cv.Laplace(image_gray, image_gray, 3)
    # cv.Threshold(image_gray, image_gray, 40.0, 255.0, cv.CV_THRESH_BINARY)
    # cv.Smooth(image_gray, image_gray, cv.CV_BLUR, 5, 5) # Its suggested that smoothing first gives better results
    # cv.ShowImage("debug", image_gray)


    contours = cv.FindContours(image_gray, cv.CreateMemStorage(), cv.CV_RETR_LIST, cv.CV_CHAIN_APPROX_SIMPLE)
    #cv.DrawContours(dest_image, contours, (255,0,0), (0,255,0), 1, 2)
    
    # Draw bounding box + mass center of shape
    while contours:
        rect = cv.MinAreaRect2(contours)
        box = cv.BoxPoints(rect)
        for i in range(4):
            cv.Line(dest_image, box[i], box[(i+1)%4], (0,0,255), 1, 8);
       
        mom = cv.Moments(contours)
        for j in mom:
            print j
        # momPoint = cv.CvPoint(mom.m10/mom.m00,mom.m01/mom.m00)
        # cv.Circle(dest_image, (mom.m10/mom.m00,mom.m01/mom.m00), 2, (0,255,255))
        # r0 = cv.BoundingRect(cnt)
        # Rectangle(dest_image, pt1, pt2, (0,255,0))
        contours = contours.h_next()

def canny(src_image, dest_image):
    image_gray = cv.CreateImage(cv.GetSize(src_image), 8, 1)
    i = cv.CreateImage(cv.GetSize(src_image), 8, 1)
    cv.CvtColor(src_image, image_gray, cv.CV_RGB2GRAY)
    cv.Canny(image_gray, i, CANNY_LOW, CANNY_HIGH)
    cv.ShowImage("result", i)

def render(image):
    if FILTER_RED: filter_red(image, image)
    # if DRAW_GOOD_FEATURES: add_good_features(image, image, 200)
    # if FIND_CONTOURS: plot_contours(image, image)
    # if DO_CANNY: canny(image, image)
    # cv.ShowImage("result", image)


def onCannyLowChange(value):
    global CANNY_LOW 
    print "CANNY_LOW: %s" % value
    CANNY_LOW = value

def onCannyHighChange(value):
    global CANNY_HIGH
    print "CANNY_HIGH: %s" % value 
    CANNY_HIGH = value

def onColorRangeLowChange(value):
    global COLOR_RANGE_LOW 
    print "COLOR_RANGE_LOW: %s" % value
    COLOR_RANGE_LOW = value

def onColorRangeHighChange(value):
    global COLOR_RANGE_HIGH
    print "COLOR_RANGE_HIGH: %s" % value 
    COLOR_RANGE_HIGH = value

# What to do
FIND_CONTOURS = True
FILTER_RED = True
DRAW_GOOD_FEATURES = True
DO_CANNY = True
# Default values
CANNY_LOW = 77
CANNY_HIGH = 92
COLOR_RANGE_LOW = 83
COLOR_RANGE_HIGH = 139

if __name__ == '__main__':
    (options, args) = get_options()

    # Setup input as either camera or image
    input_source = args[0]
    if input_source.isdigit():
        capture = cv.CreateCameraCapture(int(input_source))
    else:
        capture = None

    # Setup window to draw to
    cv.NamedWindow("result", cv.CV_WINDOW_AUTOSIZE)
    cv.NamedWindow("thresh", cv.CV_WINDOW_AUTOSIZE)
    cv.NamedWindow("histogram", cv.CV_WINDOW_AUTOSIZE)
    # Trackbar to experiment with values
    # cv.CreateTrackbar("Canny Low", "thresh", CANNY_LOW, 100, onCannyLowChange)
    # cv.CreateTrackbar("Canny High", "thresh", CANNY_HIGH, 100, onCannyHighChange)
    # FIXME: # hue varies from 0 (~0 deg red) to 180 (~360 deg red again */
    cv.CreateTrackbar("Color range low", "thresh", COLOR_RANGE_LOW, 360, onColorRangeLowChange)
    cv.CreateTrackbar("Color range high", "thresh", COLOR_RANGE_HIGH, 360, onColorRangeHighChange)

    if capture:
        stream_camera(capture)
    else:
        image = cv.LoadImage(input_source, cv.CV_LOAD_IMAGE_COLOR)
        render(image)
        # Wait for any keypress then quit
        cv.WaitKey(0)
    
    cv.DestroyWindow("result")
    

