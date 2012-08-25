// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <cv.h>
#include <highgui.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/image_encodings.h>

#include "geometry.h"

using namespace std;
using namespace cv;

// Canny threshold
int lowThreshold = 120;
int houghThreshold = 80;
int minLineLength = 40;
int maxLineGap = 10;
int shtThreshold = 100;
int lineDistThresh = 50; //mm
int lineAngleThresh = 6; //degrees
Mat frame;

camera cam;


/* Given an input image frame, returns a set of court line candidates
 * in groupedLines.
 *
 * Each line is represented as a two endpoints of a line segment,
 * in pixel coordinates (x1, y1, x2, y2).
 */
void findLines(Mat &frame, vector<Vec4i> &groupedLines) {
    Mat frame_gray, frame_thresh;

    /*/ Resize input image
    if (frame.cols != 640)
        resize(frame, frame, Size(640, 480));
    */

    // Threshold by distance: blank out all top pixels
    //rectangle(frame, Point(0,0), Point(640, 100), Scalar(0,0,0), CV_FILLED);

    // Convert to grayscale
    cvtColor(frame, frame_gray, CV_RGB2GRAY, 1);

    // Increase contrast, decrease brightness
    frame_gray.convertTo(frame_gray, -1, 1.6, -120);

    imshow("gray", frame_gray);

    // Color threshold to find white lines
    //  TODO tune thresholds
    threshold(frame_gray, frame_thresh, 210, 255, THRESH_BINARY);

    Canny(frame_thresh, frame_thresh, lowThreshold, 3*lowThreshold, 3);
    imshow("edges", frame_thresh);

    // Probabilistic Hough transform
    vector<Vec4i> lines;
    HoughLinesP(frame_thresh, lines, 2, CV_PI/180,
                houghThreshold, minLineLength, maxLineGap);


    /* Trial
    vector<Vec2f> lines;
    HoughLines( frame_thresh, lines, 2, CV_PI/180, shtThreshold );

    Mat color_dst = frame.clone();
    for( size_t i = 0; i < lines.size(); i++ ) {
        float rho = lines[i][0];
        float theta = lines[i][1];
        double a = cos(theta), b = sin(theta);
        double x0 = a*rho, y0 = b*rho;
        Point pt1(cvRound(x0 + 1000*(-b)),
                  cvRound(y0 + 1000*(a)));
        Point pt2(cvRound(x0 - 1000*(-b)),
                  cvRound(y0 - 1000*(a)));
        line( color_dst, pt1, pt2, Scalar(0,0,255), 3, 8 );
    }
    imshow("std hough", color_dst);
    //*/

    // Consolidate duplicate lines
    vector<int> numVotes;
    RNG rng(0L);
    while (!lines.empty()) {
        Vec4i group;
        int lineCount;
        int lastLineCount;

        // Select the last line and try to find all matching line segments
        group = lines.back();
        lines.pop_back();
        lineCount = 1;

        // Keep matching until lineCount doesn't change
        do {
            lastLineCount = lineCount;

            for (int i = lines.size()-1; i >= 0; i -= 1) {
                const Vec4i &line = lines[i];

                // Distance metric is based on
                //  (a) the distance between the line segment center and
                //      the _infinite_ combined line (not line segment)
                //  (b) the angle difference between the line segment
                //double a = pointLineDistance(Vec2i(line[0], line[1]), group) +
                //    pointLineDistance(Vec2i(line[2], line[3]), group);
                //double b = abs(normalizeRadians(lineAngle(line) -
                //                                lineAngle(group)));

                Vec4d realLine = cameraLineToRobot(line, cam),
                    groupedLine = cameraLineToRobot(group, cam);
                // a = distance from midpoint of line to the infinite combined
                //     line in the _ground plane_ (not the image plane)
                double a = pointLineDistance(lineCenter(realLine),groupedLine),
                    b = abs(normalizeRadians(lineAngle(realLine) -
                                             lineAngle(groupedLine)));

                // Include all lines within 1 std deviation using the
                //  pseudo-Gaussian distribution
                //double closeness = exp(-a/15.0 + -b/0.15);
                //if (closeness > 0.367) {
                //if (a < 15 && b < 0.1) {
                double athresh = lineDistThresh / 10.0,
                    bthresh = lineAngleThresh * CV_PI / 180;
                if (a < athresh && b < bthresh) {

                    // Merge the line with the group by taking the 2 points
                    //  with smallest/largest {x,y} values
                    double groupAngle = lineAngle(group);
                    int offset = 0;

                    // If line is more vertical: compare y values
                    //  Otherwise use x values
                    if (groupAngle > CV_PI/4 || groupAngle < -CV_PI/4)
                        offset = 1;

                    // Swap points so that group[0:1] has
                    //  the smaller {x,y} value
                    if (group[0+offset] > group[2+offset]) {
                        swap(group[0], group[2]);
                        swap(group[1], group[3]);
                    }

                    for (int j = 0; j < 4; j += 2) {
                        // Find the point with smallest {x,y} value
                        if (line[j+offset] < group[0+offset]) {
                            group[0] = line[j];
                            group[1] = line[j+1];
                        }
                        // Find the point with largest {x,y} value
                        if (line[j+offset] > group[2+offset]) {
                            group[2] = line[j];
                            group[3] = line[j+1];
                        }
                    }

                    lineCount += 1;
                    lines.erase(lines.begin()+i);
                }
            } // for

        } while (lineCount != lastLineCount);

        groupedLines.push_back(group);
        numVotes.push_back(lineCount);
    } // while (!lines.empty())

    //*/
} // findLines()

/* Draws a list of line segments onto the given frame,
 *  each in a different random color.
 */
void drawLines(Mat &frame, const vector<Vec4i> &lines) {
    RNG rng;
    for( size_t i = 0; i < lines.size(); i++ ) {
        const Vec4i &l = lines[i];
        Scalar color(rng(256), rng(256), rng(256));
        
        line( frame, Point(l[0], l[1]), Point(l[2], l[3]),
              color, 2, 8 );
    }
}

void redraw(int, void*) {
    vector<Vec4i> lines;
    findLines(frame, lines);

    Mat frame_copy = frame.clone();
    drawLines(frame_copy, lines);
    imshow("img", frame_copy);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        frame = cv_ptr->image;
        redraw(0, NULL);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");


    // Create image listener on topic ~image (default "gscam/image_raw")
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imageSub;
    string imageTopic;
    if (!nhPrivate.hasParam("image_transport"))
        nhPrivate.setParam("image_transport", "compressed");
    nhPrivate.param<string>("image", imageTopic, "camera0/image_raw");
    imageSub = it.subscribe(imageTopic, 1, imageCallback);

    // Create windows
    namedWindow("img", CV_WINDOW_AUTOSIZE);
    cvMoveWindow("img", 320, 0);
    namedWindow("gray", 0);
    cvMoveWindow("gray", 0, 0);
    //namedWindow("thresh", 0);
    //cvMoveWindow("thresh", 50, 350);
    namedWindow("edges", 0);
    cvMoveWindow("edges", 0, 350);
    namedWindow("std hough", 0);
    cvMoveWindow("std hough", 0, 700);

    /// Create a Trackbar for user to enter threshold
    createTrackbar("Canny Threshold:", "img", &lowThreshold, 250, redraw);
    createTrackbar("HoughP Threshold:", "img", &houghThreshold, 250, redraw);
    createTrackbar("Min Line Length:", "img", &minLineLength, 150, redraw);
    createTrackbar("Max Line Gap:", "img", &maxLineGap, 100, redraw);
    createTrackbar("Line closeness (in mm)", "img", &lineDistThresh, 300, redraw);
    createTrackbar("Line angle (in degrees)", "img", &lineAngleThresh, 90, redraw);

    createTrackbar("Hough Threshold", "std hough", &shtThreshold, 200, redraw);


    // Initialize camera parameters -- TODO consolidate these
    cam.position.z = 37;
    cam.pan = 0;
    cam.tilt = -25.0/180*CV_PI;

    // Spin loop
    while (ros::ok()) {
        waitKey(5);
        ros::spinOnce();
    }
}
