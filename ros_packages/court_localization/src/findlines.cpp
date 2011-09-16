// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include <cv.h>
#include <highgui.h>

#include "geometry.h"

using namespace std;
using namespace cv;


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

    // Color threshold to find white lines
    //  TODO tune thresholds
    threshold(frame_gray, frame_thresh, 210, 255, THRESH_BINARY);

    // Probabilistic Hough transform
    vector<Vec4i> lines;
    HoughLinesP(frame_thresh, lines, 2, CV_PI/180, 80, 40, 10 );

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
                double a = pointLineDistance(Vec2i(line[0], line[1]), group) +
                    pointLineDistance(Vec2i(line[2], line[3]), group);
                double b = abs(normalizeRadians(lineAngle(line) -
                                                lineAngle(group)));

                // Include all lines within 1 std deviation using the
                //  pseudo-Gaussian distribution
                //double closeness = exp(-a/15.0 + -b/0.15);
                //if (closeness > 0.367) {
                if (a < 15 && b < 0.1) {

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

/*
int main(int argc, char **argv)
{
    bool run = true;
    Mat frame, out;
    const char *videoCaptureName = NULL;

    // Parse command line args
    int c;
    while ((c = getopt(argc, argv, "c:dpvt")) != -1) {
        switch (c) {
        case 'c':
            videoCaptureName = optarg;
            break;
        case 'd':
            display = true;
            break;
        case 'p':
            pause = true;
            break;
        case 'v':
            verbose = true;
            break;
        case 't':
            test();
            return 0;
        default:
            return -1;
        }
    }

    // Create windows
    if (display) {
        namedWindow("img", CV_WINDOW_AUTOSIZE);
        cvMoveWindow("img", 50, 50);
        //cvNamedWindow("gray", 0);
        //cvMoveWindow("gray", 400, 50);
        //cvNamedWindow("thresh", 0);
        //cvMoveWindow("thresh", 50, 350);
        //cvNamedWindow("edges", 0);
        //cvMoveWindow("edges", 400, 350);
    }

    // Arguments remaining: load image file
    if (optind < argc) {
        frame = imread(argv[optind]);
        if (frame.data == NULL) {
            printf("could not open file: %s\n", argv[optind]);
            return -1;
        }
        resize(frame, frame, Size(480, 640));

        vector<Vec4i> lines;
        findLines(frame, lines);
        printf("\n%d lines found\n", lines.size());
        waitKey(0);
    }
    // Use video capture
    else if (videoCaptureName) {
        VideoCapture capture(videoCaptureName);
        if (capture.isOpened()) {
            //capture.set(CV_CAP_PROP_POS_FRAMES, 500);
        } else {
            // Try opening webcapture by parsing argument as an int
            capture.open(atoi(videoCaptureName));
            if (!capture.isOpened()) {
                fprintf(stderr, "could not open captureera\n");
                return -1;
            }

            // Configure captureera capture params
            capture.set(CV_CAP_PROP_FRAME_WIDTH, 640);
            capture.set(CV_CAP_PROP_FRAME_HEIGHT, 480);
        }

        printf("initialized video capture\n");
        capture >> frame;
        printf("frame size: %dx%d channels: %d\n", frame.cols, frame.rows,
               frame.channels());

        // Discard some frames
        for (int i = 0; i < 61; i += 1)
            capture >> frame;

        while (1) {
            if (run) {
                vector<Vec4i> lines;
                capture >> frame;
                process(frame, lines);

                if (display) {
                    drawLines(frame, lines);
                    imshow("img", frame);
                    if (pause) waitKey();
                }
            }

            switch ((char)waitKey(10)) {
            case 'q':
                return 0;
            case ' ':
                run = !run;
                break;
            }
        }
    }
}
*/
