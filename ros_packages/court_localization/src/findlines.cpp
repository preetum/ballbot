// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include <math.h>

#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace cv;

// Globals
bool verbose = false;
bool display = false;
bool pause = false;

/* Normalize an angle to the range (-pi, pi] */
double normalizeRadians(double rad) {
    while (rad > CV_PI)
        rad -= 2*CV_PI;
    while (rad <= -CV_PI)
        rad += 2*CV_PI;
    return rad;
}

Vec2i lineCenter(const Vec4i &line) {
    return Vec2i((line[0]+line[2]) / 2, (line[1]+line[3]) / 2);
}

/* Returns the angle of the LINE which passes through
 * (x1, y1, x2, y2).
 *
 * Angle is given in radians counterclockwise from the x-axis,
 *  in the range (-pi/2, pi/2].
 */
double lineAngle(const Vec4i &line) {
    double y = line[3] - line[1],
        x = line[2] - line[0],
        theta = atan2(y, x);

    // Normalize answer to (-pi/2, pi/2]
    if (theta > CV_PI/2)
        theta -= CV_PI;
    else if (theta <= -CV_PI/2)
        theta += CV_PI;
    return theta;
}

/* Returns the distance from POINT (x,y) to the LINE
 * which passes through points (x1, y1, x2, y2)
 *
 * Reference: http://mathworld.wolfram.com/Point-LineDistance2-Dimensional.html
 */
double pointLineDistance(const Vec2i &point, const Vec4i &line) {
    // Project point onto line
    // n is a unit normal vector of the line
    // v is a vector from point to line
    Vec2d n(line[1]-line[3], line[2]-line[0]);
    n *= 1.0/norm(n);
    Vec2d v(line[0]-point[0], line[1]-point[1]);

    return abs(n.dot(v));
}

/* Returns the distance from POINT (x,y) to line segment 
 * SEGMENT whose endpoints are (x1, y1, x2, y2)
 *
 * Reference: http://stackoverflow.com/questions/627563/702174#702174
 */
double pointLineSegmentDistance(const Vec2i &point, const Vec4i &segment) {
    Vec2d p(point);
    Vec2d r(segment[0], segment[1]),
        s(segment[2], segment[3]);

    // Project point p onto line rs
    // n is a unit normal vector of line rs
    // v is a vector from p to line rs
    // d is the distance from p to line rs
    Vec2d n(r[1]-s[1], s[0]-r[0]);
    n *= 1.0/norm(n);
    Vec2d v = r - p;
    double d = n.dot(v);

    // We can parameterize the line rs as L(u) = r + (s-r)*u
    // then solve for the value of u where the projection of p lies
    // that is: L(u) = r + (s-r)* u = p + d*n
    // ||s-r||^2 * u = (s-r) * (p - r + d*n)
    //             u = (s-r) * (p - r + d*n) / ||s-r||^2
    Vec2d x = s - r;
    double u = x.dot(p - r + d*n) / x.dot(x);

    // If projection of p lies on line segment rs, return the distance
    if (0.0 <= u && u <= 1.0) {
        return abs(d);
    }
    // Otherwise return the distance to the closest endpoint
    else {
        double a = norm(p-s),
            b = norm(p-r);
        return min(a, b);
    }
}

/* Given an input image frame, returns a set of court line candidates
 * in groupedLines.
 *
 * Each line is represented as a two endpoints of a line segment,
 * in pixel coordinates (x1, y1, x2, y2).
 */
void findLines(Mat &frame, vector<Vec4i> &groupedLines) {
    Mat frame_save, frame_gray, frame_thresh, frame_edges;

    // Resize input image
    if (frame.cols != 640)
        resize(frame, frame, Size(640, 480));

    // Threshold by distance: blank out all top pixels
    rectangle(frame, Point(0,0), Point(640, 180), Scalar(0,0,0), CV_FILLED);

    // Convert to grayscale
    cvtColor(frame, frame_gray, CV_RGB2GRAY, 1);

    // Increase contrast, decrease brightness
    frame_gray.convertTo(frame_gray, -1, 1.6, -120);

    // Color threshold to find white lines
    //  TODO tune thresholds
    threshold(frame_gray, frame_thresh, 190, 255, THRESH_BINARY);

    // Use Canny edge detection to find edges
    //Canny(frame_thresh, frame_edges, 50, 200);
    /*
      if (display) {
      imshow("gray", frame_gray);
      imshow("thresh", frame_thresh);
      //imshow("edges", frame_edges);
      }
    */

    //frame_save = frame.clone();

    // Hough transform
    vector<Vec4i> lines;
    HoughLinesP(frame_thresh, lines, 2, CV_PI/180, 80, 40, 10 );

    /*
    // Draw all lines
    for( size_t i = 0; i < lines.size(); i++ ) {
    const Vec4i &l = lines[i];
    line( frame, Point(l[0], l[1]), Point(l[2], l[3]),
    Scalar(0,0,255), 1, 8 );
    }
    imshow("img", frame);
    waitKey();
    //*/

    // Consolidate duplicate lines
    vector<int> numVotes;
    RNG rng(0L);
    while (!lines.empty()) {
        //vector<Vec4i> groupMembers;
        Vec4i group;
        int lineCount;
        int lastLineCount;

        // Select the last line and try to find all matching line segments
        group = lines.back();
        lines.pop_back();
        lineCount = 1;
        //groupMembers.push_back(group);

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
                    //groupMembers.push_back(line);

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

        /*
        // Draw group in random color
        frame = frame_save.clone();
        //Scalar color(rng(256), rng(256), rng(256));
        Scalar color(0, 0, 255);
        for( size_t i = 0; i < groupMembers.size(); i++ ) {
        const Vec4i &l = groupMembers[i];
        line( frame, Point(l[0], l[1]), Point(l[2], l[3]),
        color, 1, 8 );
        printf("Line (%d,%d) to (%d,%d)\n",
        l[0], l[1], l[2], l[3]);
        }
        // Draw grouped line
        printf("Grouped Line (%d,%d) to (%d,%d)\n",
        group[0], group[1], group[2], group[3]);
        line( frame, Point(group[0], group[1]),
        Point(group[2], group[3]),
        Scalar(0,255,0), 1, 8 );

        imshow("img", frame);
        printf("%d votes\n\n", lineCount);
        // Pause
        waitKey();
        //*/

        groupedLines.push_back(group);
        numVotes.push_back(lineCount);
    } // while (!lines.empty())

    // Draw grouped lines
    printf("\n%d lines found\n", groupedLines.size());
    if (display) {
        for( size_t i = 0; i < groupedLines.size(); i++ ) {
            const Vec4i &l = groupedLines[i];
            Scalar color(rng(256), rng(256), rng(256));
            
            line( frame, Point(l[0], l[1]), Point(l[2], l[3]),
                  color, 2, 8 );
            printf("Line (%d,%d) to (%d,%d) [votes: %d]\n",
                   l[0], l[1], l[2], l[3], numVotes[i]);
        }
        imshow("img", frame);
        if (pause) waitKey();
    }
} // findLines()

void test() {
    Vec4i a(1,2,3,4);
    Vec4i b = a;
    b[0]=-1;
    printf("a =(%d,%d,%d,%d)\n", a[0], a[1], a[2], a[3]);
    printf("b =(%d,%d,%d,%d)\n", b[0], b[1], b[2], b[3]);
    printf("%f\n", pointLineDistance(Vec2i(0,0), Vec4i(-1,0, 1,0)));
    printf("%f\n", pointLineDistance(Vec2i(3,3), Vec4i(-1,0, 1,0)));
    printf("%f\n", pointLineDistance(Vec2i(1,0), Vec4i(-1,-1, 1,1)));
    printf("%f\n", pointLineDistance(Vec2i(0,1), Vec4i(-1,-1, 1,1)));
}

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
        /*
          cvNamedWindow("gray", 0);
          cvMoveWindow("gray", 400, 50);
          cvNamedWindow("thresh", 0);
          cvMoveWindow("thresh", 50, 350);
          cvNamedWindow("edges", 0);
          cvMoveWindow("edges", 400, 350);
        */
    }

    // Arguments remaining: load image file
    if (optind < argc) {
        frame = imread(argv[optind]);
        if (frame.data == NULL) {
            printf("could not open file: %s\n", argv[optind]);
            return -1;
        }
        resize(frame, frame, Size(480, 640));
        findLines(frame, vector<Vec4i>());
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
                    imshow("img", frame);
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
