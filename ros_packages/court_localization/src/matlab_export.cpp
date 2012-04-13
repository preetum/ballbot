// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
/*
 * ekf_localization.cpp
 *
 * AUTHOR: John Wang
 * VERSION: 0.1 (4 Sep 2011)
 *
 * DESCRIPTION:
 * Kalman filter for court localization
 */
#include <math.h>

#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <bb_msgs/OdometryStamped.h>

#include "backproject.h"
#include "particle_filter.h"
#include "findlines.h"
#include "geometry.h"

using namespace std;
using namespace cv;

// Globals
FILE *outfile = NULL;
vector<Vec2d> u;
vector<int> obsTimestep;
int t = 0;

int frameNo = 1;

void odometryCallback(const bb_msgs::OdometryStampedConstPtr& msg) {
    t += 1;

    // Control vector = [distance dtheta]'
    double dist = msg->odometry.distance_delta * 100,  // to cm
        dtheta = -1 * msg->odometry.heading_delta;

    u.push_back(Vec2d(dist, dtheta));
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    printf("\nProcessing frame %d\n", frameNo++);

    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        Mat &frame = cv_ptr->image;

        // Detect lines in the image
        vector<Vec4i> seenLines;
        findLines(frame, seenLines);

        // Transform visible lines to robot frame
        camera cam;
        cam.position.z = 33;  // fixed camera height = 33cm
        cam.pan = 0;
        cam.tilt = -15.6/180.0*CV_PI;

        // Draw lines in image coordinates
        int correspond[seenLines.size()];
        bool done = false,
            skip = false;

        while (!done) {
            // Prompt for correspondences
            for (int i = 0; i < seenLines.size(); i += 1) {
                Mat copy = frame.clone();
                Vec4i &l = seenLines[i];
                Scalar color(255, 0, 0);
                line( copy, Point(l[0], l[1]), Point(l[2], l[3]),
                      color, 2, 8 );
                imshow("image", copy);

                // Label with line index (1-9)
                char c = (char)cvWaitKey(0);
                if (c >= '1' && c <= '9') {
                    correspond[i] = c-'0';
                } else if (c == 's') {
                    printf("Skipping\n");
                    skip = true;
                    goto endloop;
                } else {
                    correspond[i] = 0;
                }
            }

            // Prompt for all lines
            printf("OK?\n");
            Mat copy = frame.clone();
            for (int i = 0; i < seenLines.size(); i += 1) {
                Vec4i &l = seenLines[i];
                int c = correspond[i];

                if (c != 0) {
                    Scalar color((char)(c * 8),
                                 (char)(c * 16),
                                 (char)(c * 32));
                    line( copy, Point(l[0], l[1]), Point(l[2], l[3]),
                          color, 2, 8 );
                }
            }
            imshow("image", copy);
            char c = (char)cvWaitKey(0);
            if (c == 'y' || c == 'Y') {
                done = true;
            }
        }

    endloop:

        if (skip)
            return;

        printf("Outputting\n");

        // Output all lines whose correspondence != 0
        fprintf(outfile, "%% observation at t=%d\n", t);
        obsTimestep.push_back(t);
        fprintf(outfile, "z{end+1} = [");
        for (int i = 0; i < seenLines.size(); i += 1) {
            Vec4i &line = seenLines[i];
            fprintf(outfile, "%d %d %d %d %d;\n",
                    line[0], line[1], line[2], line[3], correspond[i]);
        }
        fprintf(outfile, "]';\n");

        // Show frame for debug
        //drawLines(frame, seenLines);
        //imshow("image", frame);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "court_localization");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    const char* filename;

    if (argc == 2)
        filename = argv[1];
    else
        filename = "export.m";

    // Get input topic names from parameters
    string imageTopic, odomTopic, pfTopic, pfEstimateTopic;
    if (!nhPrivate.hasParam("image_transport"))
        nhPrivate.setParam("image_transport", "compressed");
    nhPrivate.param<string>("image", imageTopic, "camera/image");
    nhPrivate.param<string>("odometry", odomTopic, "odometry");

    // Create listeners
    ros::Subscriber odometrySub = 
        nh.subscribe(odomTopic, 200, odometryCallback);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imageSub = 
        it.subscribe(imageTopic, 50, imageCallback);


    outfile = fopen(filename, "w");
    if (outfile == NULL) {
        fprintf(stderr, "Could not open file\n");
        return 1;
    } else {
        printf("Opened file %s\n", filename);
    }
    fprintf(outfile, "z = {};\n");
    
    // Spin loop: service ROS callbacks
    ros::spin();
    
    // Write timesteps corresponding to observations
    fprintf(outfile, "\nztime = [");
    for (unsigned int i = 0; i < obsTimestep.size(); i += 1) {
        fprintf(outfile, "%d ", obsTimestep[i]);
    }
    fprintf(outfile, "];\n");

    // Write all received odometry as u
    fprintf(outfile, "\nu = [");
    for (unsigned int i = 0; i < u.size(); i += 1) {
        fprintf(outfile, "%f %f;\n", u[i][0], u[i][1]);
    }
    fprintf(outfile, "]';\n");

    fprintf(outfile, "\n%% total timesteps = %d\n", t);
    fclose(outfile);
}
