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

void odometryCallback(const bb_msgs::OdometryStampedConstPtr& msg) {
    t += 1;

    // Control vector = [distance dtheta]'
    double dist = msg->odometry.distance_delta * 100,  // to cm
        dtheta = -1 * msg->odometry.heading_delta;

    u.push_back(Vec2d(dist, dtheta));
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
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

        // Draw lines in ground coordinates (robot frame)
        fprintf(outfile, "%% observation at t=%d\n", t);
        obsTimestep.push_back(t);
        fprintf(outfile, "z{end+1} = [");
        for (vector<Vec4i>::iterator it = seenLines.begin();
             it < seenLines.end(); it += 1) {
            Vec4i &line = *it;

            Point2d pt1 = cameraPointToRobot(Point2d(line[0], line[1]), cam),
                pt2 = cameraPointToRobot(Point2d(line[2], line[3]), cam);

            fprintf(outfile, "%f %f %f %f;\n", pt1.x, pt1.y, pt2.x, pt2.y);
        }
        fprintf(outfile, "]';\n");

        // Show frame for debug
        drawLines(frame, seenLines);
        imshow("image", frame);
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
        it.subscribe(imageTopic, 1, imageCallback);


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
