// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
/*
 * video_publisher.cpp
 *
 * AUTHOR: John Wang
 * VERSION: 0.1 (6 Sep 2011)
 *
 * DESCRIPTION:
 * Publishes a video file as a ROS topic, using image_transport.
 */
#include <stdio.h>

#include <cv.h>
#include <highgui.h>

#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

int main(int argc, char** argv) {
    ros::init(argc, argv, "video_publisher");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    if (argc < 2) {
        fprintf(stderr, "Usage: %s videofile\n", argv[0]);
        return 1;
    }

    // Open file
    cv::VideoCapture capture(argv[1]);
    if (!capture.isOpened()) {
        ROS_ERROR("Could not open file: %s", argv[1]);
        return 1;
    }

    // Setup image publisher
    image_transport::ImageTransport it(nh);
    image_transport::Publisher pub = it.advertise("camera/image", 1);

    // Main loop publishes at rate ~fps (default 30)
    int fps;
    nhPrivate.param<int>("fps", fps, 30);
    ros::Rate loopRate(fps);
    while (ros::ok()) {
        cv::Mat frame;

        // Grab new frame
        capture >> frame;

        // Publish image message
        cv_bridge::CvImage img;
        img.encoding = "bgr8";
        img.image = frame;
        pub.publish(img.toImageMsg());
    }

    capture.release();
}
