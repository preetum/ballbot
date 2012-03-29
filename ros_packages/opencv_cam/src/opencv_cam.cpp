// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
/*
 * opencv_cam.cpp
 *
 * AUTHOR: John Wang
 * VERSION: 0.1 (10 Oct 2011)
 *
 * DESCRIPTION:
 * Uses OpenCV's VideoCapture class to get fast video frame input, then
 *  publishes to a ROS topic. On Linux it uses fast V4L2 memory-mapped
 *  IO calls.
 */
#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <string.h>
#include <boost/lexical_cast.hpp>

#include <cv.h>
#include <highgui.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse_ini.h>

void print_usage() {
    fprintf(stderr, "opencv_cam [-s WIDTHxHEIGHT] [-c INDEX] [-v]\n");
    fprintf(stderr, "\t-s\tset image size to WIDTHxHEIGHT\n");
    fprintf(stderr, "\t-c\tset camera index to INDEX\n");
    fprintf(stderr, "\t-v\tverbose mode\n\n");
}

int main(int argc, char* argv[]) {
	ros::init(argc, argv, "opencv_publisher", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

	// Read camera calibration info
    std::string calib_filename;
	std::string camera_name;
    sensor_msgs::CameraInfo camera_info;
    nh_private.param<std::string>("calib", calib_filename,
                                  "camera_parameters.txt");
	if (camera_calibration_parsers::
	    readCalibrationIni(calib_filename, camera_name, camera_info)) {
        ROS_INFO("Successfully read camera calibration.  "
                 "Rerun camera calibrator if it is incorrect.");
	}
	else {
        ROS_ERROR("No camera_parameters.txt file found.  "
                  "Use default file if no other is available.");
	}

    // Parse command line options
    int camera_index = 0;
    int image_width = 640;
    int image_height = 480;
    int verbose = 0;

    int c;
    char *pch;
    opterr = 0;
    while ((c = getopt (argc, argv, "c:s:vh")) != -1) {
        switch (c) {
        case 'v':
            verbose = 1;
            break;
        case 'c':
            camera_index = atoi(optarg);
            break;
        case 's':
            pch = strtok(optarg, "xX");
            if (pch == NULL) {
                fprintf(stderr, "Could not parse image size\n");
                return 1;
            }
            image_width = atoi(pch);
            pch = strtok(NULL, "xX");
            if (pch == NULL) {
                fprintf(stderr, "Could not parse image size\n");
                return 1;
            } 
            image_height = atoi(pch);
            break;
        case 'h':
            print_usage();
            return 1;
        case '?':
            if (optopt == 'c' || optopt == 's')
                fprintf(stderr, "Option -%c requires an argument.\n", optopt);
            else if (isprint (optopt))
                fprintf(stderr, "Unknown option `-%c'.\n", optopt);
            else
                fprintf(stderr,
                        "Unknown option character `\\x%x'.\n",
                        optopt);
            print_usage();
            return 1;
        default:
            abort ();
        }
    }

    if (verbose) {
        fprintf(stderr, "Camera index: %d\n", camera_index);
        fprintf(stderr, "Image size: %dx%d\n", image_width, image_height);
    }

    // Initialize ROS publisher
	image_transport::ImageTransport it(nh);
    std::string name("camera");
    name += boost::lexical_cast<std::string>(camera_index);
    name += "/image_raw";
	image_transport::CameraPublisher pub =
	    it.advertiseCamera(name, 1);

    // Convert camera / distortion matrices to OpenCV format
    double K[3][3];
    for (int i = 0; i < 9; i += 1)
        K[i/3][i%3] = camera_info.K[i];
    double D[camera_info.D.size()];
    for (int i = 0; i < camera_info.D.size(); i += 1)
        D[i] = camera_info.D[i];
    cv::Mat cameraMatrix = cv::Mat(3, 3, CV_64F, K).inv();
    cv::Mat distCoeffs = cv::Mat(1, camera_info.D.size(), CV_64F, D);

    if (verbose) {
        fprintf(stderr, "Camera Matrix:\n");
        for (int i = 0; i < 3; i += 1)
            fprintf(stderr, "%f %f %f\n", cameraMatrix.at<double>(i,0),
                    cameraMatrix.at<double>(i,1),
                    cameraMatrix.at<double>(i,2));

        fprintf(stderr, "Distortion Coefficients:\n");
        for (int i = 0; i < camera_info.D.size(); i += 1)
            fprintf(stderr, "%f ", distCoeffs.at<double>(i));
        fprintf(stderr, "\n");
    }


    // Open VideoCapture device and set properties
	cv::Mat frame_raw;
    cv::Mat frame;
	cv::VideoCapture cam(camera_index);
    cam.set(CV_CAP_PROP_FRAME_WIDTH, image_width);
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, image_height);

    // Print this message last
    ROS_INFO("Initialized. Publishing to topic %s", name.c_str());

	while(nh.ok()) {
        // Capture frame
        cam.read(frame);
        //cv::undistort(frame_raw, frame, cameraMatrix, distCoeffs);

	    // Convert to ROS image message
	    int cols = frame.cols, rows = frame.rows;
	    sensor_msgs::Image msg;
	    msg.width = cols; 
	    msg.height = rows;
	    msg.encoding = "bgr8";
	    msg.is_bigendian = false;
	    msg.step = cols*3;
	    msg.data.resize(rows*cols*3);
        
	    // Copy raw pixels to image message data
	    if (frame.isContinuous()) {
            cols *= rows;
            rows = 1;
	    }
	    for (int i = 0; i < rows; i++) {
            const unsigned char* row = frame.ptr<unsigned char>(i);
            std::copy(row, row+(cols*3), msg.data.begin() + (i*cols*3));
	    }

        // Publish ROS image
	    pub.publish(msg, camera_info);

	    ros::spinOnce();
	}

	//close out
    ROS_INFO("Quitting...");

	return 0;
}
