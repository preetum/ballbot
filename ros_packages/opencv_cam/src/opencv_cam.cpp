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
#include <stdlib.h>
#include <unistd.h>
#include <iostream>

#include <cv.h>
#include <highgui.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/SetCameraInfo.h>
#include <camera_calibration_parsers/parse_ini.h>

int main(int argc, char** argv) {
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

	image_transport::ImageTransport it(nh);
    std::string name("camera");
    name += argv[1];
    name += "/append";
	image_transport::CameraPublisher pub =
	    it.advertiseCamera(name, 1);

	std::cout << "Processing..." << std::endl;

	cv::Mat frame;
	cv::VideoCapture cam(atoi(argv[1]));
    cam.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

	while(nh.ok()) {
        // Capture frame
        //	    cam >> frame;
        cam.read(frame);
        //cv::undistort(frame_raw, frame, camera_info.K, camera_info.D);        
	    // Convert to ROS image message
	    int cols = frame.cols, rows = frame.rows;
	    sensor_msgs::Image msg;
	    msg.width = cols; 
	    msg.height = rows;
	    msg.encoding = "bgr8";
	    msg.is_bigendian = false;
	    msg.step = cols*3;
	    msg.data.resize(rows*cols*3);
        
        // std::cout<<msg.width<<" "<<msg.height<<std::endl;
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
	std::cout << "\nquitting..." << std::endl;

	return 0;
}
