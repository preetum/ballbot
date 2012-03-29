// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
/*
 * stereo_opencv_cam.cpp
 *
 * AUTHOR: Ankush Gupta
 * BASED ON: opencv_cam.cpp | author: John Wang
 * VERSION: 0.1 (28 March 2012)
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

#define WIDTH 640
#define HEIGHT 480


class image_publisher {
    private:
    image_transport::ImageTransport it;
    std::string name;
    image_transport::CameraPublisher pub;
	cv::Mat frame;
	cv::VideoCapture cam;
    sensor_msgs::Image msg;

    public:
    image_publisher(const ros::NodeHandle &nh,
                    std::string cam_index,
                    std::string cam_name = "camera"):
        it(nh),
        name(cam_name + cam_index + "/image_raw"),
        pub(it.advertiseCamera(name, 1)),
        cam(cv::VideoCapture(atoi(cam_index.c_str()))) {
        if (cam.isOpened())
            ROS_INFO("Processing %s ...", name.c_str());
        else
            ROS_ERROR("Could not connect to the camera%d device.", atoi(cam_index.c_str()));
      
                cam.set(CV_CAP_PROP_FRAME_WIDTH, WIDTH);
                cam.set(CV_CAP_PROP_FRAME_HEIGHT, HEIGHT);
                msg.encoding = "bgr8";
                msg.is_bigendian = false;
    }
            
    /** Grabs a frame, converts it to ROS image message format
        and pubishes it on the topic CAM_INFO.*/
    void grab_frame_publish(sensor_msgs::CameraInfo *cam_info) {
        // Capture frame
        cam.read(frame);

        //undistort
        //cv::undistort(frame_raw, frame, camera_info.K, camera_info.D);        

        // Convert to ROS image message
	    msg.width = frame.cols; 
        msg.height = frame.rows;
	    msg.step = msg.width*3;
	    msg.data.resize(msg.height*msg.width*3);

        int cols = frame.cols, rows = frame.cols;
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
	    pub.publish(msg, *cam_info);
    }
};



int main(int argc, char* argv[]) {
	ros::init(argc, argv, "opencv_publisher", ros::init_options::AnonymousName);
	ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

	// Read camera calibration info
    std::string calib_filename_left, calib_filename_right;
    sensor_msgs::CameraInfo cam_info_right, cam_info_left;

    nh_private.param<std::string>("calib_left", calib_filename_left,
                                  "../camera_parameters_left.txt");
    nh_private.param<std::string>("calib_right", calib_filename_right,
                                  "../camera_parameters_right.txt");

    std::string cam_name_right, cam_name_left;
    if (camera_calibration_parsers::
	    readCalibrationIni(calib_filename_right, cam_name_right, cam_info_right)
        && camera_calibration_parsers::
	    readCalibrationIni(calib_filename_left, cam_name_left, cam_info_left)) {
	  ROS_INFO("Successfully read camera calibration.  "
		   "Rerun camera calibrator if it is incorrect.");
	}
	else {
	  ROS_ERROR("Error in finding the camera parameters files. Skipping ...");
	}

    image_publisher left(nh, argv[1]);
    image_publisher right(nh, argv[2]);

	while(nh.ok()) {
        left.grab_frame_publish(&cam_info_left);
        right.grab_frame_publish(&cam_info_right);
	    ros::spinOnce();
	}

	//close out
	std::cout << "\nquitting..." << std::endl;
	return 0;
}
