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

// globals
sensor_msgs::CameraInfo camera_info;

bool setCameraInfo(sensor_msgs::SetCameraInfo::Request &req, sensor_msgs::SetCameraInfo::Response &rsp) {

  ROS_INFO("New camera info received");
  camera_info = req.camera_info;

  if (camera_calibration_parsers::writeCalibrationIni("../camera_parameters.txt", "gscam", camera_info)) {
    ROS_INFO("Camera information written to camera_parameters.txt");
    return true;
  }
  else {
    ROS_ERROR("Could not write camera_parameters.txt");
    return false;
  }
}


int main(int argc, char** argv) {

	// We could probably do something with the camera name, check
	// errors or something, but at the moment, we don't care.
	std::string camera_name;
	if (camera_calibration_parsers::readCalibrationIni("../camera_parameters.txt", camera_name, camera_info)) {
	  ROS_INFO("Successfully read camera calibration.  Rerun camera calibrator if it is incorrect.");
	}
	else {
	  ROS_ERROR("No camera_parameters.txt file found.  Use default file if no other is available.");
	}

	ros::init(argc, argv, "opencv_publisher");
	ros::NodeHandle nh;

	image_transport::ImageTransport it(nh);
	image_transport::CameraPublisher pub =
	    it.advertiseCamera("gscam/image_raw", 1);

	ros::ServiceServer set_camera_info =
	    nh.advertiseService("gscam/set_camera_info", setCameraInfo);

	std::cout << "Processing..." << std::endl;

	cv::Mat frame;
	cv::VideoCapture cam(0);
	while(nh.ok()) {
	    cam >> frame;

	    // Send ROS image message
	    int cols = frame.cols, rows = frame.rows;
	    sensor_msgs::Image msg;
	    msg.width = cols; 
	    msg.height = rows;
	    msg.encoding = "rgb8";
	    msg.is_bigendian = false;
	    msg.step = cols*3;
	    msg.data.resize(rows*cols*3);

	    // Copy frame to image message data
	    if (frame.isContinuous()) {
		cols *= rows;
		rows = 1;
	    }
	    size_t offset = 0;
	    for (int i = 0; i < rows; i++) {
		const unsigned char* row = frame.ptr<unsigned char>(i);
		std::copy(row, row+(cols*3), msg.data.begin() + offset);
		offset += cols*3;
	    }

	    pub.publish(msg, camera_info);

	    ros::spinOnce();
	}

	//close out
	std::cout << "\nquitting..." << std::endl;

	return 0;
}
