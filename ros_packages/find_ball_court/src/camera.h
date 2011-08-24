/*
 * camera.h
 *
 *  Created on: Sep 11, 2011
 *      Author: ankush
 */

#ifndef CAMERA_H_
#define CAMERA_H_

#include <opencv2/core/core.hpp>
const double PI = 3.141592654;

struct camera_intrinsic_values  //stores camera intrinsics
{	// TODO: Add undisortion parameters
	double fx, fy, cx, cy;

	camera_intrinsic_values()
	{
	  /**
		fx = 522.1019521747;
		cx = 299.0472018951;
		fy = 524.4051288723;
		cy = 242.6572277189;
	  **/
	  fx = 261;
	  cx = 150;
	  fy = 262;
	  cy = 121;
	}
};

struct camera
{
	/*
	 *x,y,z are in centimeters
	 *theta, pan, tilt are in radians
	 */

	cv::Point3d position;
	double  theta, pan, tilt;
	camera_intrinsic_values intrinsics;

	camera() // Initialized for capture1...backcorner.avi
	{

		position.x = 0;
		position.y = 0;
		position.z = 33; // Camera is 33cm above the ground
		theta = 0;
		pan = 0;
		tilt = -18.8*PI/180.0; //<<<<< UPDATE CAMERA TILT angle here
	}
};


//--------------------------function Prototypes---------------------------
cv::Point2d cam_world_position_to_imageXY(cv::Point3d cam_world_position, camera &bb_cam);
cv::Point3d get_camera_world_coordinates(cv::Point3d real_world_position,
										 cv::Point3d camera_position, double heading,
										 double pan, double tilt);
//------------------------------------------------------------------------



#endif /* CAMERA_H_ */
