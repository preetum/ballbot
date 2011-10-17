/*
 * camera.cpp
 *
 *  Created on: Sep 11, 2011
 *      Author: ankush
 */
#include "camera.h"

cv::Point2d cam_world_position_to_imageXY(cv::Point3d cam_world_position, camera &bb_cam)
{
	/*
	 * Finds pixel(x,y) given the position of the point in camera's
	 * world and the camera position
	 *
	 * Arguments:
	 * cam_world_position: Position of the point in camera-world frame
	 * bb_cam            : Camera you are looking the point from
	 */
  
  cv::Point2d imageXY;
  
  double x = cam_world_position.x,
    y = cam_world_position.y,
    z = cam_world_position.z;
  
  imageXY.x =  ((x/z)*bb_cam.intrinsics.fx + bb_cam.intrinsics.cx);
  imageXY.y =  ((y/z)*bb_cam.intrinsics.fy + bb_cam.intrinsics.cy);
  
  return imageXY;
}


cv::Point3d get_camera_world_coordinates(cv::Point3d real_world_position,
		                     cv::Point3d camera_position,
				     double heading, double pan,
				     double tilt)
{
	/* Returns position of a point in the Camera's World Frame
	 * -------------------------------------------------------
	 * Arguments:
	 *
	 * real_world_position  : is the 3D position of a point in world frame
	 * camera_position      : is the 3D positi6on of the camera in the world frame
	 * heading 	        : the heading of the robot
	 * pan			: pan angle of the camera
	 * tilt			: tilt angle of the camera
	 *
	 * linear units: Centimeters
	 * angluar units: radians
	 *
	 * Assumes:
	 * 	1. that roll angle = 0
	 */

	cv::Point3d cam_world_position;
	double phi = heading + pan;
	double theta = tilt;

	double cos_phi = cos(phi), sin_phi = sin(phi), cos_t = cos(theta), sin_t = sin(theta);
	double x = real_world_position.x,
		   y = real_world_position.y,
		   z = real_world_position.z;

	// Translation
	x -= camera_position.x;
	y -= camera_position.y;
	z -= camera_position.z;

	// Rotation
	cam_world_position.x  =  sin_phi*x        +  -1*cos_phi*y        +   0;
	cam_world_position.y  =  cos_phi*sin_t*x  +  sin_phi*sin_t*y     +  -1*cos_t*z;
	cam_world_position.z  =  cos_phi*cos_t*x  +  sin_phi*cos_t*y     +   sin_t*z;

	return cam_world_position;
}
