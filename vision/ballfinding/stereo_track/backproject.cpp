// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
/*
 * hill-climb-find-pose.cpp
 *
 *  Created on: Aug 16, 2011
 *      Author: ankush
 */

#include <cv.h>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <limits>

#include "backproject.h"

using namespace cv;
using namespace std;

Point3d get_camera_world_coordinates(Point3d real_world_position,
		                     Point3d camera_position,
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

	Point3d cam_world_position;
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

Point3d get_world_coordinates(Point3d cam_world_position,
                 	      Point3d camera_position,
			      double heading, double pan, double tilt)
{
	/* Returns position of a point in Real World Frame
	 *
	 * cam_world_position: the position of a point in camera's frame
	 * camera_position   : the position of the camera in the world frame
	 *
	 * Assumes:
	 * 	1. Roll of the camera = 0
	 */

	Point3d world_coordinates;
	double phi = heading + pan;
	double theta = tilt;

	double cos_phi = cos(phi), sin_phi = sin(phi), cos_t = cos(theta), sin_t = sin(theta);

	double x = cam_world_position.x,
		   y = cam_world_position.y,
		   z = cam_world_position.z;

	// Rotate
	world_coordinates.x = sin_phi*x      +   cos_phi*sin_t*y + cos_phi*cos_t*z;
	world_coordinates.y = (-1)*cos_phi*x +   sin_phi*sin_t*y + sin_phi*cos_t*z;
	world_coordinates.z =     0          + (-1)*cos_t*y      + sin_t*z;

	// Translate
	world_coordinates += camera_position;

	return world_coordinates;
}

Point2d cam_world_position_to_imageXY(Point3d cam_world_position, camera &bb_cam)
{
	/*
	 * Finds pixel(x,y) given the position of the point in camera's
	 * world and the camera position
	 *
	 * Arguments:
	 * cam_world_position: Position of the point in camera-world frame
	 * bb_cam            : Camera you are looking the point from
	 */

	Point2d imageXY;

	double x = cam_world_position.x,
		   y = cam_world_position.y,
		   z = cam_world_position.z;

	imageXY.x =  ((x/z)*bb_cam.intrinsics.fx + bb_cam.intrinsics.cx);
	imageXY.y =  ((y/z)*bb_cam.intrinsics.fy + bb_cam.intrinsics.cy);

	return imageXY;
}

Point3d image_plane_to_camera_world_position(Point2f pixel_pos, double depth,
					     camera cam)
{
	/* Returns the camera world position (3D) of a point
	 * whose projection in the image plane is known (pixel_pos) and
	 * its depth in camera-world is also known = depth
	 */

	Point3d cam_world_position;

	cam_world_position.z = depth;
	cam_world_position.x = (double)((pixel_pos.x - cam.intrinsics.cx)*
							(cam_world_position.z/cam.intrinsics.fx));
	cam_world_position.y = (double)((pixel_pos.y - cam.intrinsics.cy )*
							(cam_world_position.z/cam.intrinsics.fy));

	return cam_world_position;
}

