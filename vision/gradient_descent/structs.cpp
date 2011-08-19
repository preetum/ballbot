/*
 * structs.cpp
 *
 *  Created on: Aug 17, 2011
 *      Author: ankush
 */

using namespace cv;

struct variances
{
	double x, y, len, angle;
	variances()
	{
		x = 20*20; // pixels-squared
		y = 20*20; // pixels-squared
		len = 50*50; // pixels-squared
		angle = ((pi*pi)/(7.0*7.0)); // radians-squared
	}
};

struct camera_intrinsic_values  //stores camera intrinsics
{	// TODO: Add undisortion parameters
	double fx, fy, cx, cy;

	camera_intrinsic_values()
	{
		fx = 522.1019521747;
		cx = 299.0472018951;
		fy = 524.4051288723;
		cy = 242.6572277189;
	}
};

struct camera
{
	/*
	 *x,y,z are in centimeters
	 *theta, pan, tilt are in radians
	 */

	Point3d position;
	double  theta, pan, tilt;
	camera_intrinsic_values intrinsics;

	camera() // Initialized for capture1...backcorner.avi
	{

		position.x = 0;
		position.y = 500;
		position.z = 133; // Camera is 33cm above the ground
		theta = 0;
		pan = 0;
		tilt = 175;
	}

};

struct line_segment_3d
{
	Point3d pt1, pt2;

	line_segment_3d()
	{
		pt1.x = 0; pt1.y = 0; pt1.z = 0;
		pt2.x = 0; pt2.y = 0; pt2.z = 0;
	}
};

struct line_segment_2d
{
	Point2d pt1, pt2;

	line_segment_2d()
	{
		pt1.x = 0;  pt1.y = 0;
		pt2.x = 0;  pt2.y = 0;
	}
};

struct line_candidate
{
	Point2d center;
	double length;
	double angle;

	line_candidate()
	{
		center.x = 0; center.y = 0;
		length =0;
		angle =0;
	}
};

struct line_segment_all_frames
{
	/*
	 * Structure to hold the coordinates of a line segment
	 * in 3 frames viz.:
	 * 		1. Image plane
	 * 		2. Camera world
	 * 		3. Real World
	 */

	line_segment_2d imgPlane;
	line_segment_3d camWorld;
	line_segment_3d realWorld;
};
