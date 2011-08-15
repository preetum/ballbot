/*
 * backproject.cpp
 *
 *  Created on: Aug 11, 2011
 *      Author: ankush
 */

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <limits>

using namespace cv;
using namespace std;

const double pi = 3.1415926535897;
double inf =  numeric_limits<double>::infinity();

struct line_segment
{
	Point3f pt1, pt2;

	line_segment()
	{
		pt1.x = 0; pt1.y = 0; pt1.z = 0;
		pt2.x = 0; pt2.y = 0; pt2.z = 0;
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

	Point3f position;
	float  theta, pan, tilt;
	camera_intrinsic_values intrinsics;

	camera() // Initialized for capture1...backcorner.avi
	{

		position.x = 0;
		position.y = 500;
		position.z = 100; // Camera is 33cm above the ground
		theta = 5.0*pi/180.0;
		pan = 2.0*pi/180.0;
		tilt = -5*pi/180.0;
	}

} bb_camera;

//Trackbar variables
Mat frame  = Mat::zeros(480, 640, CV_8UC3);

int x_pos = bb_camera.position.x,
	y_pos = bb_camera.position.y,
	z_pos = bb_camera.position.z,
	pan = bb_camera.pan,
	tilt = bb_camera.tilt;

struct _court
{
	vector <line_segment> court_lines;

	_court()
	{
		line_segment line_seg;

		line_seg.pt1.x = 0; line_seg.pt1.y = 0; line_seg.pt1.z = 0;
		line_seg.pt2.x = 0; line_seg.pt2.y = 1097; line_seg.pt2.z = 0;
		court_lines.push_back(line_seg);

		line_seg.pt1.x = 548; line_seg.pt1.y = 137; line_seg.pt1.z = 0;
		line_seg.pt2.x = 548; line_seg.pt2.y = 960; line_seg.pt2.z = 0;
		court_lines.push_back(line_seg);

		line_seg.pt1.x = 548; line_seg.pt1.y = 137; line_seg.pt1.z = 0;
		line_seg.pt2.x = 548; line_seg.pt2.y = 960; line_seg.pt2.z = 0;
		court_lines.push_back(line_seg);

		line_seg.pt1.x = 1188.5; line_seg.pt1.y = 0; line_seg.pt1.z = 99;
		line_seg.pt2.x = 1188.5; line_seg.pt2.y = 1097; line_seg.pt2.z = 99;
		court_lines.push_back(line_seg);

		line_seg.pt1.x = 1829; line_seg.pt1.y = 137; line_seg.pt1.z = 0;
		line_seg.pt2.x = 1829; line_seg.pt2.y = 960; line_seg.pt2.z = 0;
		court_lines.push_back(line_seg);

		line_seg.pt1.x = 2377; line_seg.pt1.y = 0; line_seg.pt1.z = 0;
		line_seg.pt2.x = 2377; line_seg.pt2.y = 1097; line_seg.pt2.z = 0;
		court_lines.push_back(line_seg);

		line_seg.pt1.x = 0; line_seg.pt1.y = 0; line_seg.pt1.z = 0;
		line_seg.pt2.x = 2377; line_seg.pt2.y = 0; line_seg.pt2.z = 0;
		court_lines.push_back(line_seg);

		line_seg.pt1.x = 0; line_seg.pt1.y = 137; line_seg.pt1.z = 0;
		line_seg.pt2.x = 2377; line_seg.pt2.y = 137; line_seg.pt2.z = 0;
		court_lines.push_back(line_seg);

		line_seg.pt1.x = 548; line_seg.pt1.y = 548; line_seg.pt1.z = 0;
		line_seg.pt2.x = 1829; line_seg.pt2.y = 548; line_seg.pt2.z = 0;
		court_lines.push_back(line_seg);

		line_seg.pt1.x = 0; line_seg.pt1.y = 960; line_seg.pt1.z = 0;
		line_seg.pt2.x = 2377; line_seg.pt2.y = 960; line_seg.pt2.z = 0;
		court_lines.push_back(line_seg);

		line_seg.pt1.x = 0; line_seg.pt1.y = 1097; line_seg.pt1.z = 0;
		line_seg.pt2.x = 2377; line_seg.pt2.y = 1097; line_seg.pt2.z = 0;
		court_lines.push_back(line_seg);

	}
} court;


Point3d get_camera_world_coordinates(Point3d real_world_position,
									 Point3d camera_position,
									 double heading, double pan,
									 double tilt)
{
	/*Returns position of a point in the Camera's World Frame
	 *
	 *Assuming:
	 *
	 * world_position  : is the 3D position of a point in world frame
	 * camera_position : is the 3D positi6on of the camera in the world frame
	 *
	 * linear units: Centimeters
	 * angluar units: radians
	 *
	 * 	 1. that height of the camera in the world is fixed = 33 above the gound.
	 * 	 2. that roll angle = 0
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
	cam_world_position.x  =  sin_phi*x  +  -1*cos_phi*y        +   0;
	cam_world_position.y  =  cos_phi*sin_t*x  +  sin_phi*sin_t*y     +  -1*cos_t*z;
	cam_world_position.z  =  cos_phi*cos_t*x  +  sin_phi*cos_t*y     +   sin_t*z;

	return cam_world_position;
}

Point2f cam_world_position_to_imageXY(Point3d cam_world_position, camera &bb_cam)
{
	Point2f imageXY;

	double x = cam_world_position.x,
		   y = cam_world_position.y,
		   z = cam_world_position.z;

	imageXY.x = (float) ((x/z)*bb_cam.intrinsics.fx + bb_cam.intrinsics.cx);
	imageXY.y = (float) ((y/z)*bb_cam.intrinsics.fy + bb_cam.intrinsics.cy);

	return imageXY;
}

Point3f find_intersection(Point3f line_pt1, Point3f line_pt2, float z_projection)
{
	/*
	 * Finds the intersection of a line defined by points
	 * line_pt1 and line_pt2, with a X-Y plane situated at
	 * z = z_projection
	 */

	Point3f pt_difference = (line_pt2 - line_pt1);
	Point3f pt_project;

	pt_project.x = (pt_difference.x/pt_difference.z
					*(z_projection - line_pt1.z)) + line_pt1.x;

	pt_project.y = (pt_difference.y/pt_difference.z
						*(z_projection - line_pt1.z)) + line_pt1.y;

	pt_project.z = z_projection;

	return pt_project;
}


bool draw_line2(Mat & frame, Point3f & pt1, Point3f & pt2)
{
	/*
	 * Sets the near point to be = 0.2cm
	 */

	float near_dist = 0.2;
	Point3f image_plane_point = find_intersection(pt1, pt2, near_dist);

	if(pt1.z < near_dist && pt2.z < near_dist)
		return false;

	Point3f draw_pt1 = (pt1.z > near_dist)? pt1 : image_plane_point,
			draw_pt2 = (pt2.z > near_dist)? pt2 : image_plane_point;

	Point2f image_pt1 = cam_world_position_to_imageXY(draw_pt1, bb_camera),
			image_pt2 = cam_world_position_to_imageXY(draw_pt2, bb_camera);

	Point image_pt1_round = Point(cvRound(image_pt1.x), cvRound(image_pt1.y)),
		  image_pt2_round = Point(cvRound(image_pt2.x), cvRound(image_pt2.y));

	bool line_in = clipLine(frame.size(), image_pt1_round, image_pt2_round);

	if(line_in)
    	line(frame, image_pt1_round, image_pt2_round, Scalar(100,200,50), 2, 8);

	return line_in;
}

void update_view_on_trackbar_change(int, void* )
{
	// Blank out the frame
	frame  = Mat::zeros(480, 640, CV_8UC3);

	bb_camera.position.x = (float) x_pos-100;
	bb_camera.position.y = (float) y_pos-100;
	bb_camera.position.z = (float) z_pos-100;
	bb_camera.tilt = ((float) tilt)*pi/180.0;
	bb_camera.pan = ((float) pan)*pi/180.0;

	for(unsigned int k = 0; k < court.court_lines.size(); k++)
    {

    	Point3f cam_world_pt1 = get_camera_world_coordinates(
											 court.court_lines[k].pt1,
    										 bb_camera.position,
    										 bb_camera.theta, bb_camera.pan,
    										 bb_camera.tilt);

    	Point3f cam_world_pt2 = get_camera_world_coordinates(
											 court.court_lines[k].pt2,
    										 bb_camera.position,
    										 bb_camera.theta, bb_camera.pan,
    										 bb_camera.tilt);
    	draw_line2(frame, cam_world_pt1, cam_world_pt2);
    }

	imshow("Court", frame);
}


int main(int argc, char** argv)
{
	namedWindow("Court", CV_WINDOW_AUTOSIZE);

    createTrackbar("x", "Court", &x_pos, 2600, update_view_on_trackbar_change);
    createTrackbar("y", "Court", &y_pos, 1200, update_view_on_trackbar_change);
    createTrackbar("z", "Court", &z_pos, 3600, update_view_on_trackbar_change);
    createTrackbar("pan", "Court", &pan, 360, update_view_on_trackbar_change);
    createTrackbar("tilt", "Court", &tilt, 360, update_view_on_trackbar_change);


	Point3f x = get_camera_world_coordinates(
											 Point3f(0,0,50),
    										 bb_camera.position,
    										 bb_camera.theta, bb_camera.pan,
    										 bb_camera.tilt);

   	Point2f x_xy = cam_world_position_to_imageXY(
											 x,
											 bb_camera);
   	circle(frame, x_xy, 4, Scalar(0,0,255), CV_FILLED);
   	update_view_on_trackbar_change(0,0);
   	waitKey(0);

	return 0;
}
