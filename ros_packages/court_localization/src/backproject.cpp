// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
/*
 * hill-climb-find-pose.cpp
 *
 *  Created on: Aug 16, 2011
 *      Author: ankush
 */

#include <cv.h>
#include <highgui.h>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <limits>

#include "structs.h"
#include "differentiation.h"

using namespace cv;
using namespace std;

vector <Point3f> world_coordinates;
vector <Point2f> image_plane_coordinates;

RNG rng; // Random Number Generator
camera bb_camera;

// Camera Matrix
Mat cam_matrix = Mat::zeros(3, 3, CV_32FC1);

//Trackbar variables
Mat frame  = Mat::zeros(480, 640, CV_8UC3);
Mat particle_frame = Mat::zeros(480, 640, CV_8UC3);

int x_pos = bb_camera.position.x,
	y_pos = bb_camera.position.y,
	z_pos = bb_camera.position.z,
	pan = bb_camera.pan,
	tilt = bb_camera.tilt;

struct _court
{
	vector <line_segment_3d> court_lines;

	_court()
	{
		line_segment_3d line_seg;

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

double find_depth(Point2d pt1, Point2d pt2, Point2d q, double z1, double z2)
{
	/*
	 * Returns the depth corresponding to point q, assuming q lies on the
	 * line defined by pt1 and pt2.
	 *
	 * 					pt1-------------q------------pt2
	 *
	 * pt1 and pt2 are the coordinates of the projections of the end-points
	 * in the image-plane and z1 and z2 are their depths in 3d space.
	 *
	 * This function returns the depth of point q (in 3D space).
	 */

	double dist = norm(pt2-pt1),
		   delta_z = z2-z1;

	double q_depth = z1 + norm(pt1 - q)*(delta_z/dist);

	return q_depth;
}

vector <line_segment_all_frames> get_view_lines(camera particle_camera,
	                                         Size frame_size,
						 Mat &view_frame,
						 float near_dist = 0.2)
{
	/* Retruns a vector of type line_segment_all_frames
	 * which contains the line segments that are VISIBLE
	 * with the given particle_camera pose
	 * 
	 * Arguments:
	 * -------------------------------------------------------------
	 * particle_camera : The camera in respect to which the view lines
	 *                   are to be calculated
	 * frame_size      : the size of the frame on which to project the lines
	 * view_frame      : the frame on which the view lines are drawn
	 * near_dist       : the closest distance out that the camera can see
	 *
	 */
	vector <line_segment_all_frames> view_lines;

	for(unsigned int k = 0; k < court.court_lines.size(); k++)
	{
		line_segment_all_frames line_seg;

		Point3d cam_world_pt1 = get_camera_world_coordinates(
											court.court_lines[k].pt1,
	    									particle_camera.position,
	    									particle_camera.theta,
	    									particle_camera.pan,
	    									particle_camera.tilt);

	    Point3d cam_world_pt2 = get_camera_world_coordinates(
											court.court_lines[k].pt2,
	    									particle_camera.position,
	    									particle_camera.theta,
	    									particle_camera.pan,
	    									particle_camera.tilt);

	    //skip the line if it is entirely behind the camera
	    if(cam_world_pt1.z < near_dist && cam_world_pt2.z < near_dist)
			continue;

	    // Find the points in the camera world coordinates which
	    // are actually to be drawn
		Point3d image_plane_point = find_intersection(cam_world_pt1,
							      cam_world_pt2, near_dist);
		Point3d draw_pt1, draw_pt2;

		// Flags for indicating which point has been
		// replaced by the intersection_point, if any
		bool pt1_out = false, pt2_out =  false;

		if(cam_world_pt1.z >= near_dist)
		{
			draw_pt1 = cam_world_pt1;
			pt1_out = false;
		}
		else if (cam_world_pt1.z < near_dist)
		{
			draw_pt1 = image_plane_point;
			pt1_out = true;
		}

		if (cam_world_pt2.z >= near_dist)
		{
			draw_pt2 = cam_world_pt2;
			pt2_out = false;
		}
		else if (cam_world_pt2.z < near_dist)
		{
			draw_pt2 = image_plane_point;
			pt2_out = true;
		}

		Point2d image_pt1 = cam_world_position_to_imageXY(draw_pt1, bb_camera),
				image_pt2 = cam_world_position_to_imageXY(draw_pt2, bb_camera);

		// Round the float to ints to obtain the pixel values
		Point image_pt1_round = Point(cvRound(image_pt1.x), cvRound(image_pt1.y)),
			  image_pt2_round = Point(cvRound(image_pt2.x), cvRound(image_pt2.y));
		// Create a copy of the pixel coordinates
		Point image_pt1_round_copy = image_pt1_round,
			  image_pt2_round_copy = image_pt2_round;

		// Get the points which lie in the frame.
		// The points image_pt1_round and image_pt2_round are
		// changed by the function clipLine to points that fit
		// in the frame
		bool line_in = clipLine(frame_size, image_pt1_round, image_pt2_round);
		bool image_pt1_out = false, image_pt2_out = false;

		if(line_in)
			{
			if(image_pt1_round_copy != image_pt1_round)
				image_pt1_out = true;
			if(image_pt2_round_copy != image_pt2_round)
				image_pt2_out = true;
			}
		else
			continue; // skip the line if both the projection
			          // points lie outside the frame

		if(!pt1_out && !image_pt1_out)
		{
			line_seg.realWorld.pt1 = court.court_lines[k].pt1;
			line_seg.camWorld.pt1 = cam_world_pt1;
			line_seg.imgPlane.pt1 = image_pt1;
		}
		else if (pt1_out && !image_pt1_out)
		{
			Point3d realWorld_pt = get_world_coordinates(
				                                     image_plane_point,
								     particle_camera.position,
								     particle_camera.theta,
								     particle_camera.pan,
								     particle_camera.tilt);

			line_seg.realWorld.pt1 = realWorld_pt;
			line_seg.camWorld.pt1 = image_plane_point;
			line_seg.imgPlane.pt1 = image_pt1;
		}
		else
		{
			double image_pt1_depth = find_depth(image_pt1, image_pt2,
												image_pt1_round,draw_pt1.z,
												draw_pt1.z);
			Point3d camWorld_pt = image_plane_to_camera_world_position(
									  image_pt1_round, image_pt1_depth,
									  particle_camera);
			Point3d realWorld_pt = get_world_coordinates(
								     camWorld_pt,
								     particle_camera.position,
								     particle_camera.theta,
								     particle_camera.pan,
								     particle_camera.tilt);

			line_seg.realWorld.pt1 = realWorld_pt;
			line_seg.camWorld.pt1  = camWorld_pt;
			line_seg.imgPlane.pt1  = image_pt1_round;
		}

		if(!pt2_out && !image_pt2_out)
		{
			line_seg.realWorld.pt2 = court.court_lines[k].pt2;
			line_seg.camWorld.pt2 = cam_world_pt2;
			line_seg.imgPlane.pt2 = image_pt2;
		}
		else if(pt2_out && !image_pt2_out)
		{
			Point3d realWorld_pt = get_world_coordinates(
								     image_plane_point,
								     particle_camera.position,
								     particle_camera.theta,
								     particle_camera.pan,
								     particle_camera.tilt);

			line_seg.realWorld.pt2 = realWorld_pt;
			line_seg.camWorld.pt2 = image_plane_point;
			line_seg.imgPlane.pt2 = image_pt2;
		}
		else
		{
			double image_pt2_depth = find_depth(image_pt1, image_pt2,
												image_pt2_round,draw_pt1.z,
												draw_pt1.z);
			Point3d camWorld_pt = image_plane_to_camera_world_position(
									  image_pt2_round, image_pt2_depth,
									  particle_camera);
			Point3d realWorld_pt = get_world_coordinates(
								     camWorld_pt,
								     particle_camera.position,
								     particle_camera.theta,
								     particle_camera.pan,
								     particle_camera.tilt);
			line_seg.realWorld.pt2 = realWorld_pt;
			line_seg.camWorld.pt2  = camWorld_pt;
			line_seg.imgPlane.pt2  = image_pt2_round;
		}

		view_lines.push_back(line_seg);

		// draw the line, if not empty
		if(line_in)
			line(view_frame, image_pt1_round, image_pt2_round, Scalar(100,200,50), 1, 8);
	}// for loop

	cout<<"# lines drawn: "<<view_lines.size()<<endl;
	return view_lines;
}

void hill_climb(vector <line_segment_2d> actual_view,
		camera particle_camera, Mat &particle_frame, Mat &actual_frame,
		unsigned int iterations, double epsilon)
{
	/* Implements the gradient ascent algorithm
	 * 
	 * Arguments:
	 * -------------------------------------------------------------------
	 * actual_view      : vector of the 2D line segments in the actual view
	 * particle_view    : vector of line-segments (in 3 frames) backprojected
	 * 					  from the 3d model of the court, given the particle
	 * 					  location
	 *  particle_camera : position and orientation of the particles camera
	 *  				  (this camera is the particle itself)
	 *  iterations		: maximum number of iterations for gradient ascent
	 *  epsilom			: if the change is subsequent iterations is smaller
	 *  				  than epsilon, the optimization terminates
	 */

	unsigned int num_iters = 0;
	vector <line_segment_all_frames> particle_view;
	variances vars;

	double alpha = 0.0001;
	double camera_pose_error = 1000;

	while(num_iters < 500)//iterations)
	{
		particle_frame  = Mat::zeros(480, 640, CV_8UC3);
		particle_view = get_view_lines(particle_camera, particle_frame.size(),
															  particle_frame);
		imshow("Particle", particle_frame);
		waitKey(10);
		vector <double> slopes(5,0);

		if(actual_view.size() == particle_view.size())
		{
			// Draw the matched lines
			for(unsigned int j = 0; j < actual_view.size(); j++)
			{

				Scalar color = Scalar(255*rng.uniform((double)0, (double)1),
									  255*rng.uniform((double)0, (double)1),
									  255*rng.uniform((double)0, (double)1));
				line(actual_frame, actual_view[j].pt1,actual_view[j].pt2, color, 2, 8);
				line(particle_frame, particle_view[j].imgPlane.pt1, particle_view[j].imgPlane.pt2, color, 2, 8);
			}


			for(int i = 1; i<=5; i++)
			{
				for(unsigned int k= 0; k < particle_view.size(); k++)
				{
					slopes[i-1] += differentiate_costFunction_by_cameraParameter(
											particle_view[k].imgPlane,
											particle_view[k].realWorld,
											particle_view[k].camWorld,
											actual_view[k], i,
											particle_camera, vars);
				}
			}

		for(unsigned int k = 0; k < slopes.size(); k++)
				cout<<"Slopes2: "<<k<<" : "<<slopes[k]<<" |";
		cout<<endl;

		particle_camera.position.x += alpha*slopes[0];
		particle_camera.position.y += alpha*slopes[1];
		particle_camera.pan += alpha*slopes[3];
		particle_camera.theta += alpha*slopes[3];
		particle_camera.tilt += alpha*slopes[4];

		}
		else
		      cout<<"Mistmatch: Different number of lines in actual view"
			  <<" and particle view"<<endl;
	num_iters += 1;

	}

	cout<<particle_camera.position.x<<" "<<particle_camera.position.y<<" "
		<<(particle_camera.pan*180.0/CV_PI)<<" "<<(particle_camera.theta*180.0/CV_PI)<<" "
		<<(particle_camera.tilt*180.0/CV_PI)<<endl;;
}

bool draw_line2(Mat & frame, Point3f & pt1, Point3f & pt2)
{

	/* Draws a line on the given frame,
	 * connecting pt1 and pt2 which are (3D) points
	 * in camera's world.
	 *
	 * --> sets the near point to be = 0.2cm away
	 * 	   so, anything with z < 0.2cm is not visible/ drawn
	 */

	float near_dist = 0.2;
	if(pt1.z < near_dist && pt2.z < near_dist)
		return false;

	Point3f image_plane_point = find_intersection(pt1, pt2, near_dist);

	Point3f draw_pt1 = (pt1.z >= near_dist)? pt1 : image_plane_point,
			draw_pt2 = (pt2.z >= near_dist)? pt2 : image_plane_point;

	Point2f image_pt1 = cam_world_position_to_imageXY(draw_pt1, bb_camera),
			image_pt2 = cam_world_position_to_imageXY(draw_pt2, bb_camera);

	// Round the float to ints to obtain the pixel values
	Point image_pt1_round = Point(cvRound(image_pt1.x), cvRound(image_pt1.y)),
		  image_pt2_round = Point(cvRound(image_pt2.x), cvRound(image_pt2.y));

	bool line_in = clipLine(frame.size(), image_pt1_round, image_pt2_round);

	if(line_in)
		line(frame, image_pt1_round, image_pt2_round, Scalar(100,200,50), 1, 8);

	return true;
}

void update_view_on_trackbar_change(int, void* )
{
	/*
	 * This is the callback function for changes in any of the
	 * trackbars.
	 * This takes the new camera position set using the trackbars
	 * and updates the view accordingly.
	 */

	// Blank out the frame
	frame  = Mat::zeros(480, 640, CV_8UC3);
	particle_frame  = Mat::zeros(480, 640, CV_8UC3);

	// Update camera pose
	bb_camera.position.x = x_pos-100;
	bb_camera.position.y = y_pos-100;
	bb_camera.position.z = z_pos-100;
	bb_camera.tilt = ((double) tilt-180)*CV_PI/180.0;
	bb_camera.pan = ((double) pan)*CV_PI/180.0;

	// Create a fake particle camera
	camera particle_cam;
	particle_cam.position.x = bb_camera.position.x-10;
	particle_cam.position.y = bb_camera.position.y-5;
	particle_cam.position.z = bb_camera.position.z;
	particle_cam.theta = bb_camera.theta;
	particle_cam.pan = bb_camera.pan;
	particle_cam.tilt = bb_camera.tilt-(10*CV_PI/180.0);

	vector <line_segment_all_frames> actual_view = get_view_lines(bb_camera, frame.size(), frame);
	cout<<"From update: actual view size: "<<actual_view.size()<<endl;

	vector <line_segment_all_frames> particle_view = get_view_lines(particle_cam, particle_frame.size(), particle_frame);


	vector <line_segment_2d> actual_segments;
	for(unsigned int k = 0; k < actual_view.size(); k++)
	{
		actual_segments.push_back(actual_view[k].imgPlane);
	}

	hill_climb(actual_segments,
		   particle_cam, particle_frame, frame,
		   1000, 0.001);

	cout<<"--------------------------------"<<endl;
	imshow("Particle", particle_frame);
	imshow("Court", frame);
}

