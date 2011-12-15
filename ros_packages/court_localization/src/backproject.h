// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
#ifndef _backproject_h
#define _backproject_h

#include <cv.h>

struct variances
{
	double x, y, len, angle;
	variances()
	{
		x = 20*20; // pixels-squared
		y = 20*20; // pixels-squared
		len = 50*50; // pixels-squared
		angle = (CV_PI/7.0*CV_PI/7.0); // radians-squared
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

	cv::Point3d position;
	double  theta, pan, tilt;
	camera_intrinsic_values intrinsics;

	camera() // Initialized for capture1...backcorner.avi
	{

		position.x = 0;
		position.y = 0;
		position.z = 33;
		theta = 0;
		pan = 0;
		tilt = 0;
	}

};

/* line_segment_3d is a pair of cv::Point3d's. The d stands for
 * double-precision.
 */
struct line_segment_3d
{
	cv::Point3d pt1, pt2;

    // Initialize a LineSegment with values pt1=(0,0,0) pt2=(0,0,0)
    // (initialized by default Point3d constructor)
    line_segment_3d() { }

    line_segment_3d(cv::Point3d _pt1, cv::Point3d _pt2) :
        pt1(_pt1), pt2(_pt2) { };

    line_segment_3d(double x1, double y1, double z1,
                    double x2, double y2, double z2) :
        pt1(x1, y1, z1), pt2(x2, y2, z2) { };

	bool operator==(const line_segment_3d &other) {
        return (other.pt1 == pt1 && other.pt2 == pt2);
    }

	void operator=(const line_segment_3d &other) {
        pt1 = other.pt1;
        pt2 = other.pt2;
    }

};

struct line_segment_2d
{
	cv::Point2d pt1, pt2;

	line_segment_2d()
	{
		pt1.x = 0;  pt1.y = 0;
		pt2.x = 0;  pt2.y = 0;
	}
	
	bool operator==(const line_segment_2d &other)
		{
			return (other.pt1 == pt1 && other.pt2 == pt2);
		}

	void operator=(const line_segment_2d &other)
		{
			pt1 = other.pt1;
			pt2 = other.pt2;
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
     * index is the index of the court line in the model
	 */

	line_segment_2d imgPlane;
	line_segment_3d camWorld;
	line_segment_3d realWorld;
    int index;
};


cv::Point3d get_camera_world_coordinates(cv::Point3d real_world_position,
                                         cv::Point3d camera_position,
                                         double heading, double pan,
                                         double tilt);
cv::Point2d cam_world_position_to_imageXY(cv::Point3d cam_world_position,
                                          camera &bb_cam);

std::vector <line_segment_all_frames> get_view_lines(camera particle_camera,
                                              cv::Size frame_size,
                                              cv::Mat &view_frame,
                                              float near_dist = 0.2,
                                              bool draw = false);
#endif
