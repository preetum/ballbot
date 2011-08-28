#ifndef _differentiation_h
#define _differentiation_h

#include <cv.h>
#include "backproject.h"

double differential(cv::Point3f real_world_position, cv::Point3f cam_position,
		    double sin_theta, double cos_theta, double sin_phi,
		    double cos_phi, int num, int denom);

double differentiate_image_coor(camera particle_cam,
				cv::Point3f cam_world_position,
				cv::Point3f real_world_position,
				int x_or_y, int denom);

double differentiate_costFunction_by_cameraParameter(
    line_segment_2d lineSegment_imagePlane,
    line_segment_3d lineSegment_realWorld,
    line_segment_3d lineSegment_camWorld,
    line_segment_2d matching_line, int parameter,
    camera particle_cam, variances vars);
  
#endif
