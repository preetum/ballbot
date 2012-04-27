// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
#ifndef _trajectory_simulator_h
#define _trajectory_simulator_h

/** Simulated trajectory generator for a ball moving in 3D space.
    @author: Ankush Gupta
    @date  : March 25, 2012.*/

#include <stdio.h>
#include <math.h>
#include <cv.h>
#include "backproject.h"

/** Generates the final position and velocity, given the initial position
    and velocity.
 *  Assumes, that z points towards sky and that floor is at z=0
 *  Distances are in CENTIMETERS and time is in SECONDS.*/
void getNextPos(double t, cv::Point3d *init_x, cv::Point3d *init_v,
                cv::Point3d *final_x, cv::Point3d *final_v);

/** 1. Stores the 3D trjectory of N data-points separated by TIME_STEP
       in the input vector TRAJ_3D.
    2. It stores the image of that trajectory as seen by CAMERA1 in TRAJ_CAM1
       and by CAMERA2 in TRAJ_CAM2.
    3. INIT_POS and INIT_POS are the initial position and velocities of the
       ball/ point. */
void generate3dTrajectory (const cv::Point3d init_pos,
                           const cv::Point3d init_vel,
                           const double time_step, const int N,
                           camera *cam1, camera *cam2,
                           std::vector<cv::Point3d> *traj_3D,
                           std::vector<cv::Point2d> *traj_cam1,
                           std::vector<cv::Point2d> *traj_cam2);

#endif
