// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-

/** Generates a file, which contains the coordinates
    of a ball moving in space, in the following format:
    (x, y, z)

    @author: Ankush Gupta
    @date  : March 25, 2012.*/


#include <stdio.h>
#include <fstream>
#include <iostream>
#include <math.h>
#include <cv.h>
#include "backproject.h"

/** Appends the VECTOR (3D) to the file FNAME */
void fileOut(cv::Point3d *vector, char *fName){
    std::ofstream f;
    f.open(fName, std::ios::app);
    f <<vector->x <<'\t'<< vector->y<<'\t'<< vector->z <<'\n';
    f.close();
}

/** Appends the VECTOR (2D) to the file FNAME */
void fileOut(cv::Point2d *vector, char *fName){
    std::ofstream f;
    f.open(fName, std::ios::app);
    f <<vector->x <<'\t'<< vector->y<<'\n';
    f.close();
}

/** Print the VECTOR in the format (x,y,z) on the standard output */
void print(cv::Point3d *vector){
    printf("(%3.3f, %3.3f, %3.3f)\n", vector->x, vector->y, vector->z);
}

/** Print the VECTOR in the format (x,y) on the standard output */
void print(cv::Point2d *vector){
    printf("(%3.3f, %3.3f)\n", vector->x, vector->y);
}

/** Generates the final position and velocity, given the initial time and velocity.
 *  Assumes, that z points towards sky and that floor is at z=0
 *  Distances are in CENTIMETERS and time is in SECONDS.*/
void getNextPos(double t, cv::Point3d *init_x, cv::Point3d *init_v,
                cv::Point3d *final_x, cv::Point3d *final_v) {
    double initX_z = init_x->z, initV_z = init_v->z;

    *final_x = *init_x + (*init_v) * t;
    final_x->z -= 490*t*t;

    *final_v = *init_v;
    final_v->z -= 980*t;

    double energy = 980*final_x->z + 0.5*(final_v->x*final_v->x
                                          + final_v->y*final_v->y
                                          + final_v->z*final_v->z);
    if (final_x->z < 0) {
        double energy = 980*final_x->z + 0.5*(final_v->x*final_v->x
                                              + final_v->y*final_v->y
                                              + final_v->z*final_v->z);
        double v0 = sqrt(2*(energy-0.5*(final_v->x*final_v->x
                                        + final_v->y*final_v->y)));
        double T = -(initV_z + v0)/980;
        final_v->z = 0.8*v0 - 980*(t-T);
        final_x->z = 0.8*v0*(t-T) - 490*(t-T)*(t-T);
    }
}

void generate3dTrajectory (const cv::Point3d init_pos,
                           const cv::Point3d init_vel,
                           const double time_step, const int N,
                           camera *cam1, camera *cam2,
                           char* fileName3D, char* fileName2DCam1,
                           char* fileName2DCam2) {
    cv::Point3d pos_i = init_pos, vel_i = init_vel;
    cv::Point3d pos_f, vel_f;

    fileOut(&(cam1->position), fileName3D);
    fileOut(&(cam2->position), fileName3D);

    getNextPos(time_step, &pos_i, &vel_i, &pos_f, &vel_f);
    print(&pos_f);
    for (int i = 0; i < N-1; i += 1) {
        getNextPos(0.01, &pos_f, &vel_f, &pos_f, &vel_f);
        print(&pos_f);
        fileOut(&pos_f, fileName3D);

        //--------- camera1 position --------------
        cv::Point3d cam_world_pos
            = get_camera_world_coordinates(pos_f,
                                           cam1->position,
                                           cam1->theta, cam1->pan,
                                           cam1->tilt);
        cv::Point2d image_pos = cam_world_position_to_imageXY(cam_world_pos,
                                                              *cam1);
        fileOut(&image_pos, fileName2DCam1);

        //--------- camera2 position --------------
        cam_world_pos = get_camera_world_coordinates(pos_f,
                                                     cam2->position,
                                                     cam2->theta, cam1->pan,
                                                     cam2->tilt);
        image_pos = cam_world_position_to_imageXY(cam_world_pos,
                                                          *cam2);
        fileOut(&image_pos, fileName2DCam2);
    }
}

int main(int argc, char* argv[]) {
    cv::Point3d init_pos(5,5,200);
    cv::Point3d init_vel(10,10,100);
    cv::Point3d cam_pos(100,50,50);
    camera cam1(cam_pos, -CV_PI/4, 0, 0);
    camera cam2(cam_pos, -CV_PI/4-0.01, 0, 0);

    generate3dTrajectory(init_pos, init_vel, 0.01,
                         200, &cam1, &cam2, "traj-out.dat",
                          "image1.dat", "image1.dat");

    /**cv::Point3d cam_pos(500,-100,50);
    camera cam1(cam_pos, CV_PI/6, 0, 0);
    camera cam2(cam_pos, CV_PI/6-0.01, 0, 0);
    generate3dTrajectory (cv::Point3d(0,200,300),
                          cv::Point3d(100, 50, 5),
                          0.1, 100, &cam1, &cam2, "traj-out.dat",
                          "image1.dat", "image1.dat");**/
    return 0;
}
