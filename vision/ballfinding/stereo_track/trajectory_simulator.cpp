// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-

#include "trajectory_simulator.h"

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
                           std::vector<cv::Point3d> *traj_3d,
                           std::vector<cv::Point2d> *traj_cam1,
                           std::vector<cv::Point2d> *traj_cam2) {
    cv::Point3d pos_i = init_pos, vel_i = init_vel;
    cv::Point3d pos_f, vel_f;

    traj_3d->clear();
    traj_cam1->clear();
    traj_cam2->clear();
    
    traj_3d->reserve(N);
    traj_cam1->reserve(N);   
    traj_cam2->reserve(N);

    getNextPos(time_step, &pos_i, &vel_i, &pos_f, &vel_f);

    for (int i = 0; i < N-1; i += 1) {
        getNextPos(0.01, &pos_f, &vel_f, &pos_f, &vel_f);
        traj_3d->push_back(pos_f);
        //--------- camera1 position --------------
        cv::Point3d cam_world_pos
            = get_camera_world_coordinates(pos_f,
                                           cam1->position,
                                           cam1->theta, cam1->pan,
                                           cam1->tilt);
        cv::Point2d image_pos = cam_world_position_to_imageXY(cam_world_pos,
                                                              *cam1);
        traj_cam1->push_back(image_pos);

        //--------- camera2 position --------------
        cam_world_pos = get_camera_world_coordinates(pos_f,
                                                     cam2->position,
                                                     cam2->theta, cam1->pan,
                                                     cam2->tilt);
        image_pos = cam_world_position_to_imageXY(cam_world_pos,
                                                          *cam2);
        traj_cam2->push_back(image_pos);
    }
}

