// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-

#include "particle_filter.h"
#include "trajectory_simulator.h"
#include <cv.h>
#include <highgui.h>

#define TIME_STEP 0.01
#define NUM_PARTICLES 200

using namespace cv;
using namespace std;


void printP(int limit, vector<BallParticle> particles) {
    unsigned int N = limit;
    if (N > particles.size()) N = particles.size();

    for (unsigned int i = 0; i < N; i += 1) {
        BallParticle &p = particles.at(i);
        printf("pos=(%.2f, %.2f, %.2f)|vel=(%.2f, %.2f, %.2f)|weight= %.5f\n",
               p.ball.pos.x, p.ball.pos.y, p.ball.pos.z,
               p.ball.vel.x, p.ball.vel.y, p.ball.vel.z, p.weight);
    }
}


int main(int argc, char** argv) {
    
    Point3d init_pos(5,5,200);
    Point3d init_vel(10,10,100);
    cv::Point3d cam_pos(100,50,50);
    camera cam1(cam_pos, CV_PI/4, 0, 0);
    camera cam2(cam_pos, CV_PI/4-0.01, 0, 0);
    vector<Point3d> traj3d;
    vector<Point2d> traj_cam1, traj_cam2;
    printf("init!\n");

    generate3dTrajectory(init_pos, init_vel, TIME_STEP,
                         200, &cam1, &cam2, &traj3d,
                         &traj_cam1, &traj_cam2);
    printf("trajectory!\n");

    BoundVec posRange(Bounds(0,6),
                      Bounds(0,6),
                      Bounds(150,200));
    BoundVec velRange(Bounds(90,100),
                      Bounds(40,60),
                      Bounds(80,100));
    ParticleFilter pf(TIME_STEP);
    pf.initializeUniformly(NUM_PARTICLES, posRange, velRange);
    printf("done!\n");
    std::vector<double> x,y,z;

    for (int i = 0; i < 199; i += 1) {
        pf.transition(0,0,0,0);      
        pf.observe(traj_cam1.at(i), traj_cam2.at(i));
        pf.resample();
        pf.plot();

        x.push_back(traj3d.at(i).x);
        y.push_back(traj3d.at(i).y);
        z.push_back(traj3d.at(i).z);
        pf.graph.plot_xyz_custom(x,y,z,"red", 1, 1, "Actual Ball");

        waitKey(200);
    }
    return 0;
}
