// -*- mode: c++; indent-tabs-mode: nil; c-basic-offset: 4 -*-
#ifndef _particle_filter_h
#define _particle_filter_h

#include <vector>
#include <cv.h>
#include "backproject.h" /** For camera transformations. */
#include "gnuplot_i.hpp" /** For plotting the particles in 3D. */
#include "trajectory_simulator.h"

class Bounds {
public:
    double min;
    double max;

    Bounds(double min, double max) :
        min(min), max(max) { }
};

class BoundVec {
    public:
    Bounds x;
    Bounds y;
    Bounds z;
    
    BoundVec(Bounds x_init, Bounds y_init, Bounds z_init) :
        x(x_init), y(y_init), z(z_init) {}
};

class Ball {
public:
    /** pos, vel represent the ball's location and velocity
        in the world frame*/

    cv::Point3d pos, vel;

    Ball() : pos(), vel() { }

    Ball(cv::Point3d pos_init, cv::Point3d vel_init) :
        pos(pos_init), vel(vel_init) { }

    Ball(double x=0, double y=0, double z=0,
         double vx=0, double vy=0, double vz=0) :
        pos(x,y,z), vel(vx, vy, vz) { }    
};

class BallParticle {
    /** A ball particle with a weight attached.*/
public:
    double weight;
    Ball ball;

    BallParticle(double x=0, double y=0, double z=0,
                 double vx=0, double vy=0, double vz=0, double weight=1) :
        ball(x, y, z, vx, vy, vz), weight(weight) { }

    BallParticle(cv::Point3d pos_init, cv::Point3d vel_init,
                 double weight=1) :
        ball(pos_init, vel_init), weight(weight) { }
};

class ParticleFilter {
    private:
    /** Stores the coordinates of the particles in the given input vectors.*/
    void getCoordinates(std::vector<double> *x, std::vector<double> *y,
                        std::vector<double> *z);
    camera camera_right, camera_left;
    double time_step;

    public:
    Gnuplot graph;
    unsigned int numParticles;
    std::vector<BallParticle> *particles;

    ParticleFilter(double t_step);
    ParticleFilter(std::vector<BallParticle> *initialParticles,
                   camera right, camera left);

    /* Initialize uniformly */
    void initialize(std::vector<BallParticle> *particles);
    void initializeUniformly(unsigned int n,
                             BoundVec pos_bounds,
                             BoundVec vel_bounds);

    /* Observe an emission and reweight the particles accordingly
       RIGHT is the current position of the ball as seen by the right camera.
       LEFT  is the current position of the ball as seen by the left camera.
       The above positions are in pixel units.*/
    void observe(cv::Point2d right, cv::Point2d left);

    /* Transition the particles using movement and Gaussian noise */
    void transition(double dist=0, double dtheta=0,
                    double sigma_xy=5, double sigma_theta=0.01);
    void resample();
    void normalize();

    /* Output functions */
    const std::vector<BallParticle> & getBeliefs() const;
    void print(int limit=-1) const;
    /** Display a scatter plot of the particles.*/
    void plot();
};

#endif
