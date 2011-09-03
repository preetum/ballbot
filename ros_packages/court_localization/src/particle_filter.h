// -*- mode: c++; indent-tabs-mode: nil; c-basic-offset: 4 -*-
#ifndef _particle_filter_h
#define _particle_filter_h

#include <vector>
#include <cv.h>

class Bounds {
public:
    double min;
    double max;

    Bounds(double min, double max) :
        min(min), max(max) { }
};

class Pose {
public:
    // x, y represent the robot's location in the world frame
    // theta represents the robot's heading (measured
    //  counter-clockwise from the world frame's x-axis)
    double x, y, theta;
    Pose(double x=0, double y=0, double theta=0) :
        x(x), y(y), theta(theta) { }
};

class PoseParticle {
public:
    Pose pose;
    double weight;

    PoseParticle(double x=0, double y=0, double theta=0, double weight=1) :
        pose(x, y, theta), weight(weight) { }
};

class ParticleFilter {
public:
    unsigned int numParticles;
    std::vector<PoseParticle> *particles;

    ParticleFilter();
    ParticleFilter(std::vector<PoseParticle> *initialParticles);

    /* Initialize uniformly */
    void initializeUniformly(unsigned int n, Bounds x, Bounds y, Bounds theta);

    /* Observe an emission and reweight the particles accordingly */
    void observe(cv::Mat &observation);
    /* Transition the particles using movement and Gaussian noise */
    void transition(Pose movement=Pose(),
                    double sigma_xy=5, double sigma_theta=0.01);
    void resample();
    void normalize();
    const std::vector<PoseParticle> & getBeliefs() const;
    void print(int limit=-1) const;
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);
};

#endif
