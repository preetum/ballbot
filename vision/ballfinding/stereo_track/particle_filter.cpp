// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
/*
 * particle_filter.cpp
 *
 * AUTHOR: John Wang
 * VERSION: 0.1 (11 Aug 2011)
 *
 * DESCRIPTION:
 * Particle filter for ball position estimation.
 */

#include <cv.h>
#include <highgui.h>

#include <stdio.h>
#include <sys/timeb.h>
#include <stdlib.h>
#include <time.h>
#include <math.h>

#include "backproject.h"
#include "particle_filter.h"

using namespace cv;
using namespace std;

// Globals
cv::RNG myRng(time(NULL));

double weightSum(vector<BallParticle> *particles) {
    double weightSum = 0.0;
    for (unsigned int i = 0; i < particles->size(); i += 1)
        weightSum += particles->at(i).weight;
    return weightSum;
}

void weighUniformly(vector<BallParticle> *particles, double weight) {
    for (unsigned int i = 0; i < particles->size(); i += 1)
        particles->at(i).weight = weight;
}

void drawUniformly(vector<BallParticle> *particles, unsigned int n,
                   BoundVec pos_bounds, BoundVec vel_bounds) {
    particles->reserve(particles->size() + n);
    while (n > 0) {
        double x = myRng.uniform(pos_bounds.x.min, pos_bounds.x.max),
            y = myRng.uniform(pos_bounds.y.min, pos_bounds.y.max),
            z = myRng.uniform(pos_bounds.z.min, pos_bounds.z.max),
            vx = myRng.uniform(vel_bounds.x.min, vel_bounds.x.max),
            vy = myRng.uniform(vel_bounds.y.min, vel_bounds.y.max),
            vz = myRng.uniform(vel_bounds.z.min, vel_bounds.z.max);

        particles->push_back(BallParticle(x, y, z, vx, vy, vz));
        n -= 1;
    }
}

/* Initializes an empty particle filter
   Initializes the time-step to T_STEP for transition updates. */
ParticleFilter::ParticleFilter(double t_step) : time_step(t_step),
                                                graph("Particles") {
    initialize(new vector<BallParticle>());
}

/* Initializes a particle filter with the given initial particles.
 * Note that this does _not_ make a deep copy of the vector.
 */
ParticleFilter::ParticleFilter(vector<BallParticle> *initialParticles,
                               camera right,
                               camera left) : camera_right(right),
                                              camera_left(left),
                                              graph("Paticles") {
    initialize(initialParticles);
}

void ParticleFilter::initialize(vector<BallParticle> *initialParticles) {
    // numParticles is the *initial* particle count
    numParticles = initialParticles->size();
    particles = initialParticles;
    normalize();
}

/* Clears the particle filter and (re)initializes it with N particles
 * using a uniform distribution over the given bounds. */
void ParticleFilter::initializeUniformly(unsigned int n,
                                         BoundVec pos_bounds,
                                         BoundVec vel_bounds) {
    particles->clear();
    particles->reserve(n);
    numParticles = n;

    while (n > 0) {
        double x = myRng.uniform(pos_bounds.x.min, pos_bounds.x.max),
            y = myRng.uniform(pos_bounds.y.min, pos_bounds.y.max),
            z = myRng.uniform(pos_bounds.z.min, pos_bounds.z.max),
            vx = myRng.uniform(vel_bounds.x.min, vel_bounds.x.max),
            vy = myRng.uniform(vel_bounds.y.min, vel_bounds.y.max),
            vz = myRng.uniform(vel_bounds.z.min, vel_bounds.z.max);

        particles->push_back(BallParticle(x, y, z, vx, vy, vz, 1));
        n -= 1;
    }
    normalize();
}

void ParticleFilter::observe(cv::Point2d right, cv::Point2d left) {
    for (unsigned int i = 0; i < particles->size(); i += 1) {
        BallParticle &p = particles->at(i);

        Point3d cam_world_right = get_camera_world_coordinates(p.ball.pos,
                                                         camera_right.position,
                                                         camera_right.theta,
                                                         camera_right.pan,
                                                         camera_right.tilt);
        Point2d right_camera_xy = cam_world_position_to_imageXY(
                                          cam_world_right,
                                          camera_right);
        
        Point3d cam_world_left = get_camera_world_coordinates(p.ball.pos,
                                                         camera_left.position,
                                                         camera_left.theta,
                                                         camera_left.pan,
                                                         camera_left.tilt);
        Point2d left_camera_xy = cam_world_position_to_imageXY(
                                          cam_world_left,
                                          camera_left);

        double power = -(sqrt(pow(right_camera_xy.x - right.x, 2)
                              + pow(right_camera_xy.y - right.y, 2)) + 
                         sqrt(pow(left_camera_xy.x - left.x, 2)
                              + pow(left_camera_xy.y - left.y, 2)));
        //printf("%f\n", power);
        p.weight = exp(-power/1000000);
    }
}

/* Move each particle by the specified amount, with Gaussian noise */
void ParticleFilter::transition(double dist, double dtheta, double sigma_dist,
                                double sigma_theta) {
    for (unsigned int i = 0; i < particles->size(); i += 1) {
        BallParticle &p = particles->at(i);
        getNextPos(time_step, &p.ball.pos, &p.ball.vel,
                   &p.ball.pos, &p.ball.vel);
        
        // Add noise using the motion model
        double pos_noise = myRng.gaussian(0.005*norm(p.ball.vel));
        double vel_noise = myRng.gaussian(0.05*norm(p.ball.vel));
        p.ball.pos += Point3d(pos_noise, pos_noise, pos_noise);
        p.ball.vel += Point3d(vel_noise, vel_noise, vel_noise);
    }
}

/* Low variance sampler from Probabilistic Robotics, Thrun et al, p 110 */
void ParticleFilter::resample() {
    normalize();


    printf("Numparticles = %d\n", numParticles);
    vector<BallParticle> *X = new vector<BallParticle>();
    X->reserve(numParticles);
    printf("here2\n");

    //r is a random number between 0 and (1/number_of_particles)
    double r = myRng.uniform(0.0, 1.0/numParticles);
    //initialize sum of weights to the weight of the first particle
    double c = particles->front().weight;
    printf("here3\n");

    unsigned int i = 0;
    for (unsigned int m = 0; m < numParticles; m += 1) {
        double U = r + m*(1.0/numParticles);
        bool isMultipleCopy = true;
        while (U > c) {
            i += 1;
            c += particles->at(i).weight;
            printf("", U,c);
            isMultipleCopy = false;
        }
        BallParticle newParticle = particles->at(i);

        // Add bounded uniform noise to copies if multiple copies
        //  of a particle are generated
        double pos_noise = myRng.uniform(-10,10);
        double vel_noise = myRng.uniform(-50,50);
        newParticle.ball.pos += Point3d(pos_noise, pos_noise, pos_noise);
        newParticle.ball.vel += Point3d(vel_noise, vel_noise, vel_noise); 

        X->push_back(newParticle);
    }
    delete particles; // deallocate former array
    particles = X;

    // Weigh the resampled particles uniformly
    printf("calling weigh uniformly with W =%f\n", 1.0/numParticles);
    weighUniformly(X, 1.0/numParticles);
}

void ParticleFilter::normalize() {
    double sum = weightSum(particles);

    if (sum != 0)
        for (unsigned int i = 0; i < particles->size(); i += 1)
            particles->at(i).weight /= sum;
}

const vector<BallParticle> & ParticleFilter::getBeliefs() const {
    return *particles;
}

/* Print beliefs, up to LIMIT */
void ParticleFilter::print(int limit) const {
    unsigned int N = limit;
    if (N > particles->size()) N = particles->size();

    for (unsigned int i = 0; i < N; i += 1) {
        BallParticle &p = particles->at(i);
        printf("pos=(%.2f, %.2f, %.2f), vel=(%.2f, %.2f, %.2f)\n",
               p.ball.pos.x, p.ball.pos.y, p.ball.pos.z,
               p.ball.vel.x, p.ball.vel.y, p.ball.vel.z);
    }
}

/* Stores the X Y Z coordinates of the particles in the given
   vectors.*/
void ParticleFilter::getCoordinates(vector<double> *x, vector<double> *y,
                                    vector<double> *z) {
    x->clear();
    y->clear();
    z->clear();

    x->reserve(numParticles);
    y->reserve(numParticles);
    z->reserve(numParticles);

    for (unsigned int i = 0; i < particles->size(); i += 1) {
        Point3d position = particles->at(i).ball.pos;
        x->push_back(position.x);
        y->push_back(position.y);
        z->push_back(position.z);
    }
}

void ParticleFilter::plot() {
    try {
        vector<double> x, y, z;
        getCoordinates(&x, &y, &z);

        graph.set_multiplot();
        graph.remove_tmpfiles();
        //graph.reset_all();
        graph.set_grid();
        graph.set_xrange(-10,100);
        graph.set_yrange(-10,100);
        graph.set_zrange(-10,500);
        graph.plot_xyz_custom(x,y,z,"blue", 1, 1, "Ball Particles");
    } catch (GnuplotException except)  {
        cout<<except.what()<<endl;
    }
}
