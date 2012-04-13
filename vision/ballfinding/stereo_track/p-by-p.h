/*
 * Basic Particle Filter Class
 *
 *  Created on: March 30, 2012
 *      Author: Ankush Gupta
 */

#ifndef __particle_filter_h
#define __particle_filter_h

#include <cv.h>
#include <time.h>
#include <math.h>
#include <iostream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include "backproject.h"

#define NUM_PARTICLES 1000
#define pi 3.1415926535897

Point3d cross(Point3d, Point3d);
double dist (Point3d, Point3d, Point3d);


/** Holds the position and the velocity
    of the ball particle.*/
struct ball_particle {
    /** Position of the ball [centimeters]. */
    cv::Point3d pos;

    /** Velocity of the ball [centimeters/sec]. */
    cv::Point3d vel;

    ball_particle() {}

    ball_particle(cv::Point3d * _pos, cv::Point3d * _vel) {
	pos = * _pos;
	vel = * _vel;
    }
};

/** Combines a particle and a weight.*/
struct weighted_particle {

    /** The ball particle.*/
    ball_particle ball;

    /** Weight of the particle. */
    double weight;

    weighted_particle(ball_particle * _ball, double _w) {
	ball = * _ball;
	weight = _w;
    }
};


/** A structure for bounds; combines a MIN and
    a MAX value. */
struct bound {
    /** The min value. */
    double min;

    /** The max value. */
    double max;

    bound() {
	min = 0.0d;
	max = 0.0d;
    }

    bound(double _min, double _max) {
	min = _min;
	max = _max;
    }
};


/** Defines the initialization bounds. */
struct bounds {
    /** Bounds for position. */
    cv::Point3_<bound> pos;
    
    /** Bounds for velocity. */
    cv::Point3_<bound> vel;

    bounds(bound *pos_x, bound *pos_y, bound *pos_z,
	   bound *vel_x, bound *vel_y, bound *vel_z) {
	pos = new cv::Point3_<bound>(*pos_x, *pos_y, *pos_z);
	vel = new cv::Point3_<bound>(*vel_x, *vel_y, *vel_z);
    }

    bounds(cv::Point3_<bound> *_pos, cv::Point3_<bound> *_vel) {
	pos = *_pos;
	vel = *_vel;
    }
};

struct observations {
    cam_pos camera;
    vector <Point3d> blobs_pos;
};

/** Outputs a random num N, where,
    bnd.min <= N <= bnd.max. */
double get_bounded_random(bound *bnd) {
    cv::RNG rnd(time(NULL));
    double rand = rnd.uniform(0.0d, 1.0d);
    return bnd->min + rand*(bnd->max - bnd->min);
}

/** Returns a vector of size N, with
    uniformly sampled ball_particles from the given bounds B. */
vector<ball_particle> sample_uniformly(int n, bounds bnd) {
    vector<ball_particle> particles;
    for(int i = 0; i < n; i += 1) {
	ball_particle particle;
	particle.pos = cv::Point3d(get_bounded_random(*bnd.pos.x),
			       get_bounded_random(*bnd.pos.y),
			       get_bounded_random(*bnd.pos.z));

	particle.vel = cv::Point3d(get_bounded_random(*bnd.vel.x),
			       get_bounded_random(*bnd.vel.y),
			       get_bounded_random(*bnd.vel.z));

	particles.push_back(particle);
    }
    return particles;
}


class particles {

    cv::RNG rng(time(NULL));
    
    public:
    vector<weighted_particle> *balls;
    int numParticles;

    particles() {}
    
    /** Initialize the particles to the particles in _BALLS.
	All the particles are assigned equal weights. */
    particles(vector<ball_particle> *_balls) {
	numParticles = _balls->size();
	balls.resize(numParticles);
	for(unsigned int i = 0; i < _balls->size(); i += 1)
	    balls[i].ball = _balls->at(i);
	weigh_uniformly();
    }

    /** Assign the particles equal weight = 1/N,
	where N is the number of particles.*/
    void weigh_uniformly() {
	double w = 1/(double) balls.size();
	for(weighted_particle particle : balls)
	    particle.weight = weight;
    }

    /** Returns the sum of the weights of the particles.*/
    double get_weight_sum()  {
	double sum = 0.0;
	for(weighted_particle particle : balls)
	    sum += particle.weight;
	return sum;
    }

    /** Normalizes the weights of the particles so that the weights
     sum to one. */
    void normalize() {
	double sum = get_weight_sum();
	for(weighted_particle particle : balls)
	    particle.weight = particle.weight/sum;
    }

    /* Low variance sampler from Probabilistic Robotics,
       Thrun et al, p 110 */
    void resample() {
	normalize();

	vector<weighted_particle> *X = new vector<weighted_particle>();
	X->reserve(numParticles);

	//r is a random number between 0 and (1/number_of_particles)
	double r = rng.uniform(0.0, 1.0/numParticles);

	//initialize sum of weights to the weight of the first particle
	double c = balls->front().weight;

	unsigned int i = 0;
	for (unsigned int m = 0; m < numParticles; m += 1) {
	    double U = r + m*(1.0/numParticles);
	    bool isMultipleCopy = true;
	    while (U > c) {
		i += 1;
		c = c + balls->at(i).weight;
		isMultipleCopy = false;
	    }
	    weighted_particle newParticle = balls->at(i);

	    // Add bounded uniform noise to copies if multiple copies
	    //  of a particle are generated
	    newParticle.ball.pos += cv::Point3d(rng.uniform(-4.0, 4.0),
						rng.uniform(-4.0, 4.0),
						rng.uniform(-4.0, 4.0));
	    newParticle.ball.vel += cv::Point3d(rng.uniform(-4.0, 4.0),
						rng.uniform(-4.0, 4.0),
						rng.uniform(-4.0, 4.0));        
	    X->push_back(newParticle);
	}
	delete balls;
	balls = X;

	// Weigh the resampled particles uniformly
	weighUniformly(X, 1.0/numParticles);
    }
};

class ParticleFilter {
    unsigned int numParticles;
    int t;
	
    public:

	particles p;

	ParticleFilter(bounds init_bounds, unsigned int n = 1000)  //n is the number of particles
	{
		t = 0; // start time at 0
		numParticles = n;
		vector <ball_particle> vect_ball_partcls = sample_uniformly(n, init_bounds); // sample uniformly
		p = particles(vect_ball_partcls); // initialize the particles uniformly
	}

	void Move_Particles(void (*Motion)(particles &, float t))
	{
		Motion(p, (1.0/fps)); //moves each particle by fixed amount + noise| particles changed in place
		t += 1;  // t passes by one
	}

	particles get_beliefs()
	{
		return p;
	}

	void observe(observations obsv, double (*prob_func)(observations, ball_particle))
	{
		// update beliefs:  P(X_t | e_1:t, e') ~ P(e'|X_t) * P(X_t | e_1:t)
		// Changes the particle distribution, by updating the particles according to the observation

		for(unsigned int i = 0; i <p.prtcls.size(); i++)
		{
			p.prtcls[i].w = prob_func(obsv, p.prtcls[i].part);

		}
		vector <weighed_particle> resampled = p.resample(); //resample based on the new weights
		p.prtcls = resampled; // see: http://www.cplusplus.com/reference/stl/vector/operator=/ for '=' operator for <vectors>
	}

};




double probDistNormal(observations obs, ball_particle p) {
    double prob = 0.0, min_dist = 100000;

	for(unsigned int i = 0; i < obs.blobs_pos.size(); i++ ) {
	    double dst = dist(obs.camera.coors, obs.blobs_pos[i], p.pos) ; //calculates the 3d distance of a point from a line
	    if(dst < min_dist)
		min_dist  = dst;
	}

	(p.pos.z <= -5)? prob = 0: prob = 1/min_dist;
	return prob;
}


void MoveParticlesConstantAcceleration(particles & p, float t)
{
	/* Changes particles to reflect their motion
	 * t: is the time to be used in the calculations
	 */

	RNG r; // opencv random number generator
	double g = -980.7; //acceleration = 9.807 m/s (source: wolframalpha.com)
	for(unsigned int i =0; i< p.prtcls.size(); i++)
	{
		// Deterministic: Physical Model + Gaussian Noise

		if (abs(p.prtcls[i].part.pos.z) <=0)
			p.prtcls[i].part.v.z *= -1.0; // bounce up if hits the ground

		p.prtcls[i].part.v.x += r.gaussian((double) 100);
		p.prtcls[i].part.v.y += r.gaussian((double) 100);
		p.prtcls[i].part.v.z += r.gaussian((double) 100) + g*t;

		p.prtcls[i].part.pos.x += p.prtcls[i].part.v.x*t + r.gaussian((double) 10);
		p.prtcls[i].part.pos.y += p.prtcls[i].part.v.y*t + r.gaussian((double) 10);
		p.prtcls[i].part.pos.z += p.prtcls[i].part.v.z*t + r.gaussian((double) 10); // Downwards accelerated motion (in the -Z direction)
    }
}

#endif
