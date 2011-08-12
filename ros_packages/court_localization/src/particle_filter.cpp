// -*- indent-tabs-mode: nil; c-basic-offset: 4 -*-
/*
 * particle_filter.cpp
 *
 * AUTHOR: John Wang
 * VERSION: 0.1 (11 Aug 2011)
 *
 * DESCRIPTION:
 * Particle filter for court localization.
 */
#include <stdio.h>
#include "particle_filter.h"

using namespace std;

double weightSum(vector<PoseParticle> *particles) {
    double weightSum = 0.0;
    for (unsigned int i = 0; i < particles->size(); i += 1)
        weightSum += particles->at(i).weight;
    return weightSum;
}

void weighUniformly(vector<PoseParticle> *particles, double weight) {
    for (unsigned int i = 0; i < particles->size(); i += 1)
        particles->at(i).weight = weight;
}

ParticleFilter::ParticleFilter() {
    numParticles = 0;
    particles = new vector<PoseParticle>();
}

ParticleFilter::ParticleFilter(vector<PoseParticle> *initialParticles) {
    // numParticles is the *initial* particle count
    numParticles = initialParticles->size();
    particles = initialParticles;
}

void ParticleFilter::initializeUniformly(unsigned int n, Bounds xRange, 
                                         Bounds yRange, Bounds thetaRange) {
    cv::RNG rng;
    particles->reserve(n);
    numParticles = n;
    while (n > 0) {
        double x = rng.uniform(xRange.min, xRange.max),
            y = rng.uniform(yRange.min, yRange.max),
            theta = rng.uniform(thetaRange.min, thetaRange.max);
        particles->push_back(PoseParticle(x, y, theta, 1));
        n -= 1;
    }
    normalize();
}

void ParticleFilter::observe(cv::Mat &observation) {
    for (unsigned int i = 0; i < particles->size(); i += 1) {
        // TODO emissionFn(observation, particles[i]);
    }
}

/* Move each particle by the specified amount, with Gaussian noise */
void ParticleFilter::transition(Pose movement, double sigma_xy,
                                double sigma_theta) {
    cv::RNG rng;
    for (unsigned int i = 0; i < particles->size(); i += 1) {
        particles->at(i).pose.x += movement.x + rng.gaussian(sigma_xy);
        particles->at(i).pose.y += movement.y + rng.gaussian(sigma_xy);
        particles->at(i).pose.theta += 
            movement.theta + rng.gaussian(sigma_theta);
    }
}

/* Low variance sampler from Probabilistic Robotics, Thrun et al, p 110 */
void ParticleFilter::resample() {
    normalize();

    vector<PoseParticle> *X = new vector<PoseParticle>();
    X->reserve(numParticles);
    
    cv::RNG rng;
    //r is a random number between 0 and (1/number_of_particles)
    double r = rng.uniform(0.0, 1.0/numParticles);
    //initialize sum of weights to the weight of the first particle
    double c = particles->front().weight;

    unsigned int i = 0;
    for (unsigned int m = 0; m < numParticles; m += 1) {
        double U = r + m*(1.0/numParticles);
        while (U > c) {
            i += 1;
            c = c + particles->at(i).weight;
        }
        X->push_back(particles->at(i));
    }
    delete particles; // deallocate former array
    particles = X;

    // Weigh the resampled particles uniformly
    weighUniformly(X, 1.0/numParticles);
}

void ParticleFilter::normalize() {
    double sum = weightSum(particles);

    if (sum != 0)
        for (unsigned int i = 0; i < particles->size(); i += 1)
            particles->at(i).weight /= sum;
}

const vector<PoseParticle> & ParticleFilter::getBeliefs() const {
    return *particles;
}

/* Print beliefs, up to LIMIT */
void ParticleFilter::print(int limit) const {
    unsigned int N = limit;
    if (N > particles->size()) N = particles->size();

    for (unsigned int i = 0; i < N; i += 1) {
        PoseParticle &p = particles->at(i);
        printf("%.2f\t%.2f\t%.2f\t%.2f\n", p.pose.x, p.pose.y,
               p.pose.theta, p.weight);
    }
}

int main() {
    // Test stub
    ParticleFilter pf;
    pf.initializeUniformly(10, Bounds(0,10), Bounds(0,10), Bounds(0,6.28));
    pf.transition();

    pf.print();
}
