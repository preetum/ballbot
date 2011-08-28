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
#include <cv.h>
#include <highgui.h>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <stdio.h>

#include "backproject.h"
#include "particle_filter.h"

#define PI 3.14159265359

using namespace cv;
using namespace std;

// Utility functions
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

/* Initializes an empty particle filter */
ParticleFilter::ParticleFilter() {
    numParticles = 0;
    particles = new vector<PoseParticle>();
}

/* Initializes a particle filter with the given initial particles.
 * Note that this does _not_ make a deep copy of the vector.
 */
ParticleFilter::ParticleFilter(vector<PoseParticle> *initialParticles) {
    // numParticles is the *initial* particle count
    numParticles = initialParticles->size();
    particles = initialParticles;
}

/* Clears the particle filter and (re)initializes it with N particles
 * using a uniform distribution over the given bounds.
 */
void ParticleFilter::initializeUniformly(unsigned int n, Bounds xRange, 
                                         Bounds yRange, Bounds thetaRange) {
    cv::RNG rng;
    particles->clear();
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

double backprojectionWeight(cv::Mat &observation,
                            const PoseParticle &particle) {
    camera particle_camera;
    particle_camera.position.x = particle.pose.x;
    particle_camera.position.y = particle.pose.y;
    particle_camera.position.z = 33;  // fixed camera height = 33cm
    particle_camera.theta = particle.pose.theta;

    vector<line_segment_all_frames> segments = 
        get_view_lines(particle_camera, Size(640,480), observation);

    return 0;
}

void ParticleFilter::observe(cv::Mat &observation) {
    for (unsigned int i = 0; i < particles->size(); i += 1) {
        backprojectionWeight(observation, particles->at(i));
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

/* On receiving an image, update this particle filter with the observation. */
void ParticleFilter::imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        observe(cv_ptr->image);
        imshow("image", cv_ptr->image);
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "image_converter");
    ros::NodeHandle nh;
    ros::NodeHandle nh_private("~");

    // Create particle filter
    ParticleFilter pf;
    pf.initializeUniformly(100, Bounds(0,1189), Bounds(0, 1097), 
                           Bounds(0,2*CV_PI));
    //pf.transition(Pose(), 50, 0.1);

    // Create debug windows
    namedWindow("image", CV_WINDOW_AUTOSIZE);

    // Create ROS image listener
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber image_sub;
    string image_topic;
    if (!nh_private.hasParam("image_transport"))
        nh_private.setParam("image_transport", "compressed");
    nh_private.param<string>("image", image_topic, "gscam/image_raw");
    it.subscribe("gscam/image_raw", 1, &ParticleFilter::imageCallback, &pf);

    // Spin
    ros::spin();
}
