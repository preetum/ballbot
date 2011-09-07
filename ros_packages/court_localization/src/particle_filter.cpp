// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
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
#include <bb_msgs/Odometry.h>
#include <bb_msgs/Pose.h>
#include <bb_msgs/PoseArray.h>

#include <stdio.h>
#include <sys/timeb.h>

#include "backproject.h"
#include "particle_filter.h"
#include "findlines.h"
#include "geometry.h"

using namespace cv;
using namespace std;

// Globals
ParticleFilter pf;
ros::Publisher particlePub;

// Utility functions
/*
void printTime(const char *label) {
    static struct timeb prev = {0,0};
    struct timeb cur;
    double diff = 0;
    ftime(&cur);
    if (prev.time) {
        diff  =  cur.time    - prev.time;
        diff += (cur.millitm - prev.millitm)/1000.0;
    }
    fprintf(stderr,"%30s  start = %d.%-3hu (+%5.3f)\n",
              label, (int)cur.time, cur.millitm, diff);
    prev = cur;
}
*/

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

void ParticleFilter::observe(cv::Mat &observation) {
    // Detect lines in the image
    vector<Vec4i> seenLines;
    findLines(observation, seenLines);

    // If no lines visible, count it as a non-observation
    if (seenLines.size() == 0)
        return;
    
    // Transform visible lines to robot frame
    camera cam;
    cam.position.z = 33;  // fixed camera height = 33cm
    cam.pan = 0;
    cam.tilt = -CV_PI/8;

    vector<Vec4d> lines;
    Mat observed = Mat::zeros(480, 640, CV_8U);
    for (vector<Vec4i>::const_iterator it =
             seenLines.begin(); it < seenLines.end(); it += 1) {
        const Vec4i &line = *it;

        Point2d pt1 = cameraPointToRobot(Point2d(line[0], line[1]), cam),
            pt2 = cameraPointToRobot(Point2d(line[2], line[3]), cam);
        lines.push_back(Vec4d(pt1.x, pt1.y, pt2.x, pt2.y));

        // Draw points
        pt1.x += 320;
        pt1.y = 480 - pt1.y;
        pt2.x += 320;
        pt2.y = 480 - pt2.y;
        cv::line(observed, pt1, pt2, Scalar(255), 2);
    }
    imshow("blah", observed);

    for (int i = lines.size()-1; i >= 0; i -= 1) {
        // If either y value exceeds what is reasonable on a tennis court,
        //  discard it
        if (lines[i][1] > 2000 || lines[i][3] > 2000) {
            lines.erase(lines.begin() + i);
            seenLines.erase(seenLines.begin() + i);
        }
    }

    // Draw detected lines for debugging
    drawLines(observation, seenLines);

    // Reweight each particle
    for (unsigned int i = 0; i < particles->size(); i += 1) {
        PoseParticle &particle = particles->at(i);

        cam.position.x = particle.pose.x;
        cam.position.y = particle.pose.y;
        cam.theta = particle.pose.theta;

        // Find lines that should be visible
        vector<line_segment_all_frames> modelLines = 
            get_view_lines(cam, Size(640,480), observation);

        // Calculate the weight for each line segment seen
        // Let's weight each line equally
        double weight = 0.0;
        for (vector<Vec4i>::const_iterator it2 =
                 seenLines.begin(); it2 < seenLines.end(); it2 += 1) {
            const Vec4i &viewLine = *it2;
            Vec2i pt1(viewLine[0], viewLine[1]),
                pt2(viewLine[2], viewLine[3]);
            double viewLineAngle = lineAngle(viewLine);

            // Calculate the weight contribution of each model line
            for (vector<line_segment_all_frames>::const_iterator it =
                     modelLines.begin(); it < modelLines.end(); it += 1) {
                const line_segment_all_frames &modelLine = *it;
                Vec4i line = pointsToLine(modelLine.imgPlane.pt1,
                                          modelLine.imgPlane.pt2);
                double modelLineAngle = lineAngle(line);
                double headingError = 
                    abs(normalizeRadians(modelLineAngle - viewLineAngle));
                double metric = pointLineSegmentDistance(pt1, line) +
                    pointLineSegmentDistance(pt2, line);
                metric = metric * exp(3 * headingError);

                weight += exp(-0.003*metric - 7*headingError);
            }
        }

        // Reweight the particle
        particle.weight *= weight;
    }
}

/* Move each particle by the specified amount, with Gaussian noise */
void ParticleFilter::transition(double dist, double dtheta, double sigma_dist,
                                double sigma_theta) {
    cv::RNG rng;
    for (unsigned int i = 0; i < particles->size(); i += 1) {
        PoseParticle &p = particles->at(i);

        // Add noise using the motion model
        double newTheta = p.pose.theta + dtheta +
            rng.gaussian(2*abs(dtheta));
        dist += rng.gaussian(abs(0.1*dist));

        p.pose.x += dist*cos(newTheta);
        p.pose.y += dist*sin(newTheta);
        p.pose.theta = normalizeRadians(newTheta +
                                        rng.gaussian(2*abs(dtheta)));
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

void ParticleFilter::publish(ros::Publisher &pub) const {
    bb_msgs::PoseArray msg;

    for (size_t i = 0; i < particles->size(); i += 1) {
        const PoseParticle &p = particles->at(i);
        bb_msgs::Pose poseMsg;
        poseMsg.x = p.pose.x;
        poseMsg.y = p.pose.y;
        poseMsg.theta = p.pose.theta;
        msg.data.push_back(poseMsg);
    }
    pub.publish(msg);
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
        printf("%.2f\t%.2f\t%.2f\t%f\n", p.pose.x, p.pose.y,
               p.pose.theta, p.weight);
    }
}

/* On receiving odometry data, update this particle filter with a transition */
void odometryCallback(const bb_msgs::OdometryConstPtr& msg) {
    static double lastHeading = HUGE_VAL;

    // On first message, initialize lastHeading
    if (lastHeading == HUGE_VAL)
        lastHeading = msg->heading;

    // Transition particles according to motion model
    double dtheta = lastHeading - msg->heading;
    double dist = msg->distance_delta * 100;      // convert to cm
    pf.transition(dist, dtheta);

    // Store last heading
    lastHeading = msg->heading;
}

/* On receiving an image, update this particle filter with the observation. */
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        // Update particle filter
        pf.observe(cv_ptr->image);
        // If weight sum falls below some threshold, reinitialize the filter
        if (weightSum(pf.particles) <= 0.01) {
            fprintf(stderr, "Reinitializing particle filter\n");
            pf.particles->clear();
            pf.initializeUniformly(500, Bounds(-200,1389), Bounds(-200, 1297),
                                   Bounds(0,2*CV_PI));
        } else {
            pf.resample();
        }

        // Add gaussian noise
        RNG rng;
        for (unsigned int i = 0; i < pf.particles->size(); i += 1) {
            PoseParticle &p = pf.particles->at(i);
            p.pose.x += rng.gaussian(5);
            p.pose.y += rng.gaussian(5);
            p.pose.theta += rng.gaussian(0.01);
        }

        // Publish particles
        pf.publish(particlePub);

        /*
        // Draw view lines
        camera particle_camera;
        particle_camera.position.x = argmax.pose.x;
        particle_camera.position.y = argmax.pose.y;
        particle_camera.position.z = 33;  // fixed camera height = 33cm
        particle_camera.theta = argmax.pose.theta;
        particle_camera.tilt = -CV_PI/8;
        get_view_lines(particle_camera, Size(640,480), 
                       cv_ptr->image, 0.2, true);
        */
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
    ros::NodeHandle nhPrivate("~");

    // Create debug windows
    namedWindow("image", CV_WINDOW_AUTOSIZE);

    // Create particle publisher to topic "filter/particles"
    particlePub = nh.advertise<bb_msgs::PoseArray>("filter/particles", 4);

    // Create image listener on topic ~image (default "gscam/image_raw")
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imageSub;
    string imageTopic;
    if (!nhPrivate.hasParam("image_transport"))
        nhPrivate.setParam("image_transport", "compressed");
    nhPrivate.param<string>("image", imageTopic, "gscam/image_raw");
    imageSub = it.subscribe(imageTopic, 1, imageCallback);

    // Create odometry listener on topic ~odometry (default "odometry")
    string odomTopic;
    nhPrivate.param<string>("odometry", odomTopic, "odometry");
    ros::Subscriber odomSub = nh.subscribe(odomTopic, 200, odometryCallback);

    // Initialize particle filter
    /* Testing only
    vector<PoseParticle> *init = new vector<PoseParticle>();
    for (int i = 0; i < 100; i += 1)
        init->push_back(PoseParticle(500, -100, 3*CV_PI/8, 1));
    pf.particles = init;
    pf.numParticles = init->size();
    pf.transition(500, 1);
    //*/
    pf.initializeUniformly(500, Bounds(-200,1389), Bounds(-200, 1297),
                               Bounds(0,2*CV_PI));
    //*/
    pf.publish(particlePub);

    // Spin loop
    while (ros::ok()) {
        waitKey(10);
        ros::spinOnce();
    }
}
