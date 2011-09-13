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
#include <time.h>

#include "backproject.h"
#include "particle_filter.h"
#include "findlines.h"
#include "geometry.h"

using namespace cv;
using namespace std;

// Globals
ParticleFilter pf;
ros::Publisher particlePub;
cv::RNG myRng(time(NULL));

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

void drawUniformly(vector<PoseParticle> *particles, unsigned int n,
                   Bounds xRange, Bounds yRange, Bounds thetaRange) {
    particles->reserve(particles->size() + n);
    while (n > 0) {
        double x = myRng.uniform(xRange.min, xRange.max),
            y = myRng.uniform(yRange.min, yRange.max);
            //theta = myRng.uniform(thetaRange.min, thetaRange.max);
        for (int i = 0; i < 16; i += 1)
            particles->push_back(PoseParticle(x, y, i*CV_PI/8, 1));
        n -= 1;
    }
}

/* Initializes an empty particle filter */
ParticleFilter::ParticleFilter() {
    initialize(new vector<PoseParticle>());
}

/* Initializes a particle filter with the given initial particles.
 * Note that this does _not_ make a deep copy of the vector.
 */
ParticleFilter::ParticleFilter(vector<PoseParticle> *initialParticles) {
    initialize(initialParticles);
}

void ParticleFilter::initialize(vector<PoseParticle> *initialParticles) {
    // numParticles is the *initial* particle count
    numParticles = initialParticles->size();
    particles = initialParticles;
    normalize();
}

/* Clears the particle filter and (re)initializes it with N particles
 * using a uniform distribution over the given bounds.
 */
void ParticleFilter::initializeUniformly(unsigned int n, Bounds xRange,
                                         Bounds yRange, Bounds thetaRange) {
    particles->clear();
    particles->reserve(n);
    numParticles = n * 16;
    while (n > 0) {
        double x = myRng.uniform(xRange.min, xRange.max),
            y = myRng.uniform(yRange.min, yRange.max);
        // Seed each position with 32 particles
        //theta = myRng.uniform(thetaRange.min, thetaRange.max);
        for (int i = 0; i < 16; i += 1) {
            particles->push_back(PoseParticle(x, y, i*CV_PI/8, 1));
        }
        n -= 1;
    }
    normalize();
}

void ParticleFilter::observe(cv::Mat &observation) {
    // Detect lines in the image
    vector<Vec4i> seenLines;
    findLines(observation, seenLines);

    // Transform visible lines to robot frame
    camera cam;
    cam.position.z = 33;  // fixed camera height = 33cm
    cam.pan = 0;
    cam.tilt = -15.6/180.0*CV_PI;

    // Draw lines in ground coordinates (robot frame)
    vector<Vec4d> lines;
    Mat observed = Mat::zeros(480, 640, CV_8U);
    for (vector<Vec4i>::const_iterator it =
             seenLines.begin(); it < seenLines.end(); it += 1) {
        const Vec4i &line = *it;

        Point2d pt1 = cameraPointToRobot(Point2d(line[0], line[1]), cam),
            pt2 = cameraPointToRobot(Point2d(line[2], line[3]), cam);
        lines.push_back(Vec4d(pt1.x, pt1.y, pt2.x, pt2.y));

        pt1.x += 320;
        pt1.y = 480 - pt1.y;
        pt2.x += 320;
        pt2.y = 480 - pt2.y;
        cv::line(observed, pt1, pt2, Scalar(255), 2);
    }
    imshow("ground view", observed);

    // Filter detected lines by distance
    for (int i = lines.size()-1; i >= 0; i -= 1) {
        // If either y value exceeds what is reasonable on a tennis court,
        //  discard it
        if (lines[i][1] > 400 && lines[i][3] > 400) {
            lines.erase(lines.begin() + i);
            seenLines.erase(seenLines.begin() + i);
        }
    }

    // If no lines visible, count it as a non-observation
    if (seenLines.size() == 0)
        return;

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

        // If there are no visible lines (usually because robot is outside
        //  the court facing out), don't do anything. This prevents a
        //  noisy line from weighting all particles down to 0
        // NO this is a BAD IDEA
        //if (modelLines.size() == 0) {
        //    particle.weight *= 0.1;
        //    continue;
        //}

        /*
        Mat tmp = Mat::zeros(480, 640, CV_8U);
        for (vector<line_segment_all_frames>::const_iterator it =
                 modelLines.begin(); it < modelLines.end(); it += 1) {
            const line_segment_all_frames &modelLine = *it;
            cv::line(tmp, modelLine.imgPlane.pt1, modelLine.imgPlane.pt2,
                     Scalar(255), 2);
        }
        imshow("test2", tmp);

        tmp = Mat::zeros(480, 640, CV_8U);
        for (vector<line_segment_all_frames>::const_iterator it =
                 modelLines.begin(); it < modelLines.end(); it += 1) {
            const line_segment_all_frames &modelLine = *it;
            fprintf(stderr, "Line: %.2f,%.2f to %.2f,%.2f\n",
                    modelLine.camWorld.pt1.x,
                    modelLine.camWorld.pt1.z,
                    modelLine.camWorld.pt2.x,
                    modelLine.camWorld.pt2.z);
            cv::Point pt1(cvRound(modelLine.camWorld.pt1.x),
                      cvRound(modelLine.camWorld.pt1.z)),
                pt2(cvRound(modelLine.camWorld.pt2.x),
                    cvRound(modelLine.camWorld.pt2.z));

            pt1.x += 320;
            pt1.y = 480 - pt1.y;
            pt2.x += 320;
            pt2.y = 480 - pt2.y;
            cv::line(tmp, pt1, pt2, Scalar(255), 2);
        }
        imshow("test", tmp);
        */

        // Calculate the weight for each line segment seen
        // Let's weight each line equally
        double weight = 0.0;
        /* This one uses lines in ground coordinates
        // but is prone to camera shake
        for (vector<Vec4d>::const_iterator it2 =
                 lines.begin(); it2 < lines.end(); it2 += 1) {
            const Vec4d &viewLine = *it2;
            Vec2d pt1(viewLine[0], viewLine[1]),
                pt2(viewLine[2], viewLine[3]);
            double viewLineAngle = lineAngle(viewLine);

            // Calculate the weight contribution of each model line
            for (vector<line_segment_all_frames>::const_iterator it =
                     modelLines.begin(); it < modelLines.end(); it += 1) {
                const line_segment_all_frames &modelLine = *it;

                Vec4d line(modelLine.camWorld.pt1.x,
                           modelLine.camWorld.pt1.z,
                           modelLine.camWorld.pt2.x,
                           modelLine.camWorld.pt2.z);
                double modelLineAngle = lineAngle(line);
                double headingError = 
                    abs(normalizeRadians(modelLineAngle - viewLineAngle));
                double metric = pointLineSegmentDistance(pt1, line) +
                    pointLineSegmentDistance(pt2, line);
                metric = metric * exp(3 * headingError);

                weight += exp(-0.005*metric - 7*headingError);
            }
        }
        //*/
        //* This one uses the backprojection model
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
        //*/

        // Reweight the particle
        particle.weight *= weight;
    }
}

/* Move each particle by the specified amount, with Gaussian noise */
void ParticleFilter::transition(double dist, double dtheta, double sigma_dist,
                                double sigma_theta) {
    for (unsigned int i = 0; i < particles->size(); i += 1) {
        PoseParticle &p = particles->at(i);

        // Add noise using the motion model
        double newTheta = normalizeRadians(p.pose.theta + dtheta +
                                           myRng.gaussian(0.01*abs(dtheta)));
        dist += myRng.gaussian(abs(0.005*dist));

        p.pose.x += dist*cos(newTheta);
        p.pose.y += dist*sin(newTheta);
        p.pose.theta = newTheta;
    }
}

/* Low variance sampler from Probabilistic Robotics, Thrun et al, p 110 */
void ParticleFilter::resample() {
    normalize();

    vector<PoseParticle> *X = new vector<PoseParticle>();
    X->reserve(numParticles);
    
    //r is a random number between 0 and (1/number_of_particles)
    double r = myRng.uniform(0.0, 1.0/numParticles);
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

    // TODO Undo hack! Publish every N particles to save time
    for (size_t i = 0; i < particles->size(); i += 20) {
        const PoseParticle &p = particles->at(i);
        bb_msgs::Pose poseMsg;
        poseMsg.x = p.pose.x;
        poseMsg.y = p.pose.y;
        poseMsg.theta = p.pose.theta;
        msg.data.push_back(poseMsg);
    }
    pub.publish(msg);

    /* Publish average
    for (size_t i = 0; i < len; i += 20) {
        const PoseParticle &p = particles->at(i);
        x += p.pose.x;
        y += p.pose.y;
        theta_x += cos(p.pose.theta);
        theta_y += sin(p.pose.theta);
    }
    bb_msgs::Pose poseMsg;
    poseMsg.x = x / len;
    poseMsg.y = y / len;
    poseMsg.theta = atan2(theta_y, theta_x);
    msg.data.push_back(poseMsg);
    pub.publish(msg);
    //*/
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
        double ws = weightSum(pf.particles);
        fprintf(stderr, "Weight sum: %f\n", ws);
        if (ws == 0.0) {
            fprintf(stderr, "Reinitializing particle filter\n");
            pf.particles->clear();
            pf.initializeUniformly(500, Bounds(-200,1389), Bounds(-200, 1297),
                                   Bounds(0,2*CV_PI));
        } else {
            pf.resample();
        }

        // Add gaussian noise
        /*
        RNG rng;
        for (unsigned int i = 0; i < pf.particles->size(); i += 1) {
            PoseParticle &p = pf.particles->at(i);
            p.pose.x += myRng.gaussian(0.02);
            p.pose.y += myRng.gaussian(0.02);
            p.pose.theta += myRng.gaussian(0.005);
        }
        */

        // Publish particles
        pf.publish(particlePub);

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
    //for (int i = 0; i < 100; i += 1)
        init->push_back(PoseParticle(500, -100, 3*CV_PI/8, 1));
    pf.particles = init;
    pf.numParticles = init->size();
    //*/
    //*
    vector<PoseParticle> *init = new vector<PoseParticle>();
    // Bottom strip
    drawUniformly(init, 50, Bounds(500, 800), Bounds(-200, 0),
                  Bounds(0,2*CV_PI));
    drawUniformly(init, 50, Bounds(-200, 0), Bounds(-200, 0),
                  Bounds(0,2*CV_PI));
    // Top strip
    drawUniformly(init, 50, Bounds(500, 800), Bounds(1097, 1297),
                  Bounds(0,2*CV_PI));
    drawUniformly(init, 50, Bounds(-200, 0), Bounds(1097, 1297),
                  Bounds(0,2*CV_PI));

    pf.initialize(init);
    //pf.initializeUniformly(600, Bounds(-200,1389), Bounds(-200, 1297),
    //                           Bounds(0,2*CV_PI));
    //*/
    pf.publish(particlePub);

    // Spin loop
    while (ros::ok()) {
        waitKey(5);
        ros::spinOnce();
    }
}
