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
#include <bb_msgs/OdometryStamped.h>
#include <bb_msgs/Pose.h>
#include <bb_msgs/PoseArray.h>

#include <stdio.h>
#include <sys/timeb.h>
#include <stdlib.h>
#include <time.h>

#include "backproject.h"
#include "particle_filter.h"
#include "findlines.h"
#include "geometry.h"

using namespace cv;
using namespace std;

// Globals
ParticleFilter pf;
ros::Publisher particlePub, posePub;
cv::RNG myRng(time(NULL));
double horizon = 0.0;

//cv::VideoWriter vid("video.avi",
//                    CV_FOURCC('P','I','M','1'), 30, Size(640,480), true);

// Utility functions

void printTime(const char *label) {
    /*
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
    */
}

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
            y = myRng.uniform(yRange.min, yRange.max),
            theta = myRng.uniform(thetaRange.min, thetaRange.max);
        particles->push_back(PoseParticle(x, y, theta));
        /*
        for (int i = 0; i < 16; i += 1)
            particles->push_back(PoseParticle(x, y, i*CV_PI/8, 1));
        */
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
    printTime("start observe");
    // Detect lines in the image
    vector<Vec4i> seenLines;
    findLines(observation, seenLines);
    printTime(" findlines");

    // Transform visible lines to robot frame
    camera cam;
    cam.position.z = 33;  // fixed camera height = 33cm
    cam.pan = 0;
    cam.tilt = -15.6/180.0*CV_PI;

    // Draw lines in ground coordinates (robot frame)
    vector<Vec4d> lines;
    Mat observed = Mat::zeros(480, 640, CV_8U);
    for (vector<Vec4i>::iterator it = seenLines.begin();
         it < seenLines.end(); it += 1) {
        Vec4i &line = *it;

        Point2d pt1 = cameraPointToRobot(Point2d(line[0], line[1]+horizon), 
                                         cam),
            pt2 = cameraPointToRobot(Point2d(line[2], line[3]+horizon),
                                     cam);
        lines.push_back(Vec4d(pt1.x, pt1.y, pt2.x, pt2.y));

        /*
        pt1.x += 320;
        pt1.y = 480 - pt1.y;
        pt2.x += 320;
        pt2.y = 480 - pt2.y;
        cv::line(observed, pt1, pt2, Scalar(255), 2);
        */
    }
    //imshow("ground view", observed);
    //*/

    // Filter detected lines by distance
    for (int i = lines.size()-1; i >= 0; i -= 1) {
        // If either y value exceeds what is reasonable on a tennis court,
        //  discard it
        //if (lines[i][1] > 400 && lines[i][3] > 400) {

        // If line is more than 4m away, discard it
        if (pointLineDistance(Vec2d(0,0), lines[i]) > 300) {
            lines.erase(lines.begin() + i);
            seenLines.erase(seenLines.begin() + i);
        }
    }
    printTime(" conv to ground plane");

    // If no lines visible, count it as a non-observation
    // This was perhaps too harsh
    //if (seenLines.size() == 0)
    //    return;

    // Draw detected lines for debugging
    drawLines(observation, seenLines);

    // Adjust y coordinates (since the ROI is shifted,
    //  all the y coordinates will be off)
    for (vector<Vec4i>::iterator it = seenLines.begin();
         it < seenLines.end(); it += 1) {
        Vec4i &line = *it;
        line[1] += horizon;
        line[3] += horizon;
    }

    // Reweight each particle
    for (size_t i = 0; i < particles->size(); i += 1) {
        PoseParticle &particle = particles->at(i);

        cam.position.x = particle.pose.x;
        cam.position.y = particle.pose.y;
        cam.theta = particle.pose.theta;

        // Find lines that should be visible
        vector<line_segment_all_frames> modelLines = 
            get_view_lines(cam, Size(640,480), observation);

        // Filter out lines that are far away
        Mat tmp = Mat::zeros(480, 640, CV_8U);
        for (int i = modelLines.size()-1; i >= 0; i -= 1) {
            const line_segment_all_frames &modelLine = modelLines[i];
            Vec4d line(modelLine.camWorld.pt1.x, modelLine.camWorld.pt1.z,
                       modelLine.camWorld.pt2.x, modelLine.camWorld.pt2.z);
            // If line is more than 5m away, discard it
            if (pointLineDistance(Vec2d(0,0), line) > 500) {
                modelLines.erase(modelLines.begin() + i);
            }
        }

        // Precompute the matching score (weight) for each pair of
        //  (visible line, model line) using the backprojection model
        double weights[seenLines.size()][modelLines.size()];
        for (size_t j = 0; j < seenLines.size(); j += 1) {
            const Vec4i &viewLine = seenLines[j];
            Vec2i pt1(viewLine[0], viewLine[1]),
                pt2(viewLine[2], viewLine[3]);
            double viewLineAngle = lineAngle(viewLine);

            for (size_t k = 0; k < modelLines.size(); k += 1) {
                const line_segment_all_frames &modelLine = modelLines[k];
                Vec4i line = pointsToLine(modelLine.imgPlane.pt1,
                                          modelLine.imgPlane.pt2);
                double modelLineAngle = lineAngle(line);
                double headingError = 
                    abs(normalizeRadians(modelLineAngle - viewLineAngle));
                double metric = pointLineSegmentDistance(pt1, line) +
                    pointLineSegmentDistance(pt2, line);
                metric = metric * exp(3 * headingError);
                weights[j][k] = exp(-0.003*metric - 3*headingError);
            }
        }

        // Calculate the weight for each line segment seen
        double w1 = 1.0,
            w2 = 1.0;

        // for each detected line:
        //   w1 *= max(weight of model lines)
        // In the case of extra lines w1 will be low (also, if you weren't
        //  supposed to see any lines but you do, w1 = 0)
        for (size_t j = 0; j < seenLines.size(); j += 1) {
            double max = 0.0;
            for (size_t k = 0; k < modelLines.size(); k += 1) {
                if (weights[j][k] > max)
                    max = weights[j][k];
            }
            w1 *= 1 + max;
        }
        // for each model line:
        //   w2 *= max(weight of detected lines)
        // In the case of missing lines w2 will be low (also, if you didn't
        //  see any lines but you were supposed to, w2 = 0)
        if (modelLines.size() == 0) {
            w2 *= 0.5;
        }
        for (size_t j = 0; j < modelLines.size(); j += 1) {
            double max = 0.0;
            for (size_t k = 0; k < seenLines.size(); k += 1) {
                if (weights[k][j] > max)
                    max = weights[k][j];
            }
            w2 *= max;
        }

        // Reweight the particle
        // w2 has to be smaller because you don't see a lot of the model lines
        // especially the faraway ones
        particle.weight *= w1 + 2*w2;
    }
    printTime(" reweight");
}

/* Move each particle by the specified amount, with Gaussian noise */
void ParticleFilter::transition(double dist, double dtheta, double sigma_dist,
                                double sigma_theta) {
    for (unsigned int i = 0; i < particles->size(); i += 1) {
        PoseParticle &p = particles->at(i);

        // Add noise using the motion model
        double newTheta = normalizeRadians(p.pose.theta + dtheta +
                                           myRng.gaussian(0.01*abs(dtheta)));
        dist += myRng.gaussian(abs(0.01*dist));

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
        bool isMultipleCopy = true;
        while (U > c) {
            i += 1;
            c = c + particles->at(i).weight;
            isMultipleCopy = false;
        }
        PoseParticle newParticle = particles->at(i);

        // Add bounded uniform noise to copies if multiple copies
        //  of a particle are generated
        newParticle.pose.x += myRng.uniform(-4.0, 4.0);
        newParticle.pose.y += myRng.uniform(-4.0, 4.0);
        newParticle.pose.theta += myRng.uniform(-0.005, 0.005);
        
        X->push_back(newParticle);
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
    int len = particles->size(),
        stepSize = 1;
    if (len > 1000)
        stepSize = 4;
    for (size_t i = 0; i < len; i += stepSize) {
        const PoseParticle &p = particles->at(i);
        bb_msgs::Pose poseMsg;
        poseMsg.x = p.pose.x;
        poseMsg.y = p.pose.y;
        poseMsg.theta = p.pose.theta;
        msg.data.push_back(poseMsg);
    }
    pub.publish(msg);

    //* Publish average
    double x = 0,
        y = 0,
        thetaX = 0,
        thetaY = 0;
    for (int i = 0; i < len; i += 1) {
        const PoseParticle &p = particles->at(i);
        x += p.pose.x;
        y += p.pose.y;
        thetaX += cos(p.pose.theta);
        thetaY += sin(p.pose.theta);
    }
    bb_msgs::Pose poseMsg;
    poseMsg.x = 0.01*x / len; // converting to m
    poseMsg.y = 0.01*y / len; // converting to m

    poseMsg.theta = atan2(thetaY, thetaX);
    fprintf(stderr, "Avg pose: %.2f, %.2f @ %f deg ", poseMsg.x, poseMsg.y,
            poseMsg.theta * 180 / CV_PI);
    posePub.publish(poseMsg);
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
        fprintf(stderr,"%.2f\t%.2f\t%.2f\t%f\n", p.pose.x, p.pose.y,
               p.pose.theta, p.weight);
    }
}

/* On receiving odometry data, update this particle filter with a transition */
void odometryCallback(const bb_msgs::OdometryStampedConstPtr& msg) {
    // Transition particles according to motion model
    double dtheta = -1 * msg->odometry.heading_delta;
    double dist = msg->odometry.distance_delta * 100;      // convert to cm
    pf.transition(dist, dtheta);
}

/* On receiving an image, update this particle filter with the observation. */
void imageCallback(const sensor_msgs::ImageConstPtr& msg) {

    printTime("start");
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        printTime("cvbridge conversion");

        // Update particle filter
        cv_ptr->image.adjustROI(-horizon, 0, 0, 0);
        pf.observe(cv_ptr->image);
        printTime("observe");

        // If weight sum falls below some threshold, reinitialize the filter
        double ws = weightSum(pf.particles);
        fprintf(stderr, "Weight sum: %f\n", ws);
        if (ws == 0.0) {
            fprintf(stderr, "Reinitializing particle filter\n");
            pf.particles->clear();
            pf.initializeUniformly(500, Bounds(-200,1389), Bounds(-200, 1297),
                                   Bounds(0,2*CV_PI));
        } else {
            /*/ Add random particles
            for (int i = 0; i < 200; i += 1) {
                double x = myRng.uniform(-200.0, 1389.0),
                    y = myRng.uniform(-200.0, 1297.0),
                    theta = myRng.uniform(0.0,2*CV_PI);
                pf.particles->push_back(PoseParticle(x, y, theta,
                                                     1.0/pf.numParticles));
            }
            //*/
            // Resample
            pf.resample();
            printTime("resample");
        }
        printTime("compute");
        // Publish particles
        pf.publish(particlePub);

        printTime("publish");

        // Restore whole image
        cv_ptr->image.adjustROI(horizon, 0, 0, 0);
        imshow("image", cv_ptr->image);
        //vid << cv_ptr->image;
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
    //namedWindow("image", CV_WINDOW_AUTOSIZE);

    // Create particle publisher to topic "filter/particles"
    particlePub = nh.advertise<bb_msgs::PoseArray>("filter/particles", 4);
    posePub = nh.advertise<bb_msgs::Pose>("pose", 10);

    // Create image listener on topic ~image (default "gscam/image_raw")
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imageSub;
    string imageTopic;
    if (!nhPrivate.hasParam("image_transport"))
        nhPrivate.setParam("image_transport", "compressed");
    nhPrivate.param<string>("image", imageTopic, "camera/image");
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

    int initParticles = 25;
    if (argc > 1) {
        initParticles = atoi(argv[1]);
    }
    //*
    vector<PoseParticle> *init = new vector<PoseParticle>();
    // Whole court
    //drawUniformly(init, 5000, Bounds(-200,1389), Bounds(-200, 1297),
    //                           Bounds(0,2*CV_PI));
    // Bottom strip
    drawUniformly(init, initParticles, Bounds(300, 800), Bounds(-400, 100),
                  Bounds(0,2*CV_PI));
    //    drawUniformly(init, 50, Bounds(-200, 0), Bounds(-250, 0),
    //                  Bounds(0,2*CV_PI));
    // Top strip
    //drawUniformly(init, 50, Bounds(400, 800), Bounds(997, 1397),
    //              Bounds(0,2*CV_PI));
    //    drawUniformly(init, 50, Bounds(-200, 0), Bounds(1097, 1297),
    //                  Bounds(0,2*CV_PI));

    pf.initialize(init);
    //pf.initializeUniformly(200, Bounds(-200,1389), Bounds(-200, 1297),
    //                           Bounds(0,2*CV_PI));
    //*/
    pf.publish(particlePub);

    // Calculate ROI cutoff
    camera cam;
    cam.position.z = 33.0;
    cam.tilt = -15.0 * CV_PI / 180.0;
    Point3d camWorldPt =
        get_camera_world_coordinates(Point3d(8000, 0, -cam.position.z),
                                     cam.position, cam.theta, cam.pan,
                                     cam.tilt);
    Point2d horizonPt =
        cam_world_position_to_imageXY(camWorldPt, cam);
    horizon = horizonPt.y;

    // Spin loop
    while (ros::ok()) {
        waitKey(5);
        ros::spinOnce();
    }
}
