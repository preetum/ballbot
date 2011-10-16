// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
/*
 * court_localization.cpp
 *
 * AUTHOR: John Wang
 * VERSION: 0.1 (4 Sep 2011)
 *
 * DESCRIPTION:
 * Kalman filter / particle filter combo for court localization.
 */
#include <math.h>

#include <cv.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <bb_msgs/Odometry.h>
#include <bb_msgs/Pose.h>
#include <bb_msgs/PoseArray.h>

using namespace std;
using namespace cv;

// Globals
ros::Publisher posePub;
cv::KalmanFilter kf;
cv::Mat_<double> kfMeasurement(3, 1),
    kfControl(2, 1);
bool kfInitialized = false;

void particleCallback(const bb_msgs::PoseArrayConstPtr& msg) {
    
}

void pfEstimateCallback(const bb_msgs::PoseConstPtr& msg) {
    // Kalman filter requires particle filter to publish its pose
    // if kf not initialized, initialize with position
    if (!kfInitialized) {
        cv::Mat_<double> initState(3, 1);
        initState(0,0) = msg->x;
        initState(1,0) = msg->y;
        initState(2,0) = msg->theta;
        kf.statePre = kf.statePost = initState;
    }

    // else if position estimate diverges, output some error pose
}

void odometryCallback(const bb_msgs::OdometryConstPtr& msg) {
    // Control vector = [distance dtheta]'
    double dist = msg->distance_delta,
        dtheta = -1 * msg->heading_delta;
    kfControl(0,0) = dist;
    kfControl(1,0) = dtheta;

    // Control matrix (B) is non-linear
    double newHeading = kf.statePost.at<double>(2) + dtheta;
    kf.controlMatrix.at<double>(0,0) = cos(newHeading);
    kf.controlMatrix.at<double>(1,0) = sin(newHeading);
    kf.controlMatrix.at<double>(2,1) = 1;

    // Do transition update
    const Mat& state = kf.predict(kfControl);

    // Publish pose estimate
    bb_msgs::Pose poseMsg;
    poseMsg.x = state.at<double>(0);
    poseMsg.y = state.at<double>(1);
    poseMsg.theta = state.at<double>(2);
    posePub.publish(msg);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

        /*
        // Make kalman filter observation
        // i.e. measurement = gradientDescent(cv_ptr->image)

        // Do transition update
        Mat& state = kf.correct(measurement);

        // Publish pose estimate
        bb_msgs::Pose poseMsg;
        poseMsg.x = prediction[0];
        poseMsg.y = prediction[1];
        poseMsg.theta = prediction[2];
        posePub.publish(msg);
        */
    }
    catch (cv_bridge::Exception& e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

int main (int argc, char** argv) {
    ros::init(argc, argv, "court_localization");
    ros::NodeHandle nh;
    ros::NodeHandle nhPrivate("~");

    // Get input topic names from parameters
    string imageTopic, odomTopic, pfTopic, pfEstimateTopic;
    nhPrivate.param<string>("image", imageTopic, "camera/image_raw");
    nhPrivate.param<string>("odometry", odomTopic, "odometry");
    nhPrivate.param<string>("particles", pfTopic, "filter/particles");
    nhPrivate.param<string>("pf_estimate", pfEstimateTopic, "filter/estimate");

    // Create listeners
    ros::Subscriber pfSub = nh.subscribe(pfTopic, 1, particleCallback),
        pfEstSub = nh.subscribe(pfEstimateTopic, 1, pfEstimateCallback),
        odometrySub = nh.subscribe(odomTopic, 200, odometryCallback);
    image_transport::ImageTransport it(nh);
    image_transport::Subscriber imageSub = 
        it.subscribe(imageTopic, 1, imageCallback);

    // Create pose publisher
    posePub = nh.advertise<bb_msgs::Pose>("pose", 10);

    // Initialize Kalman filter (conventions used below are from
    //  Probabilistic Robotics, ch. 3)
    kf.init(3, 3, 2, CV_64F);

    // Initialize transition noise (R)
    kf.processNoiseCov = Mat::eye(3, 3, CV_64F) * 1e-3;
    // Initialize measurement noise (Q)
    kf.measurementNoiseCov = Mat::eye(3, 3, CV_64F) * 1e-3;
    // Initialize the error (covariance) matrix to 1
    kf.errorCovPost = Mat::eye(3, 3, CV_64F);
    // Initialize the transition matrix (A), measurement matrix (C)
    //  and control matrix (B)
    kf.transitionMatrix = Mat::eye(3, 3, CV_64F);
    kf.measurementMatrix = Mat::eye(3, 3, CV_64F);
    kf.controlMatrix = Mat::zeros(3, 2, CV_64F);
    
    // Spin loop: service ROS callbacks
    ros::spin();
}
