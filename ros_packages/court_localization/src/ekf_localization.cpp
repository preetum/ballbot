// -*- indent-tabs-mode: nil; c-basic-offset: 4; tab-width: 4 -*-
/*
 * ekf_localization.cpp
 *
 * AUTHOR: John Wang
 * VERSION: 0.1 (4 Sep 2011)
 *
 * DESCRIPTION:
 * Kalman filter for court localization
 */
#include <math.h>

#include <cv.h>
#include <highgui.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <bb_msgs/Odometry.h>
#include <bb_msgs/Pose.h>
#include <bb_msgs/PoseArray.h>

#include "backproject.h"
#include "particle_filter.h"
#include "findlines.h"
#include "geometry.h"

using namespace std;
using namespace cv;

// Globals
ros::Publisher posePub;
cv::KalmanFilter kf;
cv::Mat_<double> kfMeasurement(3, 1),
    kfControl(2, 1),
    kfState(3, 1);

FILE *outfile = NULL;

/* Input:
 *  lines = (x1, y1, x2, y2) of lines in GROUND coordinates w.r.t robot
 *  model = lines we should be able to see
 *  
 * Output:
 *  vector of (r, alpha, k) for radius, heading angle (radians), and 
 *  correspondence to court line k
 */
vector<Vec3d> generateObservations(vector<Vec4d> detected,
                                   vector<line_segment_all_frames> model) {
    vector<Vec3d> output;

    for (int i = 0; i < detected.size(); i += 1) {
        // Calculate r and alpha from geometry
        const Vec4d &detectedLine = detected[i];
        double r = pointLineDistance(Vec2d(0, 0), detectedLine);
        double alpha = pointLineHeading(Vec2d(0, 0), detectedLine) - 
            kfState(0,2);
        
        // Calculate correspondence index k by choosing the closest
        //  fit, given our current pose
        int k = -1;
        double similarity = 0;
        for (int j = 0; j < model.size(); j += 1) {
            const line_segment_all_frames &modelLine = model[j];
            Vec4d modelCamLine =
                pointsToLine(modelLine.camWorld.pt1, modelLine.camWorld.pt2);
            double r2 = pointLineDistance(Vec2d(0,0), modelCamLine);
            double alpha2 = pointLineHeading(Vec2d(0,0), modelCamLine);
            double newSimilarity = exp((r2-r)*(r2-r)/10 + 
                                        (alpha2-alpha)*(alpha2-alpha)/0.1);
            if (newSimilarity > similarity)
                k = modelLine.index;
        }

        // I'm stuffing an int into a double. Sorry. Adding 0.1 to prevent
        //  (int)(double)k from becoming (k-1)
        if (k != -1)
            output.push_back(Vec3d(r, alpha, k+0.1));
    }

    return output;
}

// Linearize the observation fn h(x) about the current pose x
/*
cv::Mat obs_jacobian(const Pose &pose) {
    cv::Mat_<double> H(3,3);
    double eps_xy = 0.1;    // epsilon for calulating h(x+eps)
    double eps_theta = 0.01;

    // for (x, y, theta)
    for (int dim = 0; i < 3; i += 1) {
        
    }
}
*/

void ekf_correct(cv::Mat &frame) {
    // Detect lines in the image
    vector<Vec4i> seenLines;
    findLines(frame, seenLines);

    // Transform visible lines to robot frame
    camera cam;
    cam.position.z = 33;  // fixed camera height = 33cm
    cam.pan = 0;
    cam.tilt = -15.6/180.0*CV_PI;

    // Draw lines in ground coordinates (robot frame)
    vector<Vec4d> lines;
    //Mat observed = Mat::zeros(480, 640, CV_8U);

    for (vector<Vec4i>::iterator it = seenLines.begin();
         it < seenLines.end(); it += 1) {
        Vec4i &line = *it;

        Point2d pt1 = cameraPointToRobot(Point2d(line[0], line[1]), cam),
            pt2 = cameraPointToRobot(Point2d(line[2], line[3]), cam);
        lines.push_back(Vec4d(pt1.x, pt1.y, pt2.x, pt2.y));

        /* Draw line for debug
        pt1.x += 320; pt1.y = 480 - pt1.y;
        pt2.x += 320; pt2.y = 480 - pt2.y;
        cv::line(observed, pt1, pt2, Scalar(255), 2);
        //*/
    }
    //imshow("ground view", observed);
    //*/

    // For each modeled line
    // Linearize the observation matrix about current pose
    cam.position.x = kfState(0,0);
    cam.position.y = kfState(1,0);
    cam.theta = kfState(2,0);
    vector<line_segment_all_frames> modelLines =
        get_view_lines(cam, Size(640,480), frame);

    /*
    cam.position.x += eps_xy;
    vector<line_segment_all_frames> model_dx = 
        get_view_lines(cam, Size(640,480));

    cam.position.x -= eps_xy;
    cam.position.y += eps_xy;
    vector<line_segment_all_frames> model_dy = 
        get_view_lines(cam, Size(640,480));
    */
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

    for (int i = 0; i <= modelLines.size(); i += 1) {
        const line_segment_all_frames &modelLine = modelLines[i];
        int idx = modelLine.index;
    }

    // Calculate correspondence index k by choosing the closest
    //  fit, given our current pose
    int k = -1;
    double similarity = 0;
    for (int j = 0; j < model.size(); j += 1) {
        const line_segment_all_frames &modelLine = model[j];
        Vec4d modelCamLine =
            pointsToLine(modelLine.camWorld.pt1, modelLine.camWorld.pt2);
        double r2 = pointLineDistance(Vec2d(0,0), modelCamLine);
        double alpha2 = pointLineHeading(Vec2d(0,0), modelCamLine);
        double newSimilarity = exp((r2-r)*(r2-r)/10 + 
                                   (alpha2-alpha)*(alpha2-alpha)/0.1);
        if (newSimilarity > similarity)
            k = modelLine.index;
    }

    // Export lines
    fprintf(outfile, "z{end+1} = [");
    for (int i = 0; i < lines.size(); i += 1) {
        Vec4d &line = lines[i];
        fprintf(outfile, "%f %f %f %f;", line[0], line[1], line[2], line[3]);
    }
    fprintf(outfile, "]';\n");

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
    kfState = kf.predict(kfControl);

    // Publish pose estimate
    bb_msgs::Pose poseMsg;
    poseMsg.x = kfState.at<double>(0);
    poseMsg.y = kfState.at<double>(1);
    poseMsg.theta = kfState.at<double>(2);
    posePub.publish(msg);
}

void imageCallback(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
        ekf_correct(cv_ptr->image);

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
    ros::Subscriber odometrySub = 
        nh.subscribe(odomTopic, 200, odometryCallback);
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

    // Initialize the transition matrix (A), measurement matrix (C)
    //  and control matrix (B)
    kf.transitionMatrix = Mat::eye(3, 3, CV_64F);
    kf.measurementMatrix = Mat::eye(3, 3, CV_64F);
    kf.controlMatrix = Mat::zeros(3, 2, CV_64F);

    // Initialize KF state to bottom court service line corner
    kfState(0,0) = 550;
    kfState(1,0) = -50;
    kfState(2,0) = CV_PI/2;
    kf.statePre = kf.statePost = kfState;
    // Initialize the error (covariance) matrix to 50 cm / 0.1 radians
    kf.errorCovPost = Mat::eye(3, 3, CV_64F) * 50;
    kf.errorCovPost.at<double>(2,2) = 0.1;

    outfile = fopen("export.m", "w");
    
    // Spin loop: service ROS callbacks
    ros::spin();

    fclose(outfile);
}
