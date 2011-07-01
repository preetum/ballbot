#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <sys/timeb.h>
#include <unistd.h>

#include <cvblob.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#include <ros/ros.h>
#include "navigation/goal_msg.h"
#include "logitech_pantilt/SetAngle.h"
#include "logitech_pantilt/Orientation2.h"

using namespace cv;
using namespace cvb;

#define PI 3.14159265358979
#define RAD2DEG(x)  ((x) * 180.0 / PI)
#define DEG2RAD(x)  ((x) * PI / 180.0)

int frame_width = 320,
  frame_height = 240;
double camera_height = 33.5, // in cm
  camera_pan_angle = 0.0,    // in radians
  camera_tilt_angle = 0.0,
  camera_mount_pan_angle = 0.0723,
  camera_mount_tilt_angle = 0, //0.0262 = 1.5 degrees, this is a fudge factor
                               // determined from calibration measurements
  radians_per_px = 0.0032;

bool display = false,
  verbose = false,
  single = false;

void print_time(const char* label)
{
  if (verbose) {
    static struct timeb prev = {0,0,0,0};
    struct timeb cur;
    double diff = 0;
    ftime(&cur);
    if (prev.time) {
      diff  =  cur.time    - prev.time;
      diff += (cur.millitm - prev.millitm)/1000.0;
    }
    ROS_INFO("%30s  start = %d.%-3hu (+%5.3f)\n",
	    label, (int)cur.time, cur.millitm, diff);
    prev = cur;
  }
}

void open(InputArray src, OutputArray dst, InputArray element)
{
  erode (src, dst, element);
  dilate(src, dst, element);
}

void close(InputArray src, OutputArray dst, InputArray element)
{
  dilate(src, dst, element);
  erode (src, dst, element);
}

/* Updates camera angle */
void pantilt_callback(const logitech_pantilt::Orientation2::ConstPtr&
		      orientation)
{
  camera_pan_angle = DEG2RAD(orientation->pan) + camera_mount_pan_angle;
  camera_tilt_angle = DEG2RAD(orientation->tilt) + camera_mount_tilt_angle;

  ROS_INFO("pan: %.2f,\ttilt %.2f deg", 
	   RAD2DEG(camera_pan_angle), RAD2DEG(camera_tilt_angle));
}

CvBlob* find_ball(Mat &img, Mat& out)
{
  print_time("get frame");
  // Convert to HSV
  Size size = img.size();
  Mat hsv(size, CV_8UC3);
  cvtColor(img, hsv, CV_BGR2HSV);

  if (display) imshow("hsv", hsv);

  // Filter by hue
  Mat mask(size, CV_8UC1);
  inRange(hsv, Scalar(0.11*255, 0.58*255, 0.20*255, 0),
	  Scalar(0.15*255, 1.00*255, 1.00*255, 0), mask);
  print_time("convert to hsv & threshold");

  if (display) {
    IplImage maskCopy = mask;
    cvShowImage("mask", &maskCopy);
  }

  // Clean up noise
  static Mat closeElement = getStructuringElement(MORPH_RECT, Size(21, 21));
  static Mat openElement = getStructuringElement(MORPH_RECT, Size(3, 3));
  open(mask, mask, openElement);
  close(mask, mask, closeElement);
  print_time("morphological ops");

  // Find blobs
  CvBlobs blobs;
  IplImage maskCopy = mask,
    imgCopy = img;
  IplImage *labelImg = cvCreateImage(size, IPL_DEPTH_LABEL, 1);
  cvLabel(&maskCopy, labelImg, blobs);
  cvRenderBlobs(labelImg, blobs, &imgCopy, &imgCopy);
  cvReleaseImage(&labelImg);
  print_time("find blobs");


  CvBlob *largest = NULL;
  // Print blobs
  // Find largest blob
  for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it) {
    if (verbose) {
      ROS_INFO("Blob #%d: Area=%d, Centroid=(%f, %f)",
	       it->second->label,
	       it->second->area,
	       it->second->centroid.x,
	       it->second->centroid.y);
    }

    if (largest == NULL || it->second->area > largest->area)
      largest = it->second;
  }

  // Output the final image as well as the largest blob found (may be NULL)
  out = mask;
  return largest;
}

int main(int argc, char **argv)
{
  int cam_index = 0;
  bool run = true;
  Mat frame, out;
  int hz = 0;

  ros::init(argc, argv, "tracker");
  ros::NodeHandle n;
  ros::Publisher goal_pub = n.advertise<navigation::goal_msg>("goal", 100);

  // Parse command line args
  int c;
  while ((c = getopt(argc, argv, "c:r:dv1")) != -1) {
    switch (c) {
    case 'c':  // specify camera path
      cam_index = atoi(optarg);
      break;
    case 'r':  // refresh rate
      hz = atoi(optarg);
      break;
    case 'd':  // turn on GUI display
      display = true;
      break;
    case 'v':  // turn on verbose output
      verbose = true;
      break;
    case '1':  // output only one goal message
      single = true;
      break;
    default:
      return -1;
    }
  }

  // Create windows
  if (display) {
    cvNamedWindow("img", 0);
    cvMoveWindow("img", 50, 50);
    cvNamedWindow("hsv", 0);
    cvMoveWindow("hsv", 400, 50);
    cvNamedWindow("mask", 0);
    cvMoveWindow("mask", 50, 350);
    cvNamedWindow("out", 0);
    cvMoveWindow("out", 400, 350);
  }

  // Arguments remaining: load image file
  if (optind < argc) {
    frame = imread(argv[optind]);
    if (frame.data == NULL) {
      printf("could not open file: %s\n", argv[optind]);
      return -1;
    }
    resize(frame, frame, Size(640, 480));
    find_ball(frame, out);
    if (display) {
      imshow("img", frame);
      imshow("out", out);
    }
    cvWaitKey(0);
  }

  // Use webcam
  else {
    VideoCapture cam(cam_index);
    if (!cam.isOpened()) {
      ROS_ERROR("could not open camera %d", cam_index);
      return -1;
    }
    cam.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    printf("initialized camera\n");
    cam >> frame;
    printf("frame size: %dx%d channels: %d\n", frame.cols, frame.rows,
	   frame.channels());

    // Initialize webcam position
    ros::ServiceClient pan_client = 
      n.serviceClient<logitech_pantilt::SetAngle>("set_pan"),
      tilt_client = n.serviceClient<logitech_pantilt::SetAngle>("set_tilt");
    logitech_pantilt::SetAngle srv;
    srv.request.angle = 0;
    if (pan_client.call(srv)) {
      srv.response.move_time.sleep();
    }
    srv.request.angle = -20;
    if (tilt_client.call(srv)) {
      srv.response.move_time.sleep();
    }

    // Subscribe to webcam position updates
    ros::Subscriber sub = n.subscribe("orientation", 100, pantilt_callback);

    ros::Rate rate(hz);
    while (run && ros::ok()) {

      // Grab new frame
      cam >> frame;

      // Find the largest blob
      CvBlob *blob = find_ball(frame, out);

      if (blob != NULL) {
	// Distance to target
	double theta = (double)(blob->centroid.y - frame_height/2)
	  * radians_per_px - camera_tilt_angle,
	  y = camera_height / tan(theta);
	// Angle/X offset to target
	double phi = (double)(blob->centroid.x - frame_width/2)
	  * radians_per_px + camera_pan_angle,
	  x = y * tan(phi);
	double d = sqrt(x*x + y*y);
	
	ROS_INFO("Ball at r,theta = %.2f cm, %.2f deg", d, RAD2DEG(phi));

	// Publish goal message
	navigation::goal_msg msg;
	msg.d = d;
	msg.th = RAD2DEG(phi);
	msg.opt = 3;

	goal_pub.publish(msg);
	if (hz > 0)
	  rate.sleep();
      } else {
	ROS_INFO("No ball found");
      }

      // Process ROS callbacks 'n stuff
      ros::spinOnce();

      if (display) {
        imshow("img", frame);
        imshow("out", out);
      }

      if ((char)cvWaitKey(10) == 'q')
        run = false;

      if (single && blob != NULL)
	run = false;
    }
  }
}
