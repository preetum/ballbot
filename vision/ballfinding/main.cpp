#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <sys/timeb.h>
#include <iostream>
#include <unistd.h>

#include <webcam.h>

#include <cvblob.h>
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace cv;
using namespace cvb;

#define RADIANS_PER_PX (0.0016)

int frame_width = 320,
  frame_height = 240;
double camera_height = 33.5, // in cm
  camera_angle = 0.349; // in radians

bool display = false,
  verbose = false;

// Webcam control
char *webcam_name = "/dev/video0";
CHandle webcam_device;

void print_time(string label)
{
  if (verbose) {
    static struct timeb prev = {0,0};
    struct timeb cur;
    double diff = 0;
    ftime(&cur);
    if (prev.time) {
      diff  =  cur.time    - prev.time;
      diff += (cur.millitm - prev.millitm)/1000.0;
    }
    fprintf(stderr, "%30s  start = %d.%-3hu (+%5.3f)\n",
	    label.c_str(), (int)cur.time, cur.millitm, diff);
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

void process(Mat &img, Mat& out)
{
  print_time("get frame");
  // Convert to HSV
  Size size = img.size();
  Mat hsv(size, CV_8UC3);
  cvtColor(img, hsv, CV_BGR2HSV);

  if (display) imshow("hsv", hsv);

  // Filter by hue
  Mat mask(size, CV_8UC1);
  inRange(hsv, Scalar(0.11*255, 0.3*255, 0.20*255, 0),
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
      cout << "Blob #" << it->second->label;
      cout << ": Area=" << it->second->area;
      cout << ", Centroid=(" << it->second->centroid.x <<
	", " << it->second->centroid.y << ")" << endl;
    }

    if (largest == NULL || it->second->area > largest->area)
      largest = it->second;
  }

  if (largest != NULL) {
    // Distance to target
    double theta = (double)(largest->centroid.y - frame_height/2)
      * RADIANS_PER_PX,
      y = camera_height / tan(theta + camera_angle);
    // Angle/X offset to target
    double phi = (double)(frame_width/2 - largest->centroid.x)
      * RADIANS_PER_PX,
      x = -y * tan(phi);
    
    printf("Ball at x,y = %.2f, %.2f cm\n", x, y);
  } else {
    printf("No ball found\n");
  }

  out = mask;
}

int main(int argc, char **argv)
{
  bool run = true;
  Mat frame, out;

  // Parse command line args
  int c;
  while ((c = getopt(argc, argv, "c:dv")) != -1) {
    switch (c) {
    case 'c':
      webcam_name = optarg;
      break;
    case 'd':
      display = true;
      break;
    case 'v':
      verbose = true;
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
    process(frame, out);
    if (display) {
      imshow("img", frame);
      imshow("out", out);
    }
    cvWaitKey(0);
  }

  // Use webcam
  else {
    VideoCapture cam(0);
    if (!cam.isOpened()) {
      fprintf(stderr, "could not open camera\n");
      return -1;
    }
    cam.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, 240);
    printf("initialized camera\n");
    cam >> frame;
    printf("frame size: %dx%d channels: %d\n", frame.cols, frame.rows,
	   frame.channels());

    // Initialize webcam position
    c_init();
    printf("opening %s using libwebcam\n", webcam_name);
    webcam_device = c_open_device(webcam_name);
    CControlValue control_value = {CC_TYPE_BYTE, {1}};
    c_set_control(webcam_device, CC_PAN_RESET, &control_value);
    sleep(3);
    c_set_control(webcam_device, CC_TILT_RESET, &control_value);
    sleep(1);
    control_value.type = CC_TYPE_WORD;
    // Point webcam at 64*20 degrees down
    control_value.value = 1280;
    c_set_control(webcam_device, CC_TILT_RELATIVE, &control_value);

    while (run) {
      cam >> frame;
      process(frame, out);
      if (display) {
        imshow("img", frame);
        imshow("out", out);
      }
      if ((char)cvWaitKey(10) == 'q')
        run = false;
    }
  }
}
