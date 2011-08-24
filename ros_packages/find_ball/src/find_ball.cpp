/*
 * findballmouse.cpp
 *
 *  Created on: Sep 09, 2011
 *      Author: ankush
 */

#include "opencv2/highgui/highgui.hpp"
#include <opencv2/objdetect/objdetect.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <time.h>
#include "camera.h"
#include <ros/ros.h>
#include "bb_msgs/BallPosition.h"
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/image_encodings.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>

ros::Publisher ball_pub;

using namespace cv;
using namespace std;

/** Color thershold values.
    Note the channels are NOT RGB.*/
int low_r = 18, low_g = 58, low_b = 11;
int high_r = 100, high_g = 100, high_b = 17;

const double pi = 3.141592654;
Mat img;
int ffillMode = 1;
int loDiff = 35, upDiff = 35;
int connectivity = 4;
int isColor = true;
bool useMask = true;
int newMaskVal = 255;
Point2d horizonPt;
Size actualFrameSize;
RNG rng;
camera cam;


struct bgr
{
  /** Structure for accessing individual pixels from 
      a 3 channel image (Mat) */
  unsigned char b,g,r;
  bgr(unsigned char b0, unsigned char g0, unsigned char r0) {
    r = r0;
    b = b0;
    g = g0;
  }

  bgr()
  {}
  
  bool operator==(const bgr &other)
  {
    return (other.r == r && other.b == b && other.g == g);
  }
};

struct ballContour
{
  /**Structure for representing the ball candidates.
   @param:
    1. contour : the contour of the candidate
    2. pixelPosition: the position of the center of the candidate in the image
    3. sizeMeasure : place holder for some measure of size of the ball
                     currently being used for storing the number of pixels
                     which fall in the color range.
  */
  vector <Point> contour;
  Point pixelPosition;
  double sizeMeasure;
  
  ballContour(double size0) {
    sizeMeasure = size0;
  }
  ballContour() {}
  void operator= (const ballContour & other) {
    contour = other.contour;
    pixelPosition = other.pixelPosition;
    sizeMeasure = other.sizeMeasure;
  }
};

Mat getContourPixels(vector <Point> contour, Mat &src) {
  /**Returns the pixels in the actual image which fall insdie 
     the contour.
   * @param: contour - vector of points which form a contour
   * 	     src     - source Mat from which the pixels need to be extracted.
   */
  Rect R = boundingRect(contour);
  Mat outMat(Size(R.width, R.height), src.type());
  outMat = src(Range(R.y, R.y + R.height),
	       Range(R.x, R.x + R.width));
  return outMat;
}

/**Returns a vector of contours (which are vectors of points) of the SMALL, CIRCULAR blobs
 * found in input.*/
vector <ballContour> doContours(Mat & input)
{
  /** Returns a vector of ball candidates' contours
      found in the given input image.
      @param
          input : the source image
      @return
          vector of candidate contours
  */
  vector<vector<Point> > contours;
  vector<Vec4i> hierarchy;
  vector<ballContour> returnCandidates;
  findContours(input, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_NONE);
  if (!contours.empty() && !hierarchy.empty()) {
    int idx = 0;
    for( ;idx >= 0; idx = hierarchy[idx][0])
      {
	double contArea = contourArea(contours[idx]);
	if (contArea > 2) {
	  // parameters for ellipse fit onto the contour
	  RotatedRect contourEllipse;
	  float majorAxis = 0.0, minorAxis = 0.0, area = 0.0,
	    delta = 1, r = 3;
	  try {
	    //fit an ellipse to the contour
	    contourEllipse = fitEllipse(contours[idx]); 
	    majorAxis = contourEllipse.size.height;
	    minorAxis = contourEllipse.size.width;
	    area = (majorAxis * minorAxis * pi) / 4.0;
	    
	    /*filter based on shape:
	     * 1. calculate delta = |contour_area - ellipse_area|/contour_area
	     * 2. r = Major_axis/Minor_axis
	     *
	     *	for circular ball, we expect: delta -->0(+) && r --> 1(+)
	     */
	    delta = (abs(contArea - area))/contArea;
	    r = majorAxis/minorAxis;
	  }
	  catch (...) {continue;}
	  
	  if (r < 2 && delta < 0.6) {
	    ballContour candidate;
	    candidate.contour = contours[idx];
	    candidate.pixelPosition = contourEllipse.center;
	    returnCandidates.push_back(candidate);
	  }
	}
      }
  }
  return returnCandidates;
}

bool withinBounds(int n, int m, int nMax, int mMax) {
  /**Returns TRUE iff 0<=n(+-1)<nMAX && 0<= m(+-)1<mMax.*/
  return ((n-1 >= 0)
	  && (n+1 < nMax)
	  && (m-1 >= 0)
	  && (m+1 < mMax));
}

Mat floodFillPostprocess( Mat& img) {
  /** Finds connected components in the input image img.
   The similarity is based on color and intensity of neighbouring pixels.
   Filters the connected components based on size and color (here color bounds are loose).
  @param:
      1. img : The input image
  @return:
      2. maskOut: the mask (single channel, binary image) representing 
                  the connected components. The connected compoenets are
                  filtered on the size. "Appropriate sized" blobs are kept,
		  others discarded.*/
  Mat maskOut( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0) );
  Mat mask( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0) );
  Mat maskLocal( img.rows+2, img.cols+2, CV_8UC1, Scalar::all(0));
  Scalar newVal( 200, 150, 100);
  Scalar lo = Scalar(loDiff, loDiff, loDiff),
    up = Scalar(upDiff, upDiff, upDiff);
  int flags = connectivity + (newMaskVal << 8) + CV_FLOODFILL_FIXED_RANGE;
  for( int y = 0; y < img.rows; y++ )
    {
      for( int x = 0; x < img.cols; x++ )
        { 
	  if (withinBounds(x, y, img.cols, img.rows)) {
	    if(mask.at<uchar>(y+1, x+1) == 0 && mask.at<uchar>(y-1, x-1) == 0) {
	      maskLocal = Mat::zeros(mask.size(), mask.type());
	      int area;
	      //Scalar newVal( rng.uniform(0,255), rng.uniform(0, 255), rng.uniform(0, 255));
	      area = floodFill(img, maskLocal, Point(x,y), newVal, 0, lo, up, flags);
	      bitwise_or(mask, maskLocal, mask);
	      
	      if(area>0 && area < 10000) { //<<<<<<<<<<<<<<<<< was 600
		bitwise_or(maskOut, maskLocal, maskOut);
	      }
	    } else {continue;}
	  }
        }
    }
  return maskOut;
}

void publishMessage(Point ballPosition) {
  double radians_per_px = 0.0016;
  // Distance to target
  double theta = (ballPosition.y - cam.intrinsics.cy)
    * radians_per_px - cam.tilt;
  double y = ((double)cam.position.z) / tan(theta);
  // Angle/X offset to target
  double phi = (ballPosition.x - cam.intrinsics.cx)
    * radians_per_px + cam.pan;
  double x = y * tan(phi);
  double d = sqrt(x*x + y*y);

  bb_msgs::BallPosition msg;
  msg.d = (d/100.0);
  msg.theta = phi;
  ball_pub.publish(msg);
}

void processNewFrame(Mat &frame) {

  //imshow("original", frame);
  Mat frameROI = frame(Range(horizonPt.y, frame.rows), Range::all());

  //imshow("original", frame);
  /** Split the cut-off frame into 2. Use different
      thresholds for posterization and then put them
      together.*/

  Mat frameCopy, frameCopy2;
  frame.copyTo(frameCopy);
  frame.copyTo(frameCopy2);

  Mat top;
  frame.adjustROI(-horizonPt.y,((int) ((horizonPt.y - frame.rows)/2.0)), 0, 0);
  pyrMeanShiftFiltering(frame, top, 6, 30, 3);  
  //imshow("Top", top);

  Mat bottom;
  frameCopy.adjustROI(((int) (-(horizonPt.y + frameCopy.rows)/2.0)), 0, 0, 0);
  pyrMeanShiftFiltering(frameCopy, bottom, 5, 100, 4);
  //imshow("Bottom", bottom);

  Size combinedSize(top.cols, top.rows + bottom.rows);
  Mat dst(combinedSize, top.type());
  Point pt(0, 0); 
  Mat roi = Mat(dst, Rect(pt, top.size())); 
  top.copyTo(roi);

  Point pt2(0, top.rows);
  Mat roi2 = Mat(dst, Rect(pt2, bottom.size()));
  bottom.copyTo(roi2);
  //imshow("combined", dst);

  Mat mask = floodFillPostprocess(dst);
  //imshow("flood", dst);
  //imshow("flood fill filtered", mask);
  
  ballContour prevBall(-1);
  vector <ballContour> candidates  = doContours(mask);
  
  ballContour maxColorConformityContour((double) -1);
  Mat ballFound = Mat::zeros(dst.size(), dst.type());
  for (unsigned int i  = 0; i < candidates.size(); i++) {
    Mat blobPix;
    try {
      // get the blob image pixels
      blobPix = getContourPixels(candidates[i].contour, frameROI);

      //Do color filtering
      Mat contourHSV(blobPix.size(), blobPix.type());
      cvtColor(blobPix, contourHSV, CV_BGR2HSV);

      Mat colorRangeMask(blobPix.size(), CV_8UC1);
      inRange(contourHSV, Scalar((low_b/100.0)*255, (low_g/100.0)*255, (low_r/100.0)*255, 0),
		Scalar((high_b/100.0)*255, (high_g/100.0)*255, (high_r/100.0)*255, 0), colorRangeMask);

      //Find the candidate with max number of pixels in range.
      int pixCount = countNonZero(colorRangeMask);
      candidates[i].sizeMeasure = pixCount;
      if (pixCount > 0 && pixCount > maxColorConformityContour.sizeMeasure) {
	maxColorConformityContour = candidates[i];
      }
    } catch(...) {
      continue;
    }
  }
  if (maxColorConformityContour.sizeMeasure != -1) {
    Point updatedBallPosition = Point(maxColorConformityContour.pixelPosition.x,
				      maxColorConformityContour.pixelPosition.y
				      + (actualFrameSize.height - ballFound.size().height));
    publishMessage(updatedBallPosition);
    ellipse( ballFound, maxColorConformityContour.pixelPosition, Size(5,5),
      0, 0, 360, Scalar(0,0,255), CV_FILLED, 8, 0);
  } else {
    ROS_INFO("no ball found");
  }
  imshow("ball detected", ballFound+dst);
  waitKey(3);
}

void received_frame(const sensor_msgs::ImageConstPtr &msgFrame) {
  /** The callback function called whenever
      a frame is published by gscam.*/
  //ROS_INFO("frame received!");
  cv_bridge::CvImagePtr cvPtr;
  try {
    cvPtr = cv_bridge::toCvCopy(msgFrame, sensor_msgs::image_encodings::BGR8);
  } catch (cv_bridge::Exception& e) {
    ROS_ERROR("cv_bridge exception: %s", e.what());
    return;
  }
  actualFrameSize = cvPtr->image.size();
  processNewFrame(cvPtr->image);
}

int main( int argc, char** argv ) {
  ros::init(argc, argv, "ball_finder");
  ros::NodeHandle n;
  ros::NodeHandle nPrivate("~");
  image_transport::ImageTransport it(n);
  image_transport::Subscriber imageSub;
  string imageTopic;
  if (!nPrivate.hasParam("image_transport"))
    nPrivate.setParam("image_transport", "compressed");

  nPrivate.param<string>("image", imageTopic, "gscam/image_raw");
  imageSub = it.subscribe(imageTopic, 5, received_frame);
  ball_pub = n.advertise<bb_msgs::BallPosition>("ball", 1000);
  
  //find the horizon pt
  Point3d camWorldPt = get_camera_world_coordinates(
			   Point3d(3500, 0, -cam.position.z),
			   cam.position, cam.theta, cam.pan,
			   cam.tilt);
  //^^^^^^^^^^^^^^^^^^^^^^^^^ 3500 is the farthest depth we want to see (in cm).
  //------------------------- change it as per convenience----------------//

  horizonPt = cam_world_position_to_imageXY(camWorldPt, cam);
  ros::spin();
  return 0;
}
