#include <stdio.h>
#include <stdbool.h>
#include <math.h>
#include <sys/timeb.h>
#include <iostream>

#include <cvblob.h>
#include <cv.h>
#include <highgui.h>

using namespace std;
using namespace cv;
using namespace cvb;

#define RADIANS_PER_PX (0.0016)

bool display = true;

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

bool getPixelCenter(IplImage *frame, IplImage *img,
                    uint8_t threshold, int *x, int *y)
{
  int height, width, channels, pxPerRow;
  uint8_t *data;
  uint32_t xsum = 0, ysum = 0, count = 0;

  height = frame->height;
  width = frame->width;
  pxPerRow = frame->widthStep;
  channels = frame->nChannels;
  data = (uint8_t*)frame->imageData;

  for (int i = 0; i < height; i += 1)         // y-coordinate
    for (int j = 0; j < width; j += 1) {    // x-coordinate
      uint8_t* pixel = (uint8_t*)(data + i*pxPerRow + j*channels);

      // Single channel
      if (*pixel > threshold) {
        // highlight in blue
        uint8_t* imgPx = (uint8_t*)(img->imageData + img->widthStep*i + j*img->nChannels);
        imgPx[0] = 0xFF;
        imgPx[1] = imgPx[2] = 0;
                                
        xsum += j;
        ysum += i;
        count += 1;
      }
    }
        
  if (count > 0) {
    *x = xsum/count;
    *y = ysum/count;
    return true;
  }

  return false;
}

void process(Mat &img, Mat& out)
{
  // Convert to HSV
  Size size = img.size();
  Mat hsv(size, CV_8UC3);
  cvtColor(img, hsv, CV_BGR2HSV);

  if (display) imshow("hsv", hsv);

  // Filter by hue
  Mat mask(size, CV_8UC1);
  inRange(hsv, Scalar(0.11*255, 0.3*255, 0.20*255, 0),
	  Scalar(0.15*255, 1.00*255, 1.00*255, 0), mask);

  if (display) imshow("mask", mask);

  // Clean up noise
  static Mat closeElement = getStructuringElement(MORPH_RECT, Size(21, 21));
  static Mat openElement = getStructuringElement(MORPH_RECT, Size(3, 3));
  open(mask, mask, openElement);
  close(mask, mask, closeElement);

  out = mask;

  // Find blobs
  CvBlobs blobs;
  IplImage maskCopy = mask,
    imgCopy = img;
  IplImage *labelImg = cvCreateImage(size, IPL_DEPTH_LABEL, 1);
  cvLabel(&maskCopy, labelImg, blobs);
  cvRenderBlobs(labelImg, blobs, &imgCopy, &imgCopy);
  for (CvBlobs::const_iterator it=blobs.begin(); it!=blobs.end(); ++it) {
    cout << "Blob #" << it->second->label;
    cout << ": Area=" << it->second->area;
    cout << ", Centroid=(" << it->second->centroid.x <<
      ", " << it->second->centroid.y << ")" << endl;
  }
  cvReleaseImage(&labelImg);

  /*
  // Apply Gaussian blur
  vector<Vec3f> circles;
  GaussianBlur(mask, mask, Size(5, 5), 2);

  // Detect circles using Hough transform
  HoughCircles(mask, circles, CV_HOUGH_GRADIENT, 4, size.height/10, 100, 40, 0, 0);
  for (uint i = 0; i < circles.size(); i += 1) {
    Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
    int radius = cvRound(circles[i][2]);
    // draw the circle center
    circle( img, center, 3, Scalar(0,255,0), -1, 8, 0 );
    // draw the circle outline
    circle( img, center, radius, Scalar(0,0,255), 3, 8, 0 );
  }
  */
}

void test_live(CvCapture *cam)
{

}

int main(int argc, char **argv)
{
  bool run = true;
  Mat frame, out;

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

  if (argc > 1) {
    // Use image file
    frame = imread(argv[1]);
    resize(frame, frame, Size(640, 480));
    process(frame, out);
    if (display) {
      imshow("img", frame);
      imshow("out", out);
    }
    cvWaitKey(0);
  } else {
    VideoCapture cam(0);
    if (!cam.isOpened()) {
      printf("could not open camera\n");
      return -1;
    }

    while (run) {
      cam >> frame;
      process(frame, out);
      if (display) {
        imshow("img", frame);
        imshow("out", out);
      }
      if ((char)cvWaitKey(33) == 'q')
        run = false;
    }
  }
}
