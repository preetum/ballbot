#include "highgui.h"
#include "cv.h"

int main( int argc, char** argv ) {
  // cvLoadImage determines an image type and creates datastructure with appropriate size
  IplImage* img = cvLoadImage( argv[1]);

  // create a window. Window name is determined by a supplied argument
  cvNamedWindow( argv[1], CV_WINDOW_AUTOSIZE );

  cvSmooth( img, img, CV_GAUSSIAN, 9, 9 );

  // Display an image inside and window. Window name is determined by a supplied argument
  cvShowImage( argv[1], img );
  // wait indefinitely for keystroke
  cvWaitKey(0);

  // release pointer to an object
  cvReleaseImage( &img );
  // Destroy a window
  cvDestroyWindow( argv[1] );
}
