// Based on http://www.linuxconfig.org/introduction-to-computer-vision-with-opencv-on-linux

#include "highgui.h"
int main( int argc, char** argv ) {
  cvNamedWindow( "Example2", CV_WINDOW_AUTOSIZE );

  CvCapture* capture = cvCreateCameraCapture(0) ;
  IplImage* frame;

  unsigned short i = 0;
  char outFileName[12];

  while(1) {
    frame = cvQueryFrame( capture );
    if( !frame ) break;

    cvShowImage( "Example2", frame );
    char c = cvWaitKey(33);
    if( c == 27 ) break;
	else if (c == 'p') {
		sprintf(outFileName, "pic%d.jpg", i++);
		if(cvSaveImage(outFileName, frame))
			printf("Saved: %s\n", outFileName);
		else
			printf("Could not save: %s\n", outFileName);
	}


  }
  cvReleaseCapture( &capture );
  cvDestroyWindow( "Example2" );
}
