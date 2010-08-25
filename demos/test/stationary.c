/*
http://www.linuxconfig.org/introduction-to-computer-vision-with-opencv-on-linux
http://wiki.elphel.com/index.php?title=OpenCV_Tennis_balls_recognizing_tutorial
http://cgi.cse.unsw.edu.au/~cs4411/wiki/index.php?title=OpenCV_Guide#Detecting_circles
*/

#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include "cv.h"
#include "highgui.h"

#define min(a,b) ((a < b) ? a : b)

#define THRESHOLD 230

int main( int argc, char** argv ) {
	FILE* out = NULL;
	bool video = 0;

	// parameters: 
	//  -d = output stuff to serial (drive)
	//  -v = output video
	if (argc > 1 && argv[1][0] == '-') {
		char *s = argv[1] + 1;
		while (*s != '\0')
			switch (*s) {
			case 'd':
				out = popen("python ../ballbot_test.py", "w");
				break;
			case 'v':
				video = 1;
				break;
			}
	}

	if (video)
		cvNamedWindow( "Example2", CV_WINDOW_AUTOSIZE );

    CvCapture* capture = cvCreateCameraCapture(0) ;
    IplImage* frame;

    frame = cvQueryFrame( capture );
    if( !frame ) return 0;


	CvScalar lowerBound = cvScalar(0.11*256, 0.60*256, 0.20*256, 0),
		upperBound = cvScalar(0.14*256, 1.00*256, 1.00*256, 0);
	IplConvKernel *se21 = cvCreateStructuringElementEx(21, 21, 10, 10,
													   CV_SHAPE_RECT, NULL);
	IplConvKernel *se11 = cvCreateStructuringElementEx(11, 11, 5,  5,  
													   CV_SHAPE_RECT, NULL);

    while(frame) {

		// convert to HSV
		CvSize size = cvGetSize(frame);
		IplImage *hsv = cvCreateImage(size, IPL_DEPTH_8U, 3);
		cvCvtColor(frame, hsv, CV_BGR2HSV);  

		// generate a mask
		CvMat *mask = cvCreateMat(size.height, size.width, CV_8UC1);
		cvInRangeS(hsv, lowerBound, upperBound, mask);
		cvReleaseImage(&hsv);

		// clean up noise on the mask
		cvClose(mask, mask, se21);
		cvOpen(mask, mask, se11);

		// Canny edge detection and Hough transform
		//  Copy mask into a grayscale image
		IplImage *hough_in = cvCreateImage(size, 8, 1);
		cvCopy(mask, hough_in, NULL);
        cvSmooth(hough_in, hough_in, CV_GAUSSIAN, 15, 15, 0, 0);
		//  Run the Hough function
		CvMemStorage *storage = cvCreateMemStorage(0);
		CvSeq *circles = cvHoughCircles(hough_in, storage,
										CV_HOUGH_GRADIENT, 4, 
										size.height/10, 100, 40, 0, 0);
		cvReleaseMemStorage(&storage);

		for (int i = 0; i < circles->total; i++) {
			float *p = (float*)cvGetSeqElem(circles, i);
			printf("%d:(%d, %d) ", i, cvRound(p[0]), cvRound(p[1]));
		}
		putchar('\n');


		if (video)
			cvShowImage( "Example2", frame );

        char c = cvWaitKey(33);
        if( c == 27 ) break;

        frame = cvQueryFrame( capture );
    }

	cvReleaseStructuringElement(&se21);
	cvReleaseStructuringElement(&se11);

    cvReleaseCapture( &capture );

	if (video)
		cvDestroyWindow( "Example2" );
}
