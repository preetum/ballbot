#include <opencv/cv.h>
#include <opencv/highgui.h>
#include <opencv/cxcore.h>
#include <stdio.h>

#include <float.h>

#include "Blob.h"
#include "BlobResult.h"



// #define HALF_SIZE_CAPTURE
#define CV_CAPTURE_SOURCE 0


IplImage *source_frame = 0, *inputImage = 0, *histAdjustedImage = 0, *bright_cont_image = 0, *outputImage = 0, *hist_image = 0, *rainbowImage = 0;

int _brightness = 100, _contrast = 100, blob_extraction_threshold = 200, min_blob_size = 100, max_blob_size = 5000, normalization_sum = 1000;

uchar lut[256];
CvMat* lut_mat;



// Histogram initialization:
CvHistogram *my_hist;
int hist_size = 64;
int hist_size_array[] = {64};
float range_0[]={0,256};
float* ranges[] = { range_0 };

// Font initialization:
CvFont font;
double hScale=0.5;
double vScale=0.5;
int	lineWidth=2;		
char str[100];







// Takes h in degress, s and v are 0 to 1
CvScalar cv_hsv2rgb(float h, float s, float v) {

	float rgb[3];
	int MAX_COLOR_VALUE = 255;

	float f,p,q,t;
	int i;

	if(s <= FLT_EPSILON) {
		for(i=0; i<3; i++)
			rgb[i] = v;
	} else {
		while(h<0.0) h+=360.0;
		while(h>=360.0) h-=360.0;
		h/=60.0;
		i = (int) floorf(h);
		f = h-(float)i;
		p = v*(1.0-s);
		q = v*(1.0-(s*f));
		t = v*(1.0-(s*(1.0-f)));
		switch(i) {
			case 0: rgb[0] = v; rgb[1] = t; rgb[2] = p; break;
			case 1: rgb[0] = q; rgb[1] = v; rgb[2] = p; break;
			case 2: rgb[0] = p; rgb[1] = v; rgb[2] = t; break;
			case 3: rgb[0] = p; rgb[1] = q; rgb[2] = v; break;
			case 4: rgb[0] = t; rgb[1] = p; rgb[2] = v; break;
			case 5: rgb[0] = v; rgb[1] = p; rgb[2] = q; break;
		}
	}

	for(i=0; i<3; i++)
		rgb[i] *= MAX_COLOR_VALUE;

	return CV_RGB(rgb[0], rgb[1], rgb[2]);
}











// A Simple Camera Capture Framework
int main() {

	CvCapture* capture = cvCaptureFromCAM( 0 );
	if( !capture ) {
		fprintf( stderr, "ERROR: capture is NULL \n" );
		return -1;
	}

	#ifdef HALF_SIZE_CAPTURE
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_WIDTH, 352/2);
	cvSetCaptureProperty(capture, CV_CAP_PROP_FRAME_HEIGHT, 288/2);
	#endif

	// Create a window in which the captured images will be presented
	cvNamedWindow( "Source Image Window", CV_WINDOW_AUTOSIZE );
	cvNamedWindow( "Back Projected Image", CV_WINDOW_AUTOSIZE );
	cvNamedWindow( "Brightness and Contrast Window", CV_WINDOW_AUTOSIZE );
	cvNamedWindow( "Blob Output Window", CV_WINDOW_AUTOSIZE );
	cvNamedWindow( "Histogram Window", 0);

	cvNamedWindow( "Rainbow Window", CV_WINDOW_AUTOSIZE );

	// Capture one frame to get image attributes:
	source_frame = cvQueryFrame( capture );
	if( !source_frame ) {
		fprintf( stderr, "ERROR: frame is null...\n" );
		return -1;
	}

	cvCreateTrackbar("histogram\nnormalization", "Back Projected Image", &normalization_sum, 6000, NULL);
	cvCreateTrackbar("brightness", "Brightness and Contrast Window", &_brightness, 200, NULL);
	cvCreateTrackbar("contrast", "Brightness and Contrast Window", &_contrast, 200, NULL);
	cvCreateTrackbar("threshold", "Blob Output Window", &blob_extraction_threshold, 255, NULL);
	cvCreateTrackbar("min blob size", "Blob Output Window", &min_blob_size, 2000, NULL);
	cvCreateTrackbar("max blob size", "Blob Output Window", &max_blob_size, source_frame->width*source_frame->height/4, NULL);



	inputImage = cvCreateImage(cvGetSize(source_frame), IPL_DEPTH_8U, 1);
	histAdjustedImage = cvCreateImage(cvGetSize(source_frame), IPL_DEPTH_8U, 1);
	outputImage = cvCreateImage(cvGetSize(source_frame), IPL_DEPTH_8U, 3 );
	hist_image = cvCreateImage(cvSize(320,200), 8, 1);

	rainbowImage = cvCreateImage(cvGetSize(source_frame), IPL_DEPTH_8U, 3 );


	// object that will contain blobs of inputImage 
	CBlobResult blobs;
	CBlob my_enumerated_blob;

	cvInitFont(&font, CV_FONT_HERSHEY_SIMPLEX|CV_FONT_ITALIC, hScale, vScale, 0, lineWidth);





	// Some brightness/contrast stuff:
	bright_cont_image = cvCloneImage(inputImage);
	lut_mat = cvCreateMatHeader( 1, 256, CV_8UC1 );
	cvSetData( lut_mat, lut, 0 );




	while( 1 ) {

		// Get one frame
		source_frame = cvQueryFrame( capture );
		if( !source_frame ) {
			fprintf( stderr, "ERROR: frame is null...\n" );
			getchar();
			break;
		}
		cvShowImage( "Source Image Window", source_frame );
		// Do not release the frame!

		cvCvtColor(source_frame, inputImage, CV_RGB2GRAY);

		// Histogram Stuff!
		my_hist = cvCreateHist(1, hist_size_array, CV_HIST_ARRAY, ranges, 1);
		cvCalcHist( &inputImage, my_hist, 0, NULL );
		cvNormalizeHist(my_hist, normalization_sum);

		// NOTE: First argument MUST have an ampersand, or a segmentation fault will result
		cvCalcBackProject(&inputImage, histAdjustedImage, my_hist);



		// Histogram Picture
		int bin_w;
		float max_value = 0;
		cvGetMinMaxHistValue( my_hist, 0, &max_value, 0, 0 );
		cvScale( my_hist->bins, my_hist->bins, ((double)hist_image->height)/max_value, 0 );
		cvSet( hist_image, cvScalarAll(255), 0 );
		bin_w = cvRound((double)hist_image->width/hist_size);

		for(int i = 0; i < hist_size; i++ )
			cvRectangle( hist_image, cvPoint(i*bin_w, hist_image->height), cvPoint((i+1)*bin_w, hist_image->height - cvRound(cvGetReal1D(my_hist->bins,i))), cvScalarAll(0), -1, 8, 0 );
		cvShowImage( "Histogram Window", hist_image );
		cvShowImage("Back Projected Image", histAdjustedImage);






		// Brightness/contrast loop stuff:
		int brightness = _brightness - 100;
		int contrast = _contrast - 100;

		/*
		 * The algorithm is by Werner D. Streidt
		 * (http://visca.com/ffactory/archives/5-99/msg00021.html)
		 */
		if( contrast > 0 ) {
			double delta = 127.*contrast/100;
			double a = 255./(255. - delta*2);
			double b = a*(brightness - delta);
			for(int i = 0; i < 256; i++ )
			{
				int v = cvRound(a*i + b);
				if( v < 0 ) v = 0;
				if( v > 255 ) v = 255;
				lut[i] = (uchar)v;
			}
		}
		else {
			double delta = -128.*contrast/100;
			double a = (256.-delta*2)/255.;
			double b = a*brightness + delta;
			for(int i = 0; i < 256; i++ ) {
				int v = cvRound(a*i + b);
				if( v < 0 )
					v = 0;
				if( v > 255 )
					v = 255;
				lut[i] = (uchar)v;
			}
		}

		cvLUT( inputImage, bright_cont_image, lut_mat );
		cvShowImage( "Brightness and Contrast Window", bright_cont_image);







		// ---------------
		// Blob Manipulation Code begins here:

		// Extract the blobs using a threshold of 100 in the image
		blobs = CBlobResult( bright_cont_image, NULL, blob_extraction_threshold, true );

		// discard the blobs with less area than 5000 pixels
		// ( the criteria to filter can be any class derived from COperadorBlob ) 
		blobs.Filter( blobs, B_INCLUDE, CBlobGetArea(), B_GREATER_OR_EQUAL, min_blob_size);
		blobs.Filter( blobs, B_EXCLUDE, CBlobGetArea(), B_GREATER, max_blob_size);

		// build an output image equal to the input but with 3 channels (to draw the coloured blobs)
		cvMerge( bright_cont_image, bright_cont_image, bright_cont_image, NULL, outputImage );

		// plot the selected blobs in a output image
		for (int i=0; i < blobs.GetNumBlobs(); i++) {
			blobs.GetNthBlob( CBlobGetArea(), i, my_enumerated_blob );
			// Color 5/6 of the color wheel (300 degrees)
			my_enumerated_blob.FillBlob( outputImage, cv_hsv2rgb((float)i/blobs.GetNumBlobs() * 300, 1, 1));
		}
		// END Blob Manipulation Code
		// ---------------


		sprintf(str, "Count: %d", blobs.GetNumBlobs());
		cvPutText(outputImage, str, cvPoint(50, 25), &font, cvScalar(255,0,255));
		
		cvShowImage("Blob Output Window", outputImage);






/*
		// Rainbow manipulation:
		for (int i=0; i < CV_CAP_PROP_FRAME_WIDTH; i++) {
			for (int j=0; j < CV_CAP_PROP_FRAME_HEIGHT; j++) {
// This line is not figure out yet...
//				pixel_color_set = ((uchar*)(rainbowImage->imageData + rainbowImage->widthStep * j))[i * 3]

				((uchar*)(rainbowImage->imageData + rainbowImage->widthStep * j))[i * 3] = 30;
				((uchar*)(rainbowImage->imageData + rainbowImage->widthStep * j))[i * 3 + 1] = 30;
				((uchar*)(rainbowImage->imageData + rainbowImage->widthStep * j))[i * 3 + 2] = 30;
			}
		}
		cvShowImage("Rainbow Window", rainbowImage);
*/







		//If ESC key pressed, Key=0x10001B under OpenCV 0.9.7(linux version),
		//remove higher bits using AND operator
		if( (cvWaitKey(10) & 255) == 27 ) break;

	}

	cvReleaseImage(&inputImage);
	cvReleaseImage(&histAdjustedImage);
	cvReleaseImage(&hist_image);
	cvReleaseImage(&bright_cont_image);
	cvReleaseImage(&outputImage);
	cvReleaseImage(&rainbowImage);

	// Release the capture device housekeeping
	cvReleaseCapture( &capture );
	cvDestroyAllWindows();

	return 0;
}
