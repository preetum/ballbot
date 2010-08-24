#include <stdio.h>
#include <stdbool.h>
#include <err.h>
#include <errno.h>
#include <math.h>
#include <X11/keysym.h>
#include <sys/timeb.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#define VIDEO_OUT

#define RADIANS_PER_PX (0.0016)

class Robot {
	public:
	Robot() {
		cmdOut = popen("python control.py", "w");
		steering = 600;
		drive = 600;
		sweeper = 127;
		hopper = 127;
	}
	~Robot() {
		fclose(cmdOut);
	}

	void setSteering(uint16_t value) {
		steering = (value > 1023) ? 1023 : value;
	}
	void setDrive(uint16_t value) {
		drive = (value > 1023) ? 1023 : value;
	}
	void setSweeper(uint8_t value) {
		sweeper = (value > 255) ? 255 : value;
	}
	void setHopper(uint8_t value) {
		hopper = (value > 255) ? 255 : value;
	}

	void refresh() {
		fprintf(cmdOut, "%d, %d, %d, %d\n", steering, drive, sweeper, hopper);
		fflush(cmdOut);
	}

	private:
	// Initialize to center/neutral values
	uint16_t steering;
	uint16_t drive;
	uint8_t sweeper;
	uint8_t hopper;

	FILE* cmdOut;
};

Robot robot;

enum {
	STATE_LOOKING,
	STATE_FOUND,
	STATE_CLOSE,
	STATE_STOP
};

void robot_update(int x, int y) {
	static unsigned char state = STATE_LOOKING;
	
	switch (state) {
	case STATE_LOOKING:
		// When this loop is called, a ball has been found
		state = STATE_FOUND;
		//break; drop through

	case STATE_FOUND: {
	// Distance to target
	double theta = (double)(y - 240) * RADIANS_PER_PX,
		d = 26.5 / tan(theta + 0.197);
	// Angle/X offset to target
	double phi = (double)(320 - x) * RADIANS_PER_PX,
		xerror = d * tan(phi);

	printf("Center: (%d, %d)\n", x, y);
	printf("Distance: %f, theta= %f deg, phi= %f deg\n", 
		   d, theta * 180/3.141592654, phi * 180/3.141592654);

		robot.setDrive(510);
		if (xerror > 7.0)
			robot.setSteering(500);
		else if (xerror < -7.0)
			robot.setSteering(680);
		else
			robot.setSteering(600);
		robot.refresh();

		if (d < 50)
			state = STATE_CLOSE;
		break;
	}
	case STATE_CLOSE:
		//usleep(1000);
		state = STATE_STOP;
		break;

	case STATE_STOP:
	default:
		robot.setDrive(600);
		robot.refresh();
	}

}

void print_time(char *label)
{
	/*
	static struct timeb prev = {0,0};
	struct timeb cur;
	double diff = 0;
	ftime(&cur);
	if (prev.time) {
		diff  =  cur.time    - prev.time;
		diff += (cur.millitm - prev.millitm)/1000.0;
	}
	fprintf(stderr, "%30s  start = %d.%-3hu (+%5.3f)\n",
		label, (int)cur.time, cur.millitm, diff);
	prev = cur;
	*/
}

void cvOpen(const CvArr *src, CvArr *dst, IplConvKernel *element)
{
	cvErode (src, dst, element, 1);
	cvDilate(src, dst, element, 1);
}

void cvClose(const CvArr *src, CvArr *dst, IplConvKernel *element)
{
	cvDilate(src, dst, element, 1);
	cvErode (src, dst, element, 1);
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

IplImage *process(IplImage **_img)
{
	static CvRect roi = cvRect(0, 0, 640, 480);

	//fprintf(stderr, "Processing image:\n");
	IplImage *img = *_img;

	// Set ROI
	cvSetImageROI(img, roi);

	/* Convert to HSV */
	print_time("Converting to HSV");
	CvSize size = cvGetSize(img);
	IplImage *hsv = cvCreateImage(size, IPL_DEPTH_8U, 3);
	cvCvtColor(img, hsv, CV_BGR2HSV);
	//return hsv;

	/* Generate mask */
	IplImage *mask = cvCreateImage(size, 8, 1);
	cvInRangeS(hsv, cvScalar(0.11*256, 0.45*256, 0.20*256, 0),
	                cvScalar(0.14*256, 1.00*256, 1.00*256, 0), mask);
	cvReleaseImage(&hsv);

	/* Perform morphological ops 
	print_time("Performing morphologies");
	IplConvKernel *se21 = cvCreateStructuringElementEx(21, 21, 10, 10, CV_SHAPE_RECT, NULL);
	IplConvKernel *se11 = cvCreateStructuringElementEx(11, 11, 5,  5,  CV_SHAPE_RECT, NULL);
	cvClose(mask, mask, se21);
	cvOpen(mask, mask, se11);
	cvReleaseStructuringElement(&se21);
	cvReleaseStructuringElement(&se11);

	IplImage *hough_in = cvCreateImage(size, 8, 1);
	cvCopy(mask, hough_in, NULL);
	*/

	// invert image
	//cvXorS(mask, cvScalar(255), mask);
	//CBlobResult blobs = CBlobResult(mask, NULL, 0, false);
	//tune this threshold
	//blobs.Filter(blobs, B_EXCLUDE, CBlobGetArea(), B_LESS, 5);
	//CBlob biggestBlob;
	//blobs.GetNthBlob( CBlobGetArea(), 0, biggestBlob );


	/* Fancy up output */
	print_time("Generating output");
	//biggestBlob.FillBlob(img, CV_RGB(255, 0, 0));

	int x, y;
	if (getPixelCenter(mask, img, 254, &x, &y)) {
	  //robot_update(x, y);
	double theta = (double)(y - 240) * RADIANS_PER_PX,
		d = 26.5 / tan(theta + 0.197);
	// Angle/X offset to target
	double phi = (double)(320 - x) * RADIANS_PER_PX,
		xerror = d * tan(phi);

	printf("theta = %f\tphi = %f\n", theta*180/3.14159, phi*180/3.14159);
	}

	// Ghetto update ROI
	/*
	roi.x = x - 100;
	roi.y = y - 100;
	roi.height = roi.width = 200;
	*/

	return mask;
}

void test_live(CvCapture *cam)
{
	while (1) {
		IplImage *img = cvQueryFrame(cam);
		IplImage *out = process(&img);
#ifdef VIDEO_OUT
		cvShowImage("img", img);
		cvShowImage("out", out);
#endif
		if (cvWaitKey(23) == XK_q)
			return;
		//cvReleaseImage(&img);
		cvReleaseImage(&out);
	}
}

int main(int argc, char **argv)
{
	/* create windows */
#ifdef VIDEO_OUT
	cvNamedWindow("out", 0);
	cvMoveWindow("out", 200, 200);
	cvNamedWindow("img", 0);
	cvMoveWindow("img", 200, 200);
#endif


	cvInitSystem(argc, argv);
	CvCapture *cam = cvCreateCameraCapture(0);
	/* Perform any additional camera initialization */
	test_live(cam);

	return 0;
}
