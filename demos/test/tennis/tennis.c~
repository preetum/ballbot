#include <stdio.h>
#include <err.h>
#include <errno.h>
#include <X11/keysym.h>
#include <sys/timeb.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

const char *images[] = {
	"images/1-10.jpg",
	"images/1-12.jpg",
	"images/1-13.jpg",
	"images/1-17.jpg",
	"images/1-18.jpg",
	"images/1-19.jpg",
	NULL
};

void print_time(char *label)
{
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

IplImage *process(IplImage **_img)
{
	fprintf(stderr, "Processing image:\n");
	IplImage *img = *_img;

	/* Convert to HSV */
	print_time("Converting to HSV");
	CvSize size = cvGetSize(img);
	IplImage *hsv = cvCreateImage(size, IPL_DEPTH_8U, 3);
	cvCvtColor(img, hsv, CV_BGR2HSV);
	//return hsv;

	/* Generate mask */
	CvMat *mask = cvCreateMat(size.height, size.width, CV_8UC1);
	cvInRangeS(hsv, cvScalar(0.11*256, 0.60*256, 0.20*256, 0),
	                cvScalar(0.14*256, 1.00*256, 1.00*256, 0), mask);
	cvReleaseImage(&hsv);
	//IplImage *tmp = cvCreateImage(size, 8, 1);
	//cvCopy(mask, tmp, NULL);
	//return mask;

	/* Perform morphological ops */
	print_time("Performing morphologies");
	IplConvKernel *se21 = cvCreateStructuringElementEx(21, 21, 10, 10, CV_SHAPE_RECT, NULL);
	IplConvKernel *se11 = cvCreateStructuringElementEx(11, 11, 5,  5,  CV_SHAPE_RECT, NULL);
	cvClose(mask, mask, se21);
	cvOpen(mask, mask, se11);
	cvReleaseStructuringElement(&se21);
	cvReleaseStructuringElement(&se11);
	//IplImage *tmp = cvCreateImage(size, 8, 1);
	//cvCopy(mask, tmp, NULL);
	//return mask;

	/* Hough transform */
	IplImage *hough_in = cvCreateImage(size, 8, 1);
	cvCopy(mask, hough_in, NULL);

	print_time("Finding hough circles");
	CvMemStorage *storage = cvCreateMemStorage(0);
        cvSmooth(hough_in, hough_in, CV_GAUSSIAN, 15, 15, 0, 0);
	CvSeq *circles = cvHoughCircles(
		hough_in, storage, // input, storage, 
		CV_HOUGH_GRADIENT, 4, size.height/10,
		                   // type, 1/scale, min center dists
		100, 40,           // params1?, param2?
		0, 0               // min radius, max radius
	);
	cvReleaseMemStorage(&storage);

	/* Fancy up output */
	print_time("Generating output");
	int i;
	for (i = 0; i < circles->total; i++) {
             float *p = (float*)cvGetSeqElem(circles, i);
	     CvPoint center = cvPoint(cvRound(p[0]),cvRound(p[1]));
	     CvScalar val = cvGet2D(mask, center.y, center.x);
	     if (val.val[0] < 1) continue;
             cvCircle(img,  center, 3,             CV_RGB(0,255,0), -1, CV_AA, 0);
             cvCircle(img,  center, cvRound(p[2]), CV_RGB(255,0,0),  3, CV_AA, 0);
             cvCircle(mask, center, 3,             CV_RGB(0,255,0), -1, CV_AA, 0);
             cvCircle(mask, center, cvRound(p[2]), CV_RGB(255,0,0),  3, CV_AA, 0);
	}

	return hough_in;
}

void test_images(const char **images)
{
	int i = 0;
	while (i >= 0 && images[i]) {
		/* Read image */
		fprintf(stderr, "Reading input %d: `%s'\n", i, images[i]);
		IplImage *img = cvLoadImage(images[i], 1);

		/* Processing */
		IplImage *out = process(&img);

		/* Show a window */
		print_time("Showing images");
		fprintf(stderr, "\n");
		cvShowImage("out", out);
		cvShowImage("img", img);

		/* Loop */
		int key = cvWaitKey(0);
		switch (key) {
		case XK_Right: i++; break;
		case XK_j:     i++; break;
		case XK_Left:  i--; break;
		case XK_k:     i--; break;
		case XK_q: return;
		}
	}
}

void test_live(CvCapture *cam)
{
	while (1) {
		cvGrabFrame(cam);
		IplImage *img = cvRetrieveFrame(cam);
		IplImage *out = process(&img);
		cvShowImage("img", img);
		cvShowImage("out", out);
		if (cvWaitKey(10) == XK_q)
			return;
		cvReleaseImage(&img);
		cvReleaseImage(&out);
	}
}

int main(int argc, char **argv)
{
	/* create windows */
	cvNamedWindow("out", 0);
	cvMoveWindow("out", 200, 200);
	cvNamedWindow("img", 0);
	cvMoveWindow("img", 200, 200);

	if (argc > 1 && !strcmp(argv[1], "-t")) {
		test_images(images);
	} else {
		cvInitSystem(argc, argv);
		CvCapture *cam = cvCreateCameraCapture(0);
		/* Perform any additional camera initialization */
		test_live(cam);
	}

	return 0;
}
