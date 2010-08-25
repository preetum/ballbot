// http://www.linuxconfig.org/introduction-to-computer-vision-with-opencv-on-linux

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

	int threshold = THRESHOLD;
	// first parameter: threshold
	if (argc > 1)
		threshold = atoi(argv[1]);

	// second parameter: 
	//  -d = output stuff to serial
	//  -v = output video
	if (argc > 2 && argv[2][0] == '-') {
		char *s = argv[2] + 1;
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

    int height, width, channels, pxPerRow;
	int ymin, ymax;
    uint8_t *data;

    frame = cvQueryFrame( capture );
    if( !frame ) return 0;

    height = frame->height;
    width = frame->width;
    pxPerRow = frame->widthStep;
    channels = frame->nChannels;
    data = (uint8_t*)frame->imageData;

    printf("Capturing %dx%d image, %d channels\n", width, height, channels);

    // Look for center
	ymin = 100;
	ymax = 140;
    while(frame) {
        uint32_t xsum = 0, ysum = 0, count = 0;

		// highlight bounds in green
		for (int j = 0; j < width; j += 1) {    // x-coordinate
			uint8_t* pixel = (uint8_t*)(data + ymin*pxPerRow + j*channels);
			pixel[0] = pixel[2] = 0;
			pixel[1] = 0xFF;

			pixel = (uint8_t*)(data + ymax*pxPerRow + j*channels);
			pixel[0] = pixel[2] = 0;
			pixel[1] = 0xFF;
		}
        for (int i = ymin; i < ymax; i += 1)         // y-coordinate
            for (int j = 0; j < width; j += 1) {    // x-coordinate
                // Get (r,g,b) color components of each pixel
                uint8_t* pixel = (uint8_t*)(data + i*pxPerRow + j*channels);
                uint8_t b = pixel[0],
                    g = pixel[1],
                    r = pixel[2];

                // Look for white: all channels must be over threshold
                if (r > threshold && g > threshold && b > threshold) {
                    // highlight in blue
                    pixel[0] = 0xFF;
                    pixel[1] = pixel[2] = 0;

                    xsum += j;
                    ysum += i;
                    count += 1;
                }
            }

        if (count > 0) {
			int err = xsum/count - 320;
			int steer = 145 + (err / 8);
			if (steer > 190) steer = 190;
			else if (steer < 100) steer = 100;

			if (out) {
				fprintf(out, "a[0]=%d\n", steer);
				fflush(out);
			}

            printf("Center: (%d, %d) Steer: %d\n", xsum/count, ysum/count, steer);
		}
        else
            printf("No white found\n");

		if (video)
			cvShowImage( "Example2", frame );

        char c = cvWaitKey(33);
        if( c == 27 ) break;

        frame = cvQueryFrame( capture );
    }
    cvReleaseCapture( &capture );

	if (video)
		cvDestroyWindow( "Example2" );
}
