#include <stdio.h>
#include <stdbool.h>
#include <err.h>
#include <errno.h>
#include <math.h>
#include <X11/keysym.h>
#include <sys/timeb.h>
#include <opencv/cv.h>
#include <opencv/highgui.h>

#define PI 3.14159265359
#define RADIANS_PER_PX  0.0016
#define CAMERA_HEIGHT   33.5    // cm

double cameraAngle = 0.334;       // radians from horizontal
bool calibrated = true;
bool videoOut = true;
bool humanReadable = true;

/*
   Sets x,y to the average pixel location that exceeds threshold.
   Returns some confidence measure of result (number of pixels over
   threshold).
 */
int getPixelCenter(IplImage *frame, IplImage *img,
                    uint8_t threshold, int *x, int *y) {
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
        return count;
    }

    return count;
}

IplImage *process(IplImage **_img) {
    //fprintf(stderr, "Processing image:\n");
    IplImage *img = *_img;

    /* Convert to HSV */
    CvSize size = cvGetSize(img);
    IplImage *hsv = cvCreateImage(size, IPL_DEPTH_8U, 3);
    cvCvtColor(img, hsv, CV_BGR2HSV);
    //return hsv;

    /* Generate mask */
    IplImage *mask = cvCreateImage(size, 8, 1);
    cvInRangeS(hsv, cvScalar(0.10*256, 0.45*256, 0.20*256, 0),
               cvScalar(0.15*256, 1.00*256, 1.00*256, 0), mask);
    cvReleaseImage(&hsv);
    return mask;
}

void test_live(CvCapture *cam)
{
    int sampleCount = -1;
    double sampleDist = 0.0;
    double sampleSum = 0.0;

    while (1) {
        IplImage *img = cvQueryFrame(cam);
        IplImage *out = process(&img);

        char c = cvWaitKey(23);
    
        do {
            int x, y, confidence;
            // If color found
            if ((confidence = getPixelCenter(out, img, 254, &x, &y)) > 0) {
                // theta = angle down from center pixel
                // phi = angle right of center pixel
                double theta = (double)(y - 240) * RADIANS_PER_PX,
                    phi = (double)(320 - x) * RADIANS_PER_PX;
      
                if (!calibrated) {
                    // have not started sampling
                    if (sampleCount == -1) {
                        printf("Press 'c' when ready\n");
                        printf("theta = %.2f\tphi = %.2f\n", theta*180/PI, phi*180/PI);
                        if (c == 'c') {
                            sampleCount = 0;
                            sampleSum = 0.0;

                            printf("Enter actual distance (in cm): ");
                            scanf("%lf", &sampleDist);
                        } else {
                            break;
                        }
                    }
                    double camAngle = atan2(CAMERA_HEIGHT, sampleDist) - theta;
                    sampleSum += camAngle;
                    sampleCount += 1;

                    if (sampleCount > 10) {
                        cameraAngle = sampleSum / sampleCount;
                        calibrated = true;
                        printf("estimated camera angle: %.2f (%.2f deg)\n", cameraAngle,
                               cameraAngle*180/PI);
                    }
                } else { // calibrated
                    double d = CAMERA_HEIGHT / tan(theta + cameraAngle);
                    if (humanReadable)
                        printf("dist = %.2f\tphi = %.2f\n", d, phi*180/PI);
                    else
                        printf("%f,%f,%d\n", d, phi, confidence);
                }
      
            }
        } while (false);
    
        if (c == XK_q)
            return;

        if (videoOut) {
            cvShowImage("img", img);
            cvShowImage("out", out);
        }

        //cvReleaseImage(&img);
        cvReleaseImage(&out);
    }
}

int main(int argc, char **argv)
{
    // Parse command line options
    if (argc > 1) {
        // This is the flag to print only raw r,theta values
        // -a [optional camera angle in radians]
        if (strcmp(argv[1], "-a") == 0) {
            videoOut = false;
            calibrated = true;
            humanReadable = false;
            if (argc > 2) { 
                cameraAngle = atof(argv[2]);
            }
        }
    }

    /* create windows */
    if (videoOut) {
        cvNamedWindow("out", 0);
        cvMoveWindow("out", 200, 200);
        cvNamedWindow("img", 0);
        cvMoveWindow("img", 200, 200);
    }
  
  
    cvInitSystem(argc, argv);
    CvCapture *cam = cvCreateCameraCapture(0);
    /* Perform any additional camera initialization */
    test_live(cam);
  
    return 0;
}
