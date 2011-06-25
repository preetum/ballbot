#include <stdio.h>
#include <stdbool.h>
#include <math.h>

#include <cv.h>
#include <highgui.h>

using namespace cv;

int main(int argc, char **argv)
{
  bool run = true;
  Mat frame, out;

  // Create windows
  cvNamedWindow("img", 0);
  cvMoveWindow("img", 50, 50);

  if (argc > 1) {
    // Use image file
    frame = imread(argv[1]);
    resize(frame, frame, Size(640, 480));
    imshow("img", frame);
    cvWaitKey(0);
  } else {
    VideoCapture cam(0);
    cam.set(CV_CAP_PROP_FRAME_WIDTH, 320);
    cam.set(CV_CAP_PROP_FRAME_HEIGHT, 240);

    if (!cam.isOpened()) {
      printf("could not open camera\n");
      return -1;
    }
    printf("initialized camera\n");
    cam >> frame;
    printf("%d %d\n", frame.cols, frame.rows);

    while (run) {
      cam >> frame;
      imshow("img", frame);

      if ((char)cvWaitKey(33) == 'q')
        run = false;
    }
  }
}
