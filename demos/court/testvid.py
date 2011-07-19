import sys

from opencv.cv import *
from opencv.highgui import *


# initialize window
cvNamedWindow('video', CV_WINDOW_AUTOSIZE)
cvMoveWindow('video', 10, 10)

capture = cvCreateFileCapture(sys.argv[1])
if not capture: print 'Error opening capture'; sys.exit(1)

frame = cvQueryFrame(capture)
while frame != None:
    cvShowImage('video', frame)
    frame = cvQueryFrame(capture)
    cvWaitKey(33)

cvReleaseCapture(capture)
cvDestroyWindow('video')
