import sys

from opencv.cv import *
from opencv.highgui import *

def find_corners(frame):
    # Resize to 640x480
    frame_small = cvCreateMat(480, 640, CV_8UC3)
    cvResize(frame, frame_small)

    frame_size = cvGetSize(frame_small)
    frame_gray = cvCreateImage(frame_size, IPL_DEPTH_8U, 1)
    edges = cvCreateImage(frame_size, IPL_DEPTH_8U, 1)
    cvCvtColor(frame_small, frame_gray, CV_BGR2GRAY)
    cvCanny(frame_gray, edges, 400, 400)
    cvDilate(edges, edges,
             cvCreateStructuringElementEx(5, 5, 0, 0, 
                                          CV_SHAPE_ELLIPSE))
    #cvErode(edges, edges)

    line_storage = cvCreateMemStorage(0)
    lines = cvHoughLines2(edges, line_storage, CV_HOUGH_PROBABILISTIC,
                          1, CV_PI/180, 80, 200, 10)
    for i in range(lines.total):
        line = cvGetSeqElem(lines, i)
        print line
        #cvLine(frame_small, line[0], line[1], CV_RGB(255,0,0), 3, 8)

    # eig_image = cvCreateMat(480, 640, CV_32FC1)
    # temp_image = cvCreateMat(480, 640, CV_32FC1)
    # for (x,y) in cvGoodFeaturesToTrack(frame_small, eig_image, temp_image, 
    #                                    10, 0.04, 1.0):
    #     print "good feature at", x,y

    cvShowImage('frame', frame_small)
    cvShowImage('edges', edges)

def main():
    cvNamedWindow('frame', CV_WINDOW_AUTOSIZE)
    cvMoveWindow('frame', 10, 10)
    cvNamedWindow('edges', CV_WINDOW_AUTOSIZE)
    cvMoveWindow('edges', 200, 10)

    frame = cvLoadImage(sys.argv[1])
    if frame is None:
        print 'Error loading image %s' % sys.argv[1]
        return
    find_corners(frame)

    # Pause for key press
    cvWaitKey(0)

if __name__ == '__main__':
    main()
