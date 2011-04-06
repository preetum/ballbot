#! /usr/bin/env python

# Based on the capture-cam.py example from OpenCV docs
# 2010-12-01
# John Wang

import sys

# import the necessary things for OpenCV
from opencv import cv
from opencv import highgui

# the codec existing in cvcapp.cpp,
# need to have a better way to specify them in the future
# WARNING: I have see only MPEG1VIDEO working on my computer
H263 = 0x33363255
H263I = 0x33363249
MSMPEG4V3 = 0x33564944
MPEG4 = 0x58564944
MSMPEG4V2 = 0x3234504D
MJPEG = 0x47504A4D
MPEG1VIDEO = 0x314D4950
AC3 = 0x2000
MP2 = 0x50
FLV1 = 0x31564C46


def seek_onChange(pos, capture, windowName):
    '''Callback for the seek trackbar'''
    print 'Seeking to frame: %d' % pos

    # Set the pointer to frame pos and grab the frame
    highgui.cvSetCaptureProperty(capture, highgui.CV_CAP_PROP_POS_FRAMES, 
                                 pos*3600 - 1)
    frame = highgui.cvQueryFrame(capture)

    # Display the frame on the window
    highgui.cvShowImage(windowName, frame)
    

#############################################################################
# so, here is the main part of the program

if __name__ == '__main__':

    # a small welcome
    print "OpenCV Python capture video"

    # first, create the necessary window
    highgui.cvNamedWindow ('Camera', highgui.CV_WINDOW_AUTOSIZE)

    # move the new window to a better place
    highgui.cvMoveWindow ('Camera', 10, 10)

    if len (sys.argv) == 1:
        # no argument on the command line
        print "Required argument: path to file"
        sys.exit(1)
    else:
        # we have an argument on the command line,
        # we can assume this is a file name, so open it
        capture = highgui.cvCreateFileCapture (sys.argv [1])            

    # check that capture device is OK
    if not capture:
        print "Error opening capture device"
        sys.exit (1)

    # capture the 1st frame to get some propertie on it
    frame = highgui.cvQueryFrame (capture)

    # get size of the frame
    frame_size = cv.cvGetSize (frame)

    # get the frame rate of the capture device
    fps = highgui.cvGetCaptureProperty (capture, highgui.CV_CAP_PROP_FPS)
    if fps == 0:
        # no fps getted, so set it to 30 by default
        fps = 25

    frame_count = highgui.cvGetCaptureProperty(capture, 
                                               highgui.CV_CAP_PROP_FRAME_COUNT)
    frame_count = int(frame_count / 3600)
    highgui.cvCreateTrackbar('Seek', 'Camera', 0, int(frame_count),
                             lambda pos: seek_onChange(pos, capture, 'Camera'))

    # display the frames to have a visual output
    highgui.cvShowImage ('Camera', frame)

    i = 0
    start = 0
    play = False
    writer = None
    while 1:
        if play:
            frame = highgui.cvQueryFrame (capture)
            if frame is None:
                # no image captured... end the processing
                break
            # display the frames to have a visual output
            highgui.cvShowImage ('Camera', frame)

            # write frames to file
            if writer: highgui.cvWriteFrame (writer, frame)
            
        # wait for input
        k = highgui.cvWaitKey(int(1000/fps))

        if k == 27:
            # user has press the ESC key, so exit
            break
        elif k == 'm':
            # mark start of capture
            start = highgui.cvGetCaptureProperty(capture, 
                                                highgui.CV_CAP_PROP_POS_FRAMES)
            start = int(start)
            print 'Start capture at frame: %d' % start

            # create the writer
            writer = highgui.cvCreateVideoWriter (
                "captured%d.mpg" % i, MPEG1VIDEO,
                fps, frame_size, True)
            # check the writer is OK
            if not writer:
                print "Error opening writer"
                sys.exit (1)
            i += 1

        elif k == 'c':
            # pause
            play = False
            writer = None

        elif k == 's':
            # save image
            highgui.cvSaveImage('out.png', frame)

        elif k == 'p':
            # toggle playing state
            play = not play

            # if paused, update the slider
            if not play:
                pos = highgui.cvGetCaptureProperty(capture, 
                                                   highgui.CV_CAP_PROP_POS_FRAMES)
                print 'Current frame: %d' % pos

    # end working with the writer
    # not working at this time... Need to implement some typemaps...
    # but exiting without calling it is OK in this simple application
    #highgui.cvReleaseVideoWriter (writer)
