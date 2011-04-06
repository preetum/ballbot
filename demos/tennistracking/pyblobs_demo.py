#! /usr/bin/env python

from opencv import cv
from opencv import highgui

from blobs.BlobResult import CBlobResult
from blobs.Blob import CBlob	# Note: This must be imported in order to destroy blobs and use other methods

#############################################################################
# some useful functions

def hsv2rgb (hue):
    # convert the hue value to the corresponding rgb value

    sector_data = [[0, 2, 1],
                   [1, 2, 0],
                   [1, 0, 2],
                   [2, 0, 1],
                   [2, 1, 0],
                   [0, 1, 2]]
    hue *= 0.1 / 3
    sector = cv.cvFloor (hue)
    p = cv.cvRound (255 * (hue - sector))
    if sector & 1:
        p ^= 255

    rgb = {}
    rgb [sector_data [sector][0]] = 255
    rgb [sector_data [sector][1]] = 0
    rgb [sector_data [sector][2]] = p

    return cv.cvScalar (rgb [2], rgb [1], rgb [0], 0)


#############################################################################
# so, here is the main part of the program

if __name__ == '__main__':

    highgui.cvNamedWindow ('Blob View', highgui.CV_WINDOW_AUTOSIZE)


    import sys

    try:
        # try to get the device number from the command line
        device = int (sys.argv [1])

        # got it ! so remove it from the arguments
        del sys.argv [1]
    except (IndexError, ValueError):
        # no device number on the command line, assume we want the 1st device
        device = highgui.CV_CAP_ANY

    if len (sys.argv) == 1:
        # no argument on the command line, try to use the camera
        capture = highgui.cvCreateCameraCapture (device)

        # set the wanted image size from the camera
        highgui.cvSetCaptureProperty (capture,
                                      highgui.CV_CAP_PROP_FRAME_WIDTH, 320)
        highgui.cvSetCaptureProperty (capture,
                                      highgui.CV_CAP_PROP_FRAME_HEIGHT,240)


    # capture the 1st frame to get some propertie on it
    frame = highgui.cvQueryFrame (capture)

    # get some properties of the frame
    frame_size = cv.cvGetSize (frame)



    # create some images useful later
    my_grayscale = cv.cvCreateImage (frame_size, 8, 1)
    mask = cv.cvCreateImage (frame_size, 8, 1)
    cv.cvSet(mask, 1)

    blob_overlay = False

    while True:

        # 1. capture the current image
        frame = highgui.cvQueryFrame (capture)
        if frame is None:
            # no image captured... end the processing
            break

        # mirror the captured image
        cv.cvFlip (frame, None, 1)

	cv.cvCvtColor(frame, my_grayscale, cv.CV_RGB2GRAY);
	cv.cvThreshold(my_grayscale, my_grayscale, 128, 255, cv.CV_THRESH_BINARY)
	if not blob_overlay:
		# Convert black-and-white version back into three-color representation
		cv.cvCvtColor(my_grayscale, frame, cv.CV_GRAY2RGB);


	myblobs = CBlobResult(my_grayscale, mask, 100, True)

	myblobs.filter_blobs(10, 10000)
	blob_count = myblobs.GetNumBlobs()

	for i in range(blob_count):

		my_enumerated_blob = myblobs.GetBlob(i)
#		print "%d: Area = %d" % (i, my_enumerated_blob.Area())
		my_enumerated_blob.FillBlob(frame, hsv2rgb( i*180.0/blob_count ), 0, 0)




        # we can now display the images
        highgui.cvShowImage ('Blob View', frame)


        # handle events
        k = highgui.cvWaitKey (10)

        if k == '\x1b':
            # user has press the ESC key, so exit
            break
