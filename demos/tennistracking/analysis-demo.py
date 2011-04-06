# OpenCV reference
# http://opencv.willowgarage.com/documentation/python/
# Note: this is for NEW python bindings; this code uses the old SWIG bindings
#  for compatibility with pyblobs

# OpenCV samples
# http://www.cs.iit.edu/~agam/cs512/lect-notes/opencv-intro/opencv-intro.html

# Erosion and dilation
# http://idlastro.gsfc.nasa.gov/idl_html_help/Eroding_and_Dilating_Image_Objects.html

# Kalman filtering examples
# http://dasl.mem.drexel.edu/~noahKuntz/openCVTut9.html#Step%202
# https://code.ros.org/svn/opencv/trunk/opencv/samples/python/kalman.py

# Edge detection
# http://dasl.mem.drexel.edu/~noahKuntz/openCVTut5.html

# Tennis ball recognition using thresholding and hough transform
# http://wiki.elphel.com/index.php/OpenCV_Tennis_balls_recognizing_tutorial

import sys, time
import numpy
from opencv.cv import *
from opencv.highgui import *

from blobs.BlobResult import CBlobResult
from blobs.Blob import CBlob

# Turns hue,value into RGB value (saturation=1.0)
#  hue=[0,360) degrees
#  value=[0,1]
# Returns cvScalar of rgb values=[0,255]
# source: Internet
# http://en.wikipedia.org/wiki/HSL_and_HSV#Converting_to_RGB
def hv2rgb(hue, value):
    c = value # * saturation
    h = cvFloor(hue / 360.0)
    x = c if h & 1 else 0

    sector_data = [[0,2,1], [1,2,0], [1,0,2], [2,0,1], [2,1,0], [0,1,2]];
    rgb = [0]*3
    rgb[sector_data[sector][0]] = int(c * 255);
    rgb[sector_data[sector][1]] = 0;
    rgb[sector_data[sector][2]] = int(x * 255);
    
'''
    sector_data = [[0,2,1], [1,2,0], [1,0,2], [2,0,1], [2,1,0], [0,1,2]];
    hue *= 1.0/30
    sector = cvFloor(hue)
    p = cvRound(255*(hue - sector));
    p ^= sector & 1 ? 255 : 0;

    rgb = [0]*3
    rgb[sector_data[sector][0]] = 255;
    rgb[sector_data[sector][1]] = 0;
    rgb[sector_data[sector][2]] = p;

    return cvScalar(rgb[2], rgb[1], rgb[0], 0)
'''

# Returns weighted center by blob area
# blobs is a list of blobs
def weighted_center(blobs):
    xcenter, ycenter = 0, 0
    totalArea = 0

    if len(blobs) == 0:
        return None

    # weight by blob area
    for blob in blobs:
        x,y = blob_center(blob)
        area = blob.Area()
        xcenter += x * area
        ycenter += y * area
        totalArea += area

    # normalize out the weights
    xcenter = xcenter / totalArea
    ycenter = ycenter / totalArea

    return (xcenter, ycenter)

def distance_sq(u, v):
    return pow(u[0]-v[0], 2) + pow(u[1]-v[1], 2)

def remove_outlier(blobs):
    '''
    Removes one outlier only if len(blobs) >= 2.
    Outlier is defined as blob whose center is the farthest from the center
    '''
    
    if len(blobs) < 2:
        return None

    center = weighted_center(blobs)
    def k(blob):
        return distance_sq(center, blob_center(blob))
    blobs.sort(key=k, reverse=True)
    outlier = blobs[0]
    del blobs[0]
    return outlier

def draw_cross(img, center, color, d):
    cvLine(img, (center[0] - d, center[1] - d),                
            (center[0] + d, center[1] + d), color, 1, cv.CV_AA, 0) 
    cvLine(img, (center[0] + d, center[1] - d),                
            (center[0] - d, center[1] + d), color, 1, cv.CV_AA, 0)

def blob_center(blob):
    x = (blob.MaxX() + blob.MinX()) / 2
    y = (blob.MaxY() + blob.MinY()) / 2
    return (x, y)

def main():
    if len(sys.argv) == 1:
        print 'Usage: %s [inputfile]' % sys.argv[0]
        sys.exit(1)

    # initialize window
    cvNamedWindow('threshold', CV_WINDOW_AUTOSIZE)
    cvMoveWindow('threshold', 10, 10)

    cvNamedWindow('combined', CV_WINDOW_AUTOSIZE)
    cvMoveWindow('combined', 10, 10)


    cvNamedWindow('video', CV_WINDOW_AUTOSIZE)
    cvMoveWindow('video', 10, 10)

    cvNamedWindow('flow', CV_WINDOW_AUTOSIZE)
    cvMoveWindow('flow', 500, 10)

    cvNamedWindow('edges', CV_WINDOW_AUTOSIZE)
    cvMoveWindow('edges', 500, 10)

    capture = cvCreateFileCapture(sys.argv[1])
    if not capture: print 'Error opening capture'; sys.exit(1)

    # Load bg image
    bg = cvLoadImage('bg.png')

    # Discard some frames
    cvSetCaptureProperty(capture, CV_CAP_PROP_POS_FRAMES,
                         #8276399)
                         7919999)
    #for i in xrange(2300):
    #    cvGrabFrame(capture)

    #print cvGetCaptureProperty(capture, 
    #                           CV_CAP_PROP_POS_FRAMES)

    frame = cvQueryFrame(capture)
    frame_size = cvGetSize(frame)

    # vars for playback
    fps = 25
    play = True
    velx = cvCreateImage(frame_size, IPL_DEPTH_32F, 1)
    vely = cvCreateImage(frame_size, IPL_DEPTH_32F, 1)
    combined = cvCreateImage(frame_size, IPL_DEPTH_8U, 1)
    prev = cvCreateImage(frame_size, IPL_DEPTH_8U, 1)
    curr = cvCreateImage(frame_size, IPL_DEPTH_8U, 1)
    frame_sub = cvCreateImage(frame_size, IPL_DEPTH_8U, 3)

    flow = cvCreateImage(frame_size, IPL_DEPTH_8U, 3)

    edges = cvCreateImage(frame_size, IPL_DEPTH_8U, 1)
    prev_edges = None
    storage = cvCreateMemStorage(0)

    blob_mask = cvCreateImage (frame_size, IPL_DEPTH_8U, 1)
    cvSet(blob_mask, 1)

    hough_in = cvCreateImage(frame_size, IPL_DEPTH_8U, 1)
    hough_storage = cvCreateMat( 100, 1, CV_32FC3 )

    seg_mask = cvCreateImage(frame_size, IPL_DEPTH_32F, 1)
    seg_storage = cvCreateMemStorage(0)

    kf_near = cvCreateKalman(4, 2, 0)
    kf_far = cvCreateKalman(4, 2, 0)
    kf_ball = cvCreateKalman(4, 2, 0)

    for kalman in [kf_near, kf_far, kf_ball]:
        # transition matrix F is initialized to identity
        # want F = [1 0 1 0
        #           0 1 0 1
        #           0 0 1 0
        #           0 0 0 1]
        # because x1 = x coordinate and x3 = d/dt x1
        #         x2 = y coordinate and x4 = d/dt x3
        kalman.transition_matrix[0,2] = 1
        kalman.transition_matrix[1,3] = 1
        cvSetIdentity(kalman.measurement_matrix, 1)
        cvSetIdentity(kalman.process_noise_cov, 1e-5)
        cvSetIdentity(kalman.measurement_noise_cov, 1e-1)
        cvSetIdentity(kalman.error_cov_post, 1)

    cvSetIdentity(kf_ball.process_noise_cov, 1e-3)

    # initialize random number generator
    rng = cvRNG()

    # x_k = state variables
    x_ball = cvCreateMat(4, 1, CV_32FC1)
    cvZero(x_ball)
    cvZero(kf_ball.state_post)
    x_near = cvCreateMat(4, 1, CV_32FC1)
    cvZero(x_near)
    cvZero(kf_near.state_post)

    # w_k = noise
    w_k = cvCreateMat(4, 1, CV_32FC1)

    # z_k = measurements
    z_k = cvCreateMat(2, 1, CV_32FC1)


    frame_count = 0
    ball_count = 0
    present_count = 0
    present = []
    while frame_count<585:

        if play:
            frame_count += 1

            frame = cvQueryFrame(capture)

            start_time=time.clock()
            cvSub(frame, bg, frame_sub)

            '''#detect people
            found = list(cvHOGDetectMultiScale(frame, storage, win_stride=(8,8),
                padding=(32,32), scale=1.05, group_threshold=2))
            for r in found:
                (rx, ry), (rw, rh) = r
                tl = (rx + int(rw*0.1), ry + int(rh*0.07))
                br = (rx + int(rw*0.9), ry + int(rh*0.87))
                cvRectangle(frame, tl, br, (0, 255, 0), 3)
            '''
            '''
            #color thresholding
            hsv = cvCreateImage(frame_size, IPL_DEPTH_8U, 3);
            cvCvtColor(frame, hsv, CV_BGR2HSV)
            mask = cvCreateMat(frame_size.height, frame_size.width, 
                                CV_8UC1)
            cvInRangeS(hsv, (0.03*256, 0.2*256, 0.6*256, 0),
                        (0.16*256, 1.0*256, 1.0*256, 0), mask)
            cvShowImage('threshold', mask)
            '''

            #optical flow method
            # store previous frame
            prev, curr = curr, prev
            # convert next frame to single channel grayscale
            cvCvtColor(frame_sub, curr, CV_BGR2GRAY)
            #cvCalcOpticalFlowLK(prev, curr, (3,3), velx, vely)
            #cvThreshold(velx, velx, 8.0, 0, CV_THRESH_TOZERO)
            cvCalcOpticalFlowHS(prev, curr, 1, velx, vely, 0.5,
                                 (CV_TERMCRIT_ITER, 10, 0))
            cvThreshold(velx, velx, 0.5, 0, CV_THRESH_TOZERO)
            cvThreshold(vely, vely, 0.5, 0, CV_THRESH_TOZERO)
            cvErode(vely, vely, 
                     cvCreateStructuringElementEx(2, 2, 0, 0, 
                                                   CV_SHAPE_ELLIPSE))
            cvAdd(vely, velx, vely)
            cvShowImage('flow', vely)

            #edge detection/dilation
            cvCanny(curr, edges, 50, 100)
            cvDilate(edges, edges,
                     cvCreateStructuringElementEx(7, 7, 0, 0, 
                                                  CV_SHAPE_ELLIPSE))
            #cvErode(edges, edges)
            cvShowImage('edges', edges)

            # do optical flow using edge detection/dilation
            if prev_edges:
                cvCalcOpticalFlowHS(prev_edges, edges, 1, velx, vely, 0.5,
                                     (CV_TERMCRIT_ITER, 10, 0))
                #cvThreshold(vely, vely, 0.5, 0, CV_THRESH_TOZERO)
                #cvShowImage('flow', vely)
            prev_edges = edges

            cvThreshold(vely, combined, 0.5, 255, CV_THRESH_BINARY)
            cvMin(combined, edges, combined)
            cvShowImage('combined', combined)

            # blobs
            myblobs = CBlobResult(edges, blob_mask, 100, True)
            myblobs.filter_blobs(10, 10000)
            blob_count = myblobs.GetNumBlobs()

            near_player = []
            far_player = []
            possible_ball = []
            for i in range(blob_count):
                blob = myblobs.GetBlob(i)

                # Group blobs by their center point
                x, y = blob_center(blob)
                if y > 142:
                    near_player.append(blob)
                elif y < 40:
                    far_player.append(blob)
                else:
                    possible_ball.append(blob)
                    
            '''
            # Remove outliers - this was commented out because
            #  it's not as effective as hoped
            outliers = [remove_outlier(near_player), 
                        remove_outlier(far_player)]
            outliers = [x for x in outliers if x]

            print outliers

            # If no ball found in the range y=(40, 145) then
            #  find the closest blob to our state estimate
            #  that is small enough to be a ball.
            if len(possible_ball) == 0:
                outliers = [blob for blob in outliers if blob.Area() < 100]
                if len(outliers) > 0:
                    center = cvKalmanPredict(kf_ball)
                    outliers.sort(key=lambda x: \
                                      distance_sq(blob_center(x), center))
                    possible_ball.append(outliers[0])
            '''

            ball = None
            maxv = -1
            # Find the ball that's moving the fastest
            for blob in possible_ball:
                # get optical flow velocity at blob center
                x, y = blob_center(blob)
                x, y = int(x), int(y)
                # v = magnitude of velocity vector
                v = pow(cvGetReal2D(velx, y, x), 2) + \
                    pow(cvGetReal2D(vely, y, x), 2)
                if v > maxv:
                    ball = blob
                    maxv = v
            if ball and maxv > 0.01:
                x, y = blob_center(ball)
                print x, y
                #cvRectangle(frame, (ball.MinX(), ball.MinY()),
                #            (ball.MaxX(), ball.MaxY()), CV_RGB(0, 255, 0))
                cvRectangle(frame, (x-5,y-5), (x+5,y+5), CV_RGB(0, 255, 0))
                ball_count += 1
                #ball.FillBlob(frame, cvScalar(0, 255, 0), 0, 0)

            # Ask user if moving ball is present
            has_ball = raw_input("Moving ball present [y/N]?")
            has_ball = (has_ball.lower() == 'y')
            if has_ball: present_count += 1
            present.append(has_ball)
            
            print 'ball found in %d of %d frames' % \
                (ball_count, present_count)
            cvShowImage('video', frame)

            #print time.clock()-start_time

            ''' crashes
            #hough transform on dilated image
            cvCopy(edges, hough_in)
            cvSmooth(hough_in, hough_in, CV_GAUSSIAN, 15, 15, 0, 0)
            cvHoughCircles(hough_in, hough_storage, CV_HOUGH_GRADIENT,
                            4, frame_size[1]/10, 100, 40, 0, 0)
            print hough_storage
            '''


        k = cvWaitKey(1000/fps)
        if k == 27:  # ESC key
            break
        elif k == 'p':   # play/pause
            play = not play


if __name__ == '__main__':
    main()
