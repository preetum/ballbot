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
    cvNamedWindow('video', CV_WINDOW_AUTOSIZE)
    cvMoveWindow('video', 10, 10)

    cvNamedWindow('segment', CV_WINDOW_AUTOSIZE)
    cvMoveWindow('segment', 10, 500)

    cvNamedWindow('flow', CV_WINDOW_AUTOSIZE)
    cvMoveWindow('flow', 500, 10)

    cvNamedWindow('edges', CV_WINDOW_AUTOSIZE)
    cvMoveWindow('edges', 500, 500)


    capture = cvCreateFileCapture(sys.argv[1])
    if not capture: print 'Error opening capture'; sys.exit(1)

    # Load bg image
    bg = cvLoadImage('bg.png')

    # Discard some frames
    for i in xrange(2300):
        cvGrabFrame(capture)

    frame = cvQueryFrame(capture)
    frame_size = cvGetSize(frame)

    # vars for playback
    fps = 25
    play = True
    velx = cvCreateImage(frame_size, IPL_DEPTH_32F, 1)
    vely = cvCreateImage(frame_size, IPL_DEPTH_32F, 1)
    combined = cvCreateImage(frame_size, IPL_DEPTH_8U, 1)
    prev = cvCreateImage(frame_size, IPL_DEPTH_8U, 1)
    frame_gray = cvCreateImage(frame_size, IPL_DEPTH_8U, 1)
    frame_sub = cvCreateImage(frame_size, IPL_DEPTH_8U, 3)

    flow = cvCreateImage(frame_size, IPL_DEPTH_8U, 3)

    edges = cvCreateImage(frame_size, IPL_DEPTH_8U, 1)
    prev_edges = cvCreateImage(frame_size, IPL_DEPTH_8U, 1)
    storage = cvCreateMemStorage(0)

    blob_mask = cvCreateImage (frame_size, IPL_DEPTH_8U, 1)
    cvSet(blob_mask, 1)

    hough_in = cvCreateImage(frame_size, IPL_DEPTH_8U, 1)
    hough_storage = cvCreateMat( 100, 1, CV_32FC3 )

    silhouette = cvCreateImage(frame_size, IPL_DEPTH_8U, 1)
    mhi = cvCreateImage(frame_size, IPL_DEPTH_32F, 1)
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

    ball_last = None

    while True:

        if play:
            frame = cvQueryFrame(capture)

            # do background subtraction using manually created frame
            # TODO do averaging to generate bg
            cvSub(frame, bg, frame_sub)

            # convert to grayscale
            cvCvtColor(frame_sub, frame_gray, CV_BGR2GRAY)

            # edge detection/dilation
            cvCanny(frame_gray, edges, 50, 100)
            cvDilate(edges, edges,
                     cvCreateStructuringElementEx(7, 7, 0, 0, 
                                                  CV_SHAPE_ELLIPSE))
            #cvErode(edges, edges)
            cvShowImage('edges', edges)

            # do optical flow using edge detection/dilation image
            #cvCalcOpticalFlowLK(prev_edges, edges, (7,7), velx, vely)
            cvCalcOpticalFlowHS(prev_edges, edges, 1, velx, vely, 0.5,
                                (CV_TERMCRIT_ITER, 10, 0))
            #cvThreshold(vely, vely, 0.5, 0, CV_THRESH_TOZERO)
            #cvAdd(velx, vely, vely)
            cvShowImage('flow', vely)
            cvCopy(edges, prev_edges)

            '''
            # segment motion
            cvThreshold(vely, silhouette, 0.5, 255, CV_THRESH_BINARY)
            cvUpdateMotionHistory(silhouette, mhi, time.clock()/1000, 1)
            components = cvSegmentMotion(mhi, seg_mask, seg_storage,
                                         time.clock()/1000,
                                         1/fps)
            #http://opencv.willowgarage.com/documentation/python/imgproc_miscellaneous_image_transformations.html?highlight=connectedcomp#CvConnectedComp
            for c in components:
                area, value, rect = c.area, c.value, c.rect
                x, y, w, h = rect.x, rect.y, rect.width, rect.height
                cvRectangle(frame, (x,y), (x+w,y+h), CV_RGB(255, 0, 0))

            cvShowImage('segment', seg_mask)
            '''

            blobs = CBlobResult(edges, blob_mask, 100, True)
            blobs.filter_blobs(10, 10000)
            blob_count = blobs.GetNumBlobs()

            # Find blob closest to last ball position
            if ball_last:
                last_center = blob_center(ball_last)

                mind2 = 1e100
                ball = None
                for i in range(blob_count):
                    blob = blobs.GetBlob(i)
                    center = blob_center(blob)
                    d2 = distance_sq(last_center, center)
                    if d2 < mind2:
                        mind2 = d2
                        ball = blob
                if ball:
                    cvRectangle(frame, (ball.MinX(), ball.MinY()),
                                (ball.MaxX(), ball.MaxY()),
                                CV_RGB(0, 255, 0))
                ball_last = ball
            # Initialize ball estimate
            else:
                possible_ball = []
                for i in range(blob_count):
                    blob = blobs.GetBlob(i)

                    x, y = blob_center(blob)
                    if 40 < y < 142:
                        possible_ball.append(blob)

                    # Find fastest-moving possible ball
                    maxv = 3
                    ball = None
                    for blob in possible_ball:
                        #blob.FillBlob(frame, cvScalar(0, 255, 0), 0, 0)

                        x1, y1 = int(blob.MinX()), int(blob.MinY())
                        x2, y2 = int(blob.MaxX()), int(blob.MaxY())
                        
                        # get mean optical flow velocity over bounding box
                        vx = numpy.mean(velx[y1:y2, x1:x2])
                        vy = numpy.mean(vely[y1:y2, x1:x2])
                        '''
                        # get mean at center of blob
                        x, y = int((x1+x2)/2), int((y1+y2)/2)
                        vx = velx[y,x]
                        vy = vely[y,x]
                        '''
                        # v = proportional to sq magnitude of velocity vector
                        v = vx*vx + vy*vy
                        print v
                        if v > maxv:
                            ball = blob
                            maxv = v
                    if ball:
                        cvRectangle(frame, (ball.MinX(), ball.MinY()),
                                    (ball.MaxX(), ball.MaxY()),
                                    CV_RGB(0, 255, 0))
                        ball_last = ball

            cvShowImage('video', frame)

            '''
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
                cvRectangle(frame, (ball.MinX(), ball.MinY()),
                            (ball.MaxX(), ball.MaxY()), CV_RGB(0, 255, 0))
                #ball.FillBlob(frame, cvScalar(0, 255, 0), 0, 0)

            # Estimate position of near player
            if len(near_player) > 0:
                # Find x center
                xLocs = []
                yLocs = []
                for blob in near_player:
                    x, y = blob_center(blob)
                    xLocs.append(x)
                    yLocs.append(y)

                if len(xLocs) > 0:
                    xmin, ymin = min(xLocs), min(yLocs)
                    xmax, ymax = max(xLocs), max(yLocs)
                    cvRectangle(frame, (xmin, ymin), (xmax, ymax),
                                CV_RGB(255, 0, 0))

                    y_k = cvKalmanPredict(kf_near)
                    print 'near player at', y_k[0], y_k[1]
                    draw_cross(frame, y_k, CV_RGB(255, 0, 0), 10)

                    # z1, z2 = position measurement
                    z_k[0], z_k[1] = (xmin+xmax)/2, (ymin+ymax)/2
                    cvMatMulAdd(kf_near.measurement_matrix, x_near, z_k, z_k)
                    cvKalmanCorrect(kf_near, z_k)

                    # apply process noise and update state
                    cvRandArr(rng, w_k, CV_RAND_NORMAL, 0,
                              numpy.sqrt(kf_near.process_noise_cov[0,0]))
                    cvMatMulAdd(kf_near.transition_matrix, x_near, w_k, x_near)

            # Estimate position of far player
            xLocs = []
            yLocs = []
            for blob in far_player:
                x,y = blob_center(blob)
                xLocs.append(x)
                yLocs.append(y)
            if len(xLocs) > 0:
                xmin, ymin = min(xLocs), min(yLocs)
                xmax, ymax = max(xLocs), max(yLocs)
                cvRectangle(frame, (xmin, ymin), (xmax, ymax),
                            CV_RGB(0, 0, 255))
                print 'far player at', (xmin+xmax)/2, (ymin+ymax)/2

            # Estimate position of the ball
            y_k = cvKalmanPredict(kf_ball)
            x, y = y_k[0], y_k[1]
            print 'ball at', x, y
            draw_cross(frame, y_k, CV_RGB(0, 255, 0), 5)

            # kalman filter update
            if ball and maxv > 0.01:
                # z1, z2 = position measurement
                x, y = blob_center(ball)
                z_k[0], z_k[1] = x, y
                # z3, z4 = velocity measurement
                #x, y = int(x), int(y)
                #z_k[2], z_k[3] = cvGetReal2D(velx, y, x), \
                #    cvGetReal2D(vely, y, x)
                cvMatMulAdd(kf_ball.measurement_matrix, x_ball, z_k, z_k)
                cvKalmanCorrect(kf_ball, z_k)

                # apply process noise and update state
                cvRandArr(rng, w_k, CV_RAND_NORMAL, 0,
                          numpy.sqrt(kf_ball.process_noise_cov[0,0]))
                cvMatMulAdd(kf_ball.transition_matrix, x_ball, w_k, x_ball)
            #else:
                #kf_ball.transition_matrix[0,2] = 0
                #kf_ball.transition_matrix[1,3] = 0
                #x_ball = cvKalmanPredict(kf_ball)
            '''

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
