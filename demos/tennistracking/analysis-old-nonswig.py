import sys, cv
from opencv import cv as cv0

from blobs.BlobResult import CBlobResult
from blobs.Blob import CBlob

def main():
    if len(sys.argv) == 1:
        print 'Usage: %s [inputfile]' % sys.argv[0]
        sys.exit(1)

    # initialize window
    cv.NamedWindow('video', cv.CV_WINDOW_AUTOSIZE)
    cv.MoveWindow('video', 10, 10)

    cv.NamedWindow('threshold', cv.CV_WINDOW_AUTOSIZE)
    cv.MoveWindow('threshold', 10, 500)

    cv.NamedWindow('flow', cv.CV_WINDOW_AUTOSIZE)
    cv.MoveWindow('flow', 500, 10)

    cv.NamedWindow('edges', cv.CV_WINDOW_AUTOSIZE)
    cv.MoveWindow('edges', 500, 500)

    cv.NamedWindow('combined', cv.CV_WINDOW_AUTOSIZE)
    cv.MoveWindow('combined', 1000, 10)

    capture = cv.CreateFileCapture(sys.argv[1])
    if not capture: print 'Error opening capture'; sys.exit(1)

    # Load bg image
    bg = cv.LoadImage('bg.png')

    # Discard some frames
    for i in xrange(2300):
        cv.GrabFrame(capture)

    frame = cv.QueryFrame(capture)
    frame_size = cv.GetSize(frame)

    # vars for playback
    fps = 25
    play = True
    velx = cv.CreateImage(frame_size, cv.IPL_DEPTH_32F, 1)
    vely = cv.CreateImage(frame_size, cv.IPL_DEPTH_32F, 1)
    combined = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 1)
    prev = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 1)
    curr = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 1)
    frame_sub = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 3)

    edges = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 1)
    prev_edges = None
    storage = cv.CreateMemStorage(0)

    blob_mask = cv0.cvCreateImage (frame_size, cv.IPL_DEPTH_8U, 1)
    cv0.cvSet(blob_mask, 1)

    hough_in = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 1)
    hough_storage = cv.CreateMat( 100, 1, cv.CV_32FC3 )

    '''
    cv.CvtColor(frame, prev, cv.CV_BGR2GRAY)
    frame = cv.QueryFrame(capture)
    cv.CvtColor(frame, curr, cv.CV_BGR2GRAY)

    # winSize can't have even numbers
    cv.CalcOpticalFlowLK(prev, curr, (3,3), velx, vely)
    cv.ShowImage('video', frame)
    cv.ShowImage('flow', velx)
    cv.WaitKey(0)
    '''

    while True:

        if play:
            frame = cv.QueryFrame(capture)
            cv.Sub(frame, bg, frame_sub)

            '''#detect people
            found = list(cv.HOGDetectMultiScale(frame, storage, win_stride=(8,8),
                padding=(32,32), scale=1.05, group_threshold=2))
            for r in found:
                (rx, ry), (rw, rh) = r
                tl = (rx + int(rw*0.1), ry + int(rh*0.07))
                br = (rx + int(rw*0.9), ry + int(rh*0.87))
                cv.Rectangle(frame, tl, br, (0, 255, 0), 3)
            '''

            #color thresholding
            hsv = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 3);
            cv.CvtColor(frame, hsv, cv.CV_BGR2HSV)
            mask = cv.CreateMat(frame_size[1], frame_size[0], 
                                cv.CV_8UC1)
            cv.InRangeS(hsv, (0.06*256, 0.2*256, 0.6*256, 0),
                        (0.16*256, 1.0*256, 1.0*256, 0), mask)
            cv.ShowImage('threshold', mask)
            

            #optical flow method
            # store previous frame
            prev, curr = curr, prev
            # convert next frame to single channel grayscale
            cv.CvtColor(frame_sub, curr, cv.CV_BGR2GRAY)
            #cv.CalcOpticalFlowLK(prev, curr, (3,3), velx, vely)
            #cv.Threshold(velx, velx, 8.0, 0, cv.CV_THRESH_TOZERO)
            cv.CalcOpticalFlowHS(prev, curr, 1, velx, vely, 0.5,
                                 (cv.CV_TERMCRIT_ITER, 10, 0))
            cv.Threshold(velx, velx, 0.5, 0, cv.CV_THRESH_TOZERO)
            cv.Threshold(vely, vely, 0.5, 0, cv.CV_THRESH_TOZERO)
            cv.Erode(vely, vely, 
                     cv.CreateStructuringElementEx(2, 2, 0, 0, 
                                                   cv.CV_SHAPE_ELLIPSE))
            cv.Add(vely, velx, vely)
            cv.ShowImage('flow', vely)

            #edge detection
            cv.Canny(curr, edges, 50, 100)
            cv.Dilate(edges, edges,
                     cv.CreateStructuringElementEx(7, 7, 0, 0, 
                                                   cv.CV_SHAPE_ELLIPSE))
            cv.ShowImage('edges', edges)

            if prev_edges:
                cv.CalcOpticalFlowHS(prev_edges, edges, 1, velx, vely, 0.5,
                                     (cv.CV_TERMCRIT_ITER, 10, 0))
                cv.Threshold(velx, velx, 0.5, 0, cv.CV_THRESH_TOZERO)
                cv.Threshold(vely, vely, 0.5, 0, cv.CV_THRESH_TOZERO)
                cv.ShowImage('flow', vely)
            prev_edges = edges

            cv.Threshold(vely, combined, 0.5, 255, cv.CV_THRESH_BINARY)
            cv.Min(combined, edges, combined)
            cv.ShowImage('combined', combined)

            # blobs
            myblobs = CBlobResult(edges, blob_mask, 100, False)
            myblobs.filter_blobs(10, 10000)
            blob_count = myblobs.GetNumBlobs()
            
            for i in range(blob_count):
                
                my_enumerated_blob = myblobs.GetBlob(i)
#               print "%d: Area = %d" % (i, my_enumerated_blob.Area())
                my_enumerated_blob.FillBlob(frame, 
                                            hsv2rgb( i*180.0/blob_count), 0, 0)


            cv.ShowImage('video', frame)

            ''' crashes
            #hough transform on dilated image
            #http://wiki.elphel.com/index.php?
            # title=OpenCV_Tennis_balls_recognizing_tutorial&redirect=no
            cv.Copy(edges, hough_in)
            cv.Smooth(hough_in, hough_in, cv.CV_GAUSSIAN, 15, 15, 0, 0)
            cv.HoughCircles(hough_in, hough_storage, cv.CV_HOUGH_GRADIENT,
                            4, frame_size[1]/10, 100, 40, 0, 0)
            print hough_storage
            '''


        k = cv.WaitKey(1000/fps)
        if k == 27:  # ESC key
            break
        elif k == 'p':   # play/pause
            play = not play


if __name__ == '__main__':
    main()
