import sys, cv, time

import util
from gui import Simulator
from filter import ParticleFilter

def hv2rgb(hue, value):
    c = value # * saturation
    h = cv.Floor(hue / 60.0)
    x = c if h & 1 else 0

    sector_data = [[0,2,1], [1,2,0], [1,0,2], [2,0,1], [2,1,0], [0,1,2]];
    rgb = [0]*3
    rgb[sector_data[h][0]] = int(c * 255);
    rgb[sector_data[h][1]] = 0;
    rgb[sector_data[h][2]] = int(x * 255);
    return cv.CV_RGB(rgb[0], rgb[1], rgb[2])

def find_corners(frame, pf):
    # Resize to 640x480
    frame_small = cv.CreateMat(480, 640, cv.CV_8UC3)
    cv.Resize(frame, frame_small)

    frame_size = cv.GetSize(frame_small)
    frame_gray = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 1)
    edges = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 1)
    cv.CvtColor(frame_small, frame_gray, cv.CV_BGR2GRAY)
    cv.Canny(frame_gray, edges, 400, 400)
    cv.Dilate(edges, edges)

    line_storage = cv.CreateMemStorage()
    lines = cv.HoughLines2(edges, line_storage, cv.CV_HOUGH_PROBABILISTIC,
                          1, cv.CV_PI/180.0, 300, 100, 40)
    print len(lines), 'lines found'
    for i in range(len(lines)):
        line = lines[i]
        cv.Line(frame_small, line[0], line[1], hv2rgb(360.0*i/len(lines), 1.0),
                3, 8)
        print line

        # Generate an observation: (dist, heading) to line
        if i < 4:
            p1 = util.pixelToDistance(line[0])
            p2 = util.pixelToDistance(line[1])
            dist = util.pointLineDistance((0, 0), (p1, p2))
            pf.observeLine((dist, 0))

    # Find corners
    eig_image = cv.CreateImage(frame_size, cv.IPL_DEPTH_32F, 1)
    temp_image = cv.CreateImage(frame_size, cv.IPL_DEPTH_32F, 1)
    corners = cv.GoodFeaturesToTrack(frame_gray, eig_image, temp_image, 
                                     10, 0.04, 1.0, useHarris=True)
    # Take 2 strongest corners
    for pt in corners[:2]:
        print "good feature at", pt[0], pt[1]
        cv.Circle(frame_small, pt, 5, cv.CV_RGB(255,0,0), 2, 5, 0)

    cv.ShowImage('frame', frame_small)
    cv.ShowImage('edges', edges)

def main():
    # initialize OpenCV windows
    cv.NamedWindow('frame', cv.CV_WINDOW_AUTOSIZE)
    cv.MoveWindow('frame', 10, 10)
    cv.NamedWindow('edges', cv.CV_WINDOW_AUTOSIZE)
    cv.MoveWindow('edges', 200, 10)

    # initialize Simulator window
    pf = ParticleFilter(numParticles=1000)
    sim = Simulator(pf)
#    sim.start()
    sim.refresh()

    time.sleep(1)

    frame = cv.LoadImage(sys.argv[1])
    if frame is None:
        print 'Error loading image %s' % sys.argv[1]
        return
    find_corners(frame, pf)
    sim.refresh()

    # Pause for key press
    while True:
        k = cv.WaitKey(33)
        if k == 'q':
            break

if __name__ == '__main__':
    main()
