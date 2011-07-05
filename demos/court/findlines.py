import sys
import cv

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

def find_lines(frame):
    # Resize to 640x480
    frame_small = cv.CreateMat(480, 640, cv.CV_8UC3)
    cv.Resize(frame, frame_small)

    # Threshold by distance: blank out all top pixels
    cv.Rectangle(frame_small, (0,0), (640, 80), (0,0,0,0), cv.CV_FILLED)

    frame_size = cv.GetSize(frame_small)
    frame_gray = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 1)
    edges = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 1)
    cv.CvtColor(frame_small, frame_gray, cv.CV_BGR2GRAY)

    cv.Canny(frame_gray, edges, 400, 400)
    cv.Dilate(edges, edges,
              cv.CreateStructuringElementEx(3, 3, 0, 0, 
                                            cv.CV_SHAPE_RECT))
    cv.Erode(edges, edges,
              cv.CreateStructuringElementEx(1, 1, 0, 0, 
                                            cv.CV_SHAPE_RECT))


    line_storage = cv.CreateMemStorage()
    lines = cv.HoughLines2(edges, line_storage, cv.CV_HOUGH_PROBABILISTIC,
                          1, cv.CV_PI/180.0, 300, 100, 40)
    print len(lines), 'lines found'
    for i in range(len(lines)):
        line = lines[i]
        cv.Line(frame_small, line[0], line[1], hv2rgb(360.0*i/len(lines), 1.0),
                3, 8)

    cv.ShowImage('frame', frame_small)
    cv.ShowImage('edges', edges)

def main():
    cv.NamedWindow('frame', cv.CV_WINDOW_AUTOSIZE)
    cv.MoveWindow('frame', 10, 10)
    cv.NamedWindow('edges', cv.CV_WINDOW_AUTOSIZE)
    cv.MoveWindow('edges', 600, 10)

    frame = cv.LoadImage(sys.argv[1])
    if frame is None:
        print 'Error loading image %s' % sys.argv[1]
        return
    find_lines(frame)

    # Pause for key press
    while True:
        k = cv.WaitKey(33)
        if k == 'q':
            break

if __name__ == '__main__':
    main()
