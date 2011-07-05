import sys, math
import cv

def cvOpen(src, dst, element):
  cv.Erode (src, dst, element)
  cv.Dilate(src, dst, element)

def cvClose(src, dst, element):
  cv.Dilate(src, dst, element)
  cv.Erode (src, dst, element)

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

def normalize_line(line):
    '''
    Returns normalized line = (r, theta) such that
    r >=0
    0 <= theta < 2pi
    '''
    r, theta = line
    if r < 0:
        r = -r
        theta += math.pi
    return (r, theta % (2*math.pi))

def avg_lines(lines):
    '''
    Computes some average of the lines (given as (r, theta) pairs)
    Returns average (r, theta)
    '''
    r = sum(x[0] for x in lines) / len(lines)
    theta = math.atan2(sum(math.sin(x[1]) for x in lines),
                       sum(math.cos(x[1]) for x in lines))
    return normalize_line((r, theta))

def angle_difference(a, b):
    '''
    Computes the angle difference (a - b), normalized to [-pi, pi)
    '''
    diff = a - b
    return (diff + math.pi) % (2*math.pi) - math.pi

def find_lines(frame):
    # Resize to 640x480
    frame_small = cv.CreateMat(480, 640, cv.CV_8UC3)
    cv.Resize(frame, frame_small)

    # Threshold by distance: blank out all top pixels
    cv.Rectangle(frame_small, (0,0), (640, 80), (0,0,0,0), cv.CV_FILLED)

    # Convert to grayscale
    frame_size = cv.GetSize(frame_small)
    frame_gray = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 1)
    cv.CvtColor(frame_small, frame_gray, cv.CV_BGR2GRAY)

    # Use color thresholding to get white lines
    edges = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 1)
    cv.Threshold(frame_gray, edges, 200, 255, cv.CV_THRESH_BINARY)

    openElement = cv.CreateStructuringElementEx(11,11,5,5,
                                                cv.CV_SHAPE_RECT)
    closeElement = cv.CreateStructuringElementEx(21, 21, 10, 10,
                                                 cv.CV_SHAPE_RECT)
    cvOpen(edges, edges, openElement)
    cvClose(edges, edges, closeElement)
    
    
    # Use Canny edge detection to get lines
    cv.Canny(edges, edges, 400, 400)
    """
    cv.Dilate(edges, edges,
              cv.CreateStructuringElementEx(3, 3, 0, 0, 
                                            cv.CV_SHAPE_RECT))
    cv.Erode(edges, edges,
              cv.CreateStructuringElementEx(1, 1, 0, 0, 
                                            cv.CV_SHAPE_RECT))
    """

    line_storage = cv.CreateMemStorage()

    probabilistic = False
    if probabilistic:     # Probabilistic
        lines = cv.HoughLines2(edges, line_storage, cv.CV_HOUGH_PROBABILISTIC,
                               1, cv.CV_PI/180.0, 120, 50, 40)
        print len(lines), 'lines found'
        for i in range(len(lines)):
            line = lines[i]
            cv.Line(frame_small, line[0], line[1],
                    hv2rgb(360.0*i/len(lines), 1.0), 3, 8)

    else:        # Classic
        lines = cv.HoughLines2(edges, line_storage, cv.CV_HOUGH_STANDARD,
                               1, cv.CV_PI/180.0, 120)

        # Group lines that are within r +/-10 and theta +/- 2 degrees
        lines = list(lines)
        grouped_lines = []
        while len(lines) > 0:
            line1 = normalize_line(lines.pop())
            avg_line = line1
            matched_lines = [line1]
            for j in range(len(lines)-1, -1, -1):
                line2 = normalize_line(lines[j])
                if (abs(avg_line[0] - line2[0]) < 12):
                    print 1,
                if (abs(avg_line[1] - line2[1]) < cv.CV_PI*2/180 or
                     abs(avg_line[1] - line2[1]) > cv.CV_PI * (2 - 2.0/180)): 
                    print 2,
                print avg_line, line2
                if (abs(avg_line[0] - line2[0]) < 12 and
                    # Different signs: handle discontinuity between 0 and 2pi
                    (abs(avg_line[1] - line2[1]) < cv.CV_PI*2/180 or
                     abs(avg_line[1] - line2[1]) > cv.CV_PI * (2 - 2.0/180))):
                    matched_lines.append(line2)
                    avg_line = avg_lines(matched_lines)
                    lines.pop(j)
            print matched_lines
            
            grouped_lines.append(avg_line)
        lines = grouped_lines
        
        # Print lines
        for i in range(len(lines)):
            print lines[i]
            rho, theta = lines[i]
            a, b = math.cos(theta), math.sin(theta)
            x0, y0 = a*rho, b*rho
            pt1 = (cv.Round(x0 + 1000*(-b)),
                   cv.Round(y0 + 1000*(a)))
            pt2 = (cv.Round(x0 - 1000*(-b)),
                   cv.Round(y0 - 1000*(a)));
            cv.Line(frame_small, pt1, pt2,
                    hv2rgb(360.0*i/len(lines), 1.0), 1, 8)

        # Pair up lines by smallest angle difference
        lines = list(lines)
        grouped_lines = []
        while len(lines) > 0:
            line1 = normalize_line(lines.pop())
            closest = None
            for j in range(len(lines)-1, -1, -1):
                line2 = normalize_line(lines[j])
                # Find the closest match
                if ((closest is None
                     or abs(angle_difference(line1[1], line2[1])) < \
                         abs(angle_difference(line1[1], closest[1])))
                    # Make sure difference < pi/4 to reduce errors
                    and abs(angle_difference(line1[1], line2[1])) < \
                        cv.CV_PI / 4):
                    closest = line2
            if closest is not None:
                lines.remove(closest)
                # Sort list by line[0] (radius)
                if line1[0] > closest[0]:
                    line1, closest = closest, line1
            # Make a tuple (line1, line2) or (line, None) if no match found
            grouped_lines.append((line1, closest))

        print 'Pairs of lines:', grouped_lines

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
        if k == ord('q'):
            break

if __name__ == '__main__':
    main()
