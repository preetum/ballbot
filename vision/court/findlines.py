import sys
import numpy as np
import cv
from optparse import OptionParser

verbose = False

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
        theta += np.pi
    return (r, theta % (2*np.pi))

def avg_lines(lines):
    '''
    Computes some average of the lines (given as (r, theta) pairs)
    Returns average (r, theta)
    '''
    r = sum(x[0] for x in lines) / len(lines)
    theta = np.arctan2(sum(np.sin(x[1]) for x in lines),
                       sum(np.cos(x[1]) for x in lines))
    return normalize_line((r, theta))

def angle_difference(a, b):
    '''
    Computes the angle difference (a - b), normalized to [-pi, pi)
    '''
    diff = a - b
    return (diff + np.pi) % (2*np.pi) - np.pi

def to_xy(r_theta):
    '''
    Converts (r, theta) to (x, y)
    '''
    r, theta = r_theta
    return (r * np.cos(theta), r * np.sin(theta))

def line_intersection(line1, line2):
    '''
    Finds the intersection point of two 2D lines in (r, theta) form,
     where the line is parallel to the vector denoted by (r, theta)
    Returns intersection point as (x, y)
    See http://mathworld.wolfram.com/Line-LineIntersection.html
    '''
    x1, y1 = to_xy(line1)
    x2, y2 = x1-y1, x1+y1
    x3, y3 = to_xy(line2)
    x4, y4 = x3-y3, x3+y3

    print x1, y1
    print x3, y3

    a = np.linalg.det([[x1, y1], [x2, y2]])
    b = np.linalg.det([[x3, y3], [x4, y4]])
    d = np.linalg.det([[x1-x2, y1-y2], [x3-x4, y3-y4]])
    x = np.linalg.det([[a, x1-x2], [b, x3-x4]]) / d
    y = np.linalg.det([[a, y1-y2], [b, y3-y4]]) / d

    print 'center of', line1, line2, 'is', (x,y)

    return (x, y)

def find_lines(frame):
    # Resize to 640x480
    frame_size = cv.GetSize(frame)
    if frame_size[0] != 640:
      frame_small = cv.CreateMat(480, 640, cv.CV_8UC3)
      cv.Resize(frame, frame_small)
    else:
      frame_small = frame

    # Threshold by distance: blank out all top pixels
    cv.Rectangle(frame_small, (0,0), (640, 80), (0,0,0,0), cv.CV_FILLED)

    # Convert to grayscale
    frame_size = cv.GetSize(frame_small)
    frame_gray = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 1)
    cv.CvtColor(frame_small, frame_gray, cv.CV_BGR2GRAY)

    # Use color thresholding to get white lines
    threshold = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 1)
    cv.Threshold(frame_gray, threshold, 190, 255, cv.CV_THRESH_BINARY)

    # Morphological ops to reduce noise
    # TODO try to reduce sizes to increase detection of faraway lines
    openElement = cv.CreateStructuringElementEx(7, 7, 3, 3,
                                                cv.CV_SHAPE_RECT)
    closeElement = cv.CreateStructuringElementEx(11, 11, 5, 5,
                                                 cv.CV_SHAPE_RECT)
    cvOpen(threshold, threshold, openElement)
    cvClose(threshold, threshold, closeElement)
    
    # Use Canny edge detection to find edges
    edges = cv.CreateImage(frame_size, cv.IPL_DEPTH_8U, 1)
    cv.Canny(threshold, edges, 100, 200)

    # Use Hough transform to find equations for lines
    line_storage = cv.CreateMemStorage()

    lines = cv.HoughLines2(edges, line_storage, cv.CV_HOUGH_STANDARD,
                               1, cv.CV_PI/180.0, 120)
    lines = list(lines)

    # Remove spurious line from the black rectangle up top
    for line in lines[:]:
      if (abs(180 - line[0]) < 10 and
          abs(angle_difference(cv.CV_PI/2, line[1])) < 0.01):
        lines.remove(line)

    # Group lines that are within r +/-12 and theta +/- 5 degrees
    grouped_lines = []
    r_threshold = 12                      # in px
    theta_threshold = cv.CV_PI * 5 / 180  # in radians
    while len(lines) > 0:
        line1 = normalize_line(lines.pop())
        avg_line = line1
        matched_lines = [line1]
        for j in range(len(lines)-1, -1, -1):
            line2 = normalize_line(lines[j])
            if verbose:
              # Print which criteria were matched
              if (abs(avg_line[0] - line2[0]) < r_threshold):
                print 1,
              if (abs(angle_difference(avg_line[1], line2[1])) < 
                  theta_threshold):
                print 2,
              print avg_line, line2
            if (abs(avg_line[0] - line2[0]) < r_threshold and
                abs(angle_difference(avg_line[1],line2[1])) < theta_threshold):
                matched_lines.append(line2)
                avg_line = avg_lines(matched_lines)
                lines.pop(j)
        if verbose: print matched_lines
        grouped_lines.append(avg_line)
    lines = grouped_lines

    # Group possible pairs of lines by smallest angle difference
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
            # Make a tuple (line1, line2) or (line,) if no match found
            grouped_lines.append((line1, closest))
        else:
          grouped_lines.append((line1,))

    # Print lines
    if len(grouped_lines) > 0:
      print 'Groups of lines:', grouped_lines
    i = 0
    for group in grouped_lines:
      for j in range(len(group)):
        rho, theta = group[j]
        a, b = np.cos(theta), np.sin(theta)
        x0, y0 = a*rho, b*rho
        pt1 = (cv.Round(x0 + 1000*(-b)),
               cv.Round(y0 + 1000*(a)))
        pt2 = (cv.Round(x0 - 1000*(-b)),
               cv.Round(y0 - 1000*(a)));
        cv.Line(frame_small, pt1, pt2,
                hv2rgb(360.0*i/len(grouped_lines), 1.0), 1, 8)
      i += 1

    # If 2+ groups of lines, find corners (intersection point of lines)
    if len(grouped_lines) > 1:
      for i in range(len(grouped_lines)):
        pair1 = grouped_lines[i]
        for j in range(i+1, len(grouped_lines)): 
          pair2 = grouped_lines[j]

          # Make sure their angles differ by more than 10 deg to
          #  reduce errors
          if (abs(angle_difference(pair1[0][1], pair2[0][1])) < 
              cv.CV_PI*10/180):
            break

          # Enumerate intersection points
          pts = []
          for line1 in pair1:
            for line2 in pair2:
              pts.append(line_intersection(line1, line2))
          # Find average of intersection points
          x = sum(pt[0] for pt in pts)/len(pts)
          y = sum(pt[1] for pt in pts)/len(pts)
          print 'Intersection:', (x, y),
          pt = cv.Round(x), cv.Round(y)
          cv.Circle(frame_small, pt, 4, (0,255,0,0))

          # Find direction of intersection by following each line
          angles = []
          for pair in grouped_lines:
            angles.append(pair[0][1] + cv.CV_PI/2)
            angles.append(pair[0][1] - cv.CV_PI/2)
          for angle in angles:
            # Look 50px down the line for white pixels
            # TODO look a variable amount
            x1 = x + 50*np.cos(angle)
            y1 = y + 50*np.sin(angle)
            # Enforce limits
            x1 = min(max(0, x1), frame_size[0]-1)
            y1 = min(max(0, y1), frame_size[1]-1)
            srchPt = cv.Round(x1), cv.Round(y1)

            if threshold[srchPt[1], srchPt[0]] == 0:
              x1 = x + 50*np.cos(angle + cv.CV_PI)
              y1 = y + 50*np.sin(angle + cv.CV_PI)
              invSrchPt = cv.Round(x1), cv.Round(y1)
              cv.Line(frame_small, pt, invSrchPt,
                      (0,255,0,0), 1, 8)
              print 'Angle:', angle+cv.CV_PI
              break

    cv.ShowImage('frame', frame_small)
    cv.ShowImage('edges', threshold)

def main():
    cv.NamedWindow('frame', cv.CV_WINDOW_AUTOSIZE)
    cv.MoveWindow('frame', 10, 10)
    cv.NamedWindow('edges', cv.CV_WINDOW_AUTOSIZE)
    cv.MoveWindow('edges', 600, 10)

    parser = OptionParser()
    parser.add_option('-s', '--seek', action='store', type='int', dest='seek',
                      default=0)
    parser.add_option('-i', '--image', action='store_true', dest='image',
                      default=False)
    parser.add_option('-v', '--verbose', action='store_true', dest='verbose',
                      default=False)
    options, args = parser.parse_args()

    global verbose
    verbose = options.verbose

    if options.image:
        # Use image
        frame = cv.LoadImage(args[0])
        if frame is None:
          print 'Error loading image %s' % args[0]
          return
        find_lines(frame)

        # Pause for key press
        while True:
          k = cv.WaitKey(33)
          if k == ord('q'):
            break
    else:
      if len(args) > 0:
        # Use video
        cam = cv.CaptureFromFile(args[0])
        if cam is None:
          print 'Error opening file'
          return
      else:
        # Use webcam
        cam = cv.CreateCameraCapture(1);
        if cam is None:
          print 'Error opening camera'
          return

      # Seek to initial position by discarding frames
      for i in range(options.seek):
        cv.GrabFrame(cam)

      while True:
        frame = cv.QueryFrame(cam)
        if frame is None:
          print 'End of video'
          break
        find_lines(frame)
        
        k = cv.WaitKey(10)
        if k == ord('q'):
          break
      cv.ReleaseCapture(cam)


if __name__ == '__main__':
    main()
