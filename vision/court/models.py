import numpy as np
import util
from findlines import *

corners = [(640, 550), (640, 137), (640, 960), (1189, 137), (1189, 960)]
lines = [
  # Field boundaries
  ((0, 0), (1189, 0)),
  ((0, 1097), (1189, 1097)),
  ((1189, 0), (1189, 1097)),
  # Singles lines
  ((0, 137), (1189, 137)),
  ((0, 960), (1189, 960)),
  # Center lines
  ((0, 1097/2.0), (640, 1097/2.0)),
  ((640, 137), (640, 960)),
  ]
camera_params = {'height': 33.5,  # in cm
  'offset_x': -4.5,  # offset from center of front axle, in cm
  'offset_y': -10,
  'tilt':  None,   # not sure about this one
  'resolution': (640, 480),
  'radians_per_px': 0.0016
  }

def cameraPointToXY(p):
  px, py = p

  # TODO get these parameters from rosparam store
  radians_per_px = 0.0016
  frame_height = 480
  frame_width = 640
  camera_tilt_angle = -20.0/180*np.pi
  camera_pan_angle = 0.0
  camera_height = 33.5

  theta = (py - frame_height/2) * radians_per_px - camera_tilt_angle
  y = camera_height / np.tan(theta);

  phi = (px - frame_width/2) * radians_per_px + camera_pan_angle
  x = y * np.tan(phi)

  return x, y

def distHeadingToLine(line):
  '''
  Returns real distance reading to the line, where line is a camera line
  line = (x1,y1), (x2,y2)
  '''
  # Get two points on the (camera) line
  (x1, y1), (x2, y2) = line

  # Convert to points in real space
  x1, y1 = cameraPointToXY((x1, y1))
  x2, y2 = cameraPointToXY((x2, y2))

  return util.pointLineVector((0,0), (x1,y1), (x2,y2))

def distHeadingToPoint(pt):
  '''
  Returns real distance and heading to the point, where pt is a camera point
  '''
  x, y = pt
  x, y = cameraPointToXY((x, y))
  r, theta = np.linalg.norm((x, y)), np.arctan2(y, x)
  return r, theta

def cornerProbabilityGivenParticleLocation(observation, particles):
  '''
  Calculates P(e|x_t), given that observation is a point in the 
  robot coordinate frame

  observation is a single observation
  particles is a 2D numpy array of particles:
  [[x0, y0, theta0],
   [x1, y1, theta1], 
   etc...]
  '''
  obs_dist, obs_heading = distHeadingToPoint(observation)
  print 'Corner: %f cm\t%f deg' % (obs_dist, obs_heading*180/np.pi)
  
  probs = np.zeros(len(particles))
  for corner in corners:
    # calculate distance from each particle to corner
    corner = np.array(corner)
    distanceVectors = corner - particles[:, 0:2]
    distances = np.array(map(np.linalg.norm, distanceVectors))

    # calculate the relative heading w.r.t particle position and heading
    headings = np.arctan2(distanceVectors[:,1], distanceVectors[:,0])
    headings = util.normalizeRadians(headings - (particles[:,2]))
    
    # TODO tune sigmas
    # (assume P(e|x_t) ~ exp{-1/2 * |distance - obs_dist| / sigma_1^2} 
    #                    * exp{-1/2 * |heading - obs_heading| / sigma_2^2} )
    probs = probs + np.exp(-0.005 * np.abs(distances - obs_dist) +
                            -7 * np.abs(headings - obs_heading))
    '''
    if corner is corners[0]:
      print particle, 'hdg:', heading
      print np.exp(-0.1 * np.abs(distance - obs_dist) +
        -10 * np.abs(heading - obs_heading))
      print np.exp(-1 * np.abs(heading - obs_heading))
    '''
  
  return probs

def transform(point, transforms):
  '''
  Transforms a 2D point into the coordinate system(s) described by
   translation and angle.

  point is a 2D vector (list, tuple or ndarray)
  transforms is either a 2D vector in the form [x, y, theta]
   (where x,y represents a translation and theta represents rotation)
   or it may be a list of 2D vectors:
   [[x1,y1,theta], [x2,y2,theta], ...]

  translation (x,y) represents the origin of the new coordinate system in the 
   old coordinate system
  angle (theta) represents the rotation of the new coordinate system w.r.t.
   the old coordinate system
  '''
  p = np.array(point)
  T = np.array(transforms)

  # Handle single-transform case
  if np.ndim(T) == 1: T = np.array([T])

  angles = T[:,2]
  A = np.cos(angles)
  B = np.sin(angles)
  tmp = p - T[:,0:2]

  # new x = (p-translate)*cos(angles) + (p-translate)*sin(angles)
  # new y = (p-translate)*-sin(angles) + (p-translate)*cos(angles)
  X = np.sum(tmp * np.array([A, B]).T, axis=1)
  Y = np.sum(tmp * np.array([-B, A]).T, axis=1)
  newPts = np.array([X, Y]).T

  # Return answer with same dimensions as transforms
  if np.ndim(transforms) == 1:
    return newPts[0]
  else:
    return newPts

def lineProbabilityGivenParticleLocation(observation, particles):
  '''
  observation are the endpoints of the line segment
  '''
  obs_dist, obs_heading = distHeadingToLine(observation)
  print 'Line: %f cm\t %f deg' % (obs_dist, obs_heading*180/np.pi)

  pt1, pt2 = observation
  pt1 = cameraPointToXY(pt1)
  pt2 = cameraPointToXY(pt2)
  print 'Line segment: %s, %s' % (pt1, pt2)

  probs = np.zeros(len(particles))
  for line in lines:
    # Get the distance, absolute heading from particle to the line
    dists, headings = util.pointLineVector(particles[:,0:2], 
                                           line[0], line[1])
    # Adjust heading to account for the heading of the robot
    headings = util.normalizeRadians(headings - (particles[:,2]))

    # Use new distance metric for line segments
    # Convert candidate line into robot coordinate frame
    lineA = transform(line[0], particles)
    lineB = transform(line[1], particles)

    # Take each endpoint of the observed line
    #  and calculate its distance to the candidate line
    #  (both in the robot's coordinate frame)
    dist_metric = (util.pointLineSegmentDistance(pt1, lineA, lineB) + 
                   util.pointLineSegmentDistance(pt2, lineA, lineB))

    # Scale distance metric by heading of line
    heading_error = np.abs(util.normalizeRadians(obs_heading - headings))
    dist_metric = dist_metric * \
        np.exp(3*heading_error)

    probs = probs + np.exp(-0.005 * np.abs(dist_metric) -7 * heading_error)

  return probs
