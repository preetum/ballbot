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

def camera_point_to_xy(p):
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

def dist_heading_to_line(line):
  '''
  Returns real distance reading to the line, where line is a camera line
  line = (x1,y1), (x2,y2)
  '''
  # Get two points on the (camera) line
  (x1, y1), (x2, y2) = line

  # Convert to points in real space
  x1, y1 = camera_point_to_xy((x1, y1))
  x2, y2 = camera_point_to_xy((x2, y2))

  line = ((x1, y1), (x2, y2))
  return util.pointLineVector((0,0), line)

def dist_heading_to_point(pt):
  '''
  Returns real distance and heading to the point, where pt is a camera point
  '''
  x, y = pt
  x, y = camera_point_to_xy((x, y))
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
  obs_dist, obs_heading = dist_heading_to_point(observation)
  print 'Corner: %f cm\t%f deg' % (obs_dist, obs_heading*180/np.pi)
  
  probs = np.zeros(len(particles))
  for corner in corners:
    # calculate distance from each particle to corner
    corner = np.array(corner)
    distanceVectors = corner - particles[:, 0:2]
    distances = np.array(map(np.linalg.norm, distanceVectors))

    # calculate the relative heading w.r.t particle position and heading
    headings = np.arctan2(distanceVectors[:,1], distanceVectors[:,0])
    headings = util.normalizeRadians(headings - particles[:, 2])
    
    # TODO tune sigmas
    # (assume P(e|x_t) ~ exp{-1/2 * |distance - obs_dist| / sigma_1^2} 
    #                    * exp{-1/2 * |heading - obs_heading| / sigma_2^2} )
    prob += np.exp(-0.1 * np.abs(distances - obs_dist) +
      -7 * np.abs(headings - obs_heading))
    '''
    if corner is corners[0]:
      print particle, 'hdg:', heading
      print np.exp(-0.1 * np.abs(distance - obs_dist) +
        -10 * np.abs(heading - obs_heading))
      print np.exp(-1 * np.abs(heading - obs_heading))
    '''
  
  return probs

def transform(points, translation, angle):
  '''
  Transforms a 2D point into the coordinate system described by
   translation and angle.

  points is either a 2D vector (list, tuple or ndarray), or a numpy array
   of 2D *row* vectors
  translation is a 2D vector representing the origin of the new coordinate
   system in the old coordinate system
  angle is the rotation of the new coordinate system w.r.t. the old
  '''
  p = np.array(points)
  t = np.array(translation)
  a,b = np.cos(angle), np.sin(angle)
  R = np.array([[a, b], [-b, a]])
  newPts = np.dot(R, (p-t).T).T
  return newPts

def lineProbabilityGivenParticleLocation(observation, particles):
  '''
  observation are the endpoints of the line segment
  '''
  pt1, pt2 = observation
  pt1 = camera_point_to_xy(pt1)
  pt2 = camera_point_to_xy(pt2)

  obs_dist, obs_heading = dist_heading_to_line(observation)
  print 'Line: %f cm\t %f deg' % (obs_dist, obs_heading*180/np.pi)

  probs = np.zeros(len(particles))
  i = 0
  for particle in particles:
    for line in lines:
      # Get the distance, absolute heading from particle to the line
      dist, heading = util.pointLineVector(particle[0:2], line)

      # Use new distance metric for line segments
      # Convert candidate line into robot coordinate frame
      new_line = transform(line, particle[0:2], particle[2])

      # Take each endpoint of the observed line
      #  and calculate its distance to the candidate line
      #  (both in the robot's coordinate frame)
      dist_metric = util.pointLineSegmentDistance(pt1, new_line) + \
          util.pointLineSegmentDistance(pt2, new_line)

      # Adjust heading to account for the heading of the robot
      heading = util.normalizeRadians(heading - particle[2])

      probs[i] += np.exp(-0.1 * np.abs(dist_metric) +
                          -7 * np.abs(heading - obs_heading))
    i += 1
  return probs
