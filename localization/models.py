import numpy
import util

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
  'tilt':  1.228,   # in radians
  'resolution': (640, 480),
  'radians_per_px': 0.0016
  }


def cornerProbabilityGivenParticleLocation(observation, particle):
  '''
  Calculates P(e|x_t), given that observation is the (scalar) distance
  to the corner
  TODO: r, theta = observation
  '''
  obs_dist, obs_heading = observation
  
  prob = 0.0
  for corner in corners:
    # calculate distance to corner
    distance = util.distance(particle[0:2], corner)
    # calculate the relative heading w.r.t particle position and heading
    heading = util.normalizeRadians(util.heading(particle[0:2], corner) - particle[2])
    
    # TODO tune sigmas
    # (assume P(e|x_t) ~ exp{-1/2 * |distance - obs_dist| / sigma_1^2} 
    #                    * exp{-1/2 * |heading - obs_heading| / sigma_2^2} )
    prob += numpy.exp(-0.1 * numpy.abs(distance - obs_dist) +
      -7 * numpy.abs(heading - obs_heading))
    '''
    if corner is corners[0]:
      print particle, 'hdg:', heading
      print numpy.exp(-0.1 * numpy.abs(distance - obs_dist) +
        -10 * numpy.abs(heading - obs_heading))
      print numpy.exp(-1 * numpy.abs(heading - obs_heading))
    '''
  
  return prob

def lineProbabilityGivenParticleLocation(observation, particle):
  obs_dist, obs_heading = observation
  prob = 0.0
  for line in lines:
    dist = util.pointLineDistance(particle[0:2], line)
    prob += numpy.exp(-0.1 * numpy.abs(dist - obs_dist))
  return prob
