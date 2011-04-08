import numpy as np
import models, util

class LinearSystem:
  def __init__(self, C, R, Q):
    '''
    Models the system
    x[k] = A[k] * x[k-1] + B[k] * u[k] + transition noise
    z[k] = C[k] * x[k] + measurement noise
    
    R is the covariance of transition noise
    Q is the covariance of measurement noise
    '''
    self.C = np.array(C)
    self.R = np.array(R)
    self.Q = np.array(Q)
  def A(self, x):
    raise NotImplementedError()
  def B(self, x):
    raise NotImplementedError()

class KalmanFilter:
  def __init__(self, system, state, covariance):
    self.system = system
    self.x = np.array(state)
    self.sigma = np.array(covariance)

  def elapseTime(self, control):
    A = np.array(self.system.A(self.x))
    B = np.array(self.system.B(self.x))
    u = np.array(control)

    # Calculate new mean and covariance
    x = array_multiply(A, self.x) + array_multiply(B, u)
    sigma = array_multiply(A, self.sigma, A.T) + self.system.R

    # Update belief
    self.x = x
    self.sigma = sigma

  def observe(self, observation):
    z = np.matrix(observation).T
    C = np.matrix(self.system.C)
    Q = np.matrix(self.system.Q)
    x = np.matrix(self.x).T
    sigma = np.matrix(self.sigma)

    # Calculate Kalman gain and new state
    K = sigma * C.T * (C*sigma*C.T + Q).I
    x = x + K * (z - C * x)
    sigma = (np.eye(3) - K*C) * sigma

    # Update belief
    self.x = np.array(x.T)[0]
    self.sigma = np.array(sigma)

class ParticleFilter:
  def __init__(self, numParticles=1000):
    self.numParticles = numParticles
    self.particles = util.Counter()
    self.initializeUniformly()
    
  def initializeUniformly(self):
    # Uniform distribution over one half of the tennis court
    # TODO Handle off court too?
    locations = sample_uniform(self.numParticles, -30, 1219, -30, 1127)
    for loc in locations:
      self.particles[loc] = 1.0
    self.particles.normalize()
    
  def observeLine(self, observation):
    return self.observe(observation, models.lineProbabilityGivenParticleLocation)
  
  def observeCorner(self, observation):
    return self.observe(observation, models.cornerProbabilityGivenParticleLocation)
  
  def observe(self, observation, emissionFn):
    # update beliefs
    # P(X_t | e_1:t, e') ~ P(e'|X_t) * P(X_t | e_1:t)
    for particle in self.particles.keys():
      # reweight each particle by observations
      self.particles[particle] *= emissionFn(observation, particle)
    
    # resample observation
    self.particles.normalize()
    samples = util.sampleMultiple(self.particles, self.numParticles)
    self.particles = util.listToDistribution(samples)
    
  def elapseTime(self, motion=None):
    oldBeliefs = self.particles
    self.particles = util.Counter()
    
    # move each particle with some gaussian probability
    for particle in oldBeliefs.keys():
      for i in range(int(oldBeliefs[particle] + 0.5)):
        # TODO tune sigma (2nd parameter)
        dx, dy = np.random.normal(0, 10.0, 2)
        dtheta = np.random.normal(0, 0.5)
        
        # Account for motion
        if motion is not None:
          dx += motion[0]
          dy += motion[1]
          dtheta += motion[2]
        
        newParticle = (particle[0] + dx, particle[1] + dy, particle[2] + dtheta)
        self.particles[newParticle] += 1.0
  
  def getBeliefs(self):
    return self.particles
  
  def getEstimate(self):
    estimate = np.array((0,0,0))
    totalCount = 0
    for particle in self.particles.keys():
      count = np.array(self.particles[particle])
      totalCount += count
      estimate += count * particle
    estimate /= totalCount
    return tuple(estimate)
  
def sample_uniform(n, xmin, xmax, ymin, ymax):
  x = (xmax - xmin) * np.random.random_sample((n,)) + xmin
  y = (ymax - ymin) * np.random.random_sample((n,)) + ymin
  # theta = heading from [-pi,pi]
  t = 2*np.pi * np.random.random_sample((n,)) - np.pi
  
  # Return n tuples of (x,y) pairs
  return [(x[i], y[i], t[i]) for i in xrange(n)]

def array_multiply(*args):
  return reduce(np.dot, args)
