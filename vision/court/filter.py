import numpy as np
import models, util

def sampleUniform(n, xmin, xmax, ymin, ymax):
  x = (xmax - xmin) * np.random.random_sample((n,)) + xmin
  y = (ymax - ymin) * np.random.random_sample((n,)) + ymin
  # theta = heading from [-pi,pi]
  t = 2*np.pi * np.random.random_sample((n,)) - np.pi
  
  # Return n tuples of (x,y) pairs
  return np.array([x, y, t]).T

class Particles:
  '''
  Provides convenience methods over a set of particles
  self.particles is a 2D numpy array. That is, each particle 
   is a 1D numpy array
  self.weights is a 1D numpy array
  '''
  def __init__(self, samples):
    '''
    samples should be a 2D numpy array of the following format:
    [[x1, y1, ...],
     [x2, y2, ...],
      etc... ]
    '''
    self.initializeUniformly(samples)

  def initializeUniformly(self, samples):
    count = len(samples)
    self.particles = np.array(samples)
    # Assign each particle a uniform weight
    self.weights = np.empty(count)
    if count > 0:
      np.ndarray.fill(self.weights, 1.0/count)

  def __len__(self):
    return len(self.particles)

  def normalize(self):
    total = float(np.sum(self.weights))
    self.weights = self.weights / total

  def sortedParticles(self):
    '''
    Returns particles by sorted by weight.
    Note that the conversions from list to tuple may be slow.
    '''
    # Append weights to beginning of each particle
    particles = np.concatenate((np.vstack(self.weights), self.particles),
                               axis=1)
    # Sort by first column (weights)
    dtype = [(str(i), float) for i in range(len(particles[0]))]
    particles = np.array([tuple(x) for x in particles], dtype=dtype)
    particles.sort(order='0')
    print particles

    # Return all particles excluding the first column
    particles = np.array([list(x) for x in particles])
    return particles[:, 1:]

  def argmax(self):
    '''
    Returns particle with the highest weight
    '''
    return self.particles[self.weights.argmax()]

  def resample(self):
    '''
    Resamples the particle distribution. This does *not* modify
    self.particles or self.weights in place. All references to
    self.particles and self.weights will become invalidated.
    '''
    self.normalize()

    cumulative_probs = np.add.accumulate(self.weights)
    total = cumulative_probs[-1]
    if total > 1.0001: raise 'sampling error: bad distribution'
    if total < 0.9999: raise 'sampling error: bad distribution'

    samples = []
    for i in range(len(self.particles)):
      uniform_sample = np.random.random_sample()
      if uniform_sample > total: uniform_sample = total
      samples.append(self.particles[
          util.binarySearch(cumulative_probs, uniform_sample)])

    self.initializeUniformly(samples)

class ParticleFilter:
  def __init__(self, numParticles=1000):
    self.numParticles = numParticles

    # Initialize distribution uniformly
    locations = sampleUniform(numParticles, 0, 1189, 0, 1097)
    self.particles = Particles(locations)
    
  def observeLine(self, observation):
    return self.observe(observation, models.lineProbabilityGivenParticleLocation)
  
  def observeCorner(self, observation):
    return self.observe(observation, models.cornerProbabilityGivenParticleLocation)
  
  def observe(self, observation, emissionFn):
    # update beliefs
    # P(X_t | e_1:t, e') ~ P(e'|X_t) * P(X_t | e_1:t)

    weights = emissionFn(observation, self.particles.particles)
    self.particles.weights = self.particles.weights * weights
    '''
    for particle in self.particles.keys():
      # reweight each particle by observations
      self.particles[particle] *= emissionFn(observation, particle)
    '''
    
    # resample observation
    #self.particles.resample()
    '''
    samples = util.sampleMultiple(self.particles, self.numParticles)
    self.particles = util.listToDistribution(samples)
    '''

  def resample(self):
    self.particles.resample()
    
  def elapseTime(self, motion=None):
    oldBeliefs = self.particles.particles.copy()
    newBeliefs = util.Counter()
    
    # Move each particle with some gaussian probability;
    #  generate movements for all particles at once
    # TODO tune sigmas (2nd parameter)
    count = len(self.particles)
    dpos = np.random.normal(0, 30.0, (count, 2))
    dtheta = np.random.normal(0, 0.5, (count, 1))
    delta = np.concatenate([dpos, dtheta], axis=1)
    if motion is not None:
      delta = delta + np.array(motion)
    self.particles.particles = self.particles.particles + delta

    '''
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
    '''
  
  def getBeliefs(self):
    '''
    Returns some iterable object of (x,y,z) triplets
    '''
    return self.particles.particles
