import numpy
import models, util

class ParticleFilter:
  def __init__(self, numParticles=1000):
    self.numParticles = numParticles
    self.particles = util.Counter()
    self.initializeUniformly()
    
  def initializeUniformly(self):
    # Uniform distribution over one half of the tennis court
    # TODO Handle off court too?
    locations = sample_uniform(self.numParticles, 0, 1189, 0, 1097)
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
        dx, dy = numpy.random.normal(0, 10.0, 2)
        dtheta = numpy.random.normal(0, 0.5)
        
        # Account for motion
        if motion is not None:
          dx += motion[0]
          dy += motion[1]
          dtheta += motion[2]
        
        newParticle = (particle[0] + dx, particle[1] + dy, particle[2] + dtheta)
        self.particles[newParticle] += 1.0
  
  def getBeliefs(self):
    return self.particles
  
def sample_uniform(n, xmin, xmax, ymin, ymax):
  x = (xmax - xmin) * numpy.random.random_sample((n,)) + xmin
  y = (ymax - ymin) * numpy.random.random_sample((n,)) + ymin
  # theta = heading from [-pi,pi]
  t = 2*numpy.pi * numpy.random.random_sample((n,)) - numpy.pi
  
  # Return n tuples of (x,y) pairs
  return [(x[i], y[i], t[i]) for i in xrange(n)]
