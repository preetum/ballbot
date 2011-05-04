import random, time, thread
import numpy
from Tkinter import *

import models, util
from filter import ParticleFilter

class Simulator:

  def __init__(self, pf):
    self.width = 800
    self.height = 600
    self.scale = 0.5
    self.border = 20
    
    self.pf = pf
  
    self.root = Tk()
    self.root.title('Ballbot Simulator')
    self.canvas = Canvas(self.root, width=self.width, height=self.height)
    self.canvas.pack()
    
    self.draw_field()
  
  def start(self):
    '''
    Spawns the Tk main loop in a new thread
    '''
    thread.start_new_thread(self.root.mainloop, ())
  
  def draw_robot(self, x0, y0, t=None, color='orange'):
    '''
    Draw a robot at (x0, y0) with optional heading t
    x, y given in centimeters, t given in radians
    '''
    x, y = self.transform(x0, y0)
    self.canvas.create_oval(x-4, y-4, x+4, y+4, outline=color)
    if t is not None:
      self.draw_line(x0, y0, x0+12*numpy.cos(t), y0+12*numpy.sin(t), fill=color)
  
  def refresh(self, beliefs=None):
    '''
    Refresh the field, given a current list of robots
    '''
    
    self.draw_field()
    
    # Draw the beliefs and estimate from self.pf
    if beliefs is None:
      x, y, t = self.pf.getEstimate()
      self.draw_robot(x, y, t, color='red')
      beliefs = self.pf.getBeliefs()
    # Draw only the beliefs given
    
    for x, y, t in beliefs:
      self.draw_robot(x, y, t)
  
  def transform(self, x, y):
    '''
    Flips the coordinate system so that (0,0) is in the bottom left.
    Adds a border and scales the system
    '''
    return (self.border + (x*self.scale), self.height - self.border - (y*self.scale));
  
  def draw_line(self, x1, y1, x2, y2, *args, **kwargs):
    '''
    Helper for draw_field
    '''
    x1, y1 = self.transform(x1, y1)
    x2, y2 = self.transform(x2, y2)
    self.canvas.create_line(x1, y1, x2, y2, *args, **kwargs)

  def draw_field(self):
    '''
    Redraws the field (use it to clear the simulation)
    '''
    self.canvas.create_rectangle(0, 0, self.width, self.height, fill='dark green')
    
    # Field boundaries (dimensions in cm)
    for line in models.lines:
      (x1, y1), (x2, y2) = line
      self.draw_line(x1, y1, x2, y2, width=5, fill='white')
    '''
    self.draw_line(0, 0, 1189, 0, width=5, fill='white')
    self.draw_line(0, 1097, 1189, 1097, width=5, fill='white')
    self.draw_line(1189, 0, 1189, 1097, width=5, fill='white')
    
    # Singles lines
    self.draw_line(0, 137, 1189, 137, width=5, fill='white')
    self.draw_line(0, 960, 1189, 960, width=5, fill='white')
    
    # Center lines
    self.draw_line(0, 1097/2.0, 640, 1097/2.0, width=5, fill='white')
    self.draw_line(640, 137, 640, 960, width=5, fill='white')
    '''
    # Net
    self.draw_line(0, 0, 0, 1097, width=4, fill='black')
    
    # Center tick
    self.draw_line(1150, 1097/2.0, 1189, 1097/2.0, width=5, fill='white')
    
    
def main():
  pf = ParticleFilter(numParticles=2000)
  sim = Simulator(pf)
  sim.start()
  '''
  pf.particles = util.Counter()
  pf.numParticles = 8
  pf.particles[(640, 500, 0)] = 1.0
  pf.particles[(640, 500, -numpy.pi/2)] = 1.0
  pf.particles[(640, 500, numpy.pi/2)] = 1.0
  pf.particles[(640, 500, numpy.pi)] = 1.0
  pf.particles[(640, 600, 0)] = 1.0
  pf.particles[(640, 600, -numpy.pi/2)] = 1.0
  pf.particles[(640, 600, numpy.pi/2)] = 1.0
  pf.particles[(640, 600, numpy.pi)] = 1.0
  '''
  sim.refresh()
  
  time.sleep(2)
  
  
  # Observed lines at pixels
  '''
  lines = [
    ((0, 117), (639, 125)),
    ((11, 116), (639, 123)),
    ((0, 454), (639, 430)),
    ((26, 452), (638, 429)),
    ]
  
  for line in lines:
    p1 = util.pixelToDistance(line[0])
    p2 = util.pixelToDistance(line[1])
    dist = util.pointLineDistance((0, 0), (p1, p2))
    print line
    print 'to', (p1,p2), 'at distance', dist
    pf.observeLine((dist, 0))
    sim.refresh(pf.getBeliefs())
    time.sleep(1)
  '''
  # Observation is a distance and heading to an unidentified line
  pf.observeLine((10, 0))
  pf.observeLine((150, 0))
  sim.refresh()
  time.sleep(2)
  
  # Observation is a distance and heading to an unidentified corner
  pf.observeCorner((150, 0))
  sim.refresh()
  time.sleep(2)
  
  pf.elapseTime()
  sim.refresh()
  
  # Main logic loop
  dist = 150
  while True:
    pf.observeLine((dist, 0))
    sim.refresh()
    pf.elapseTime((0, 30, 0))
    dist -= 30

if __name__ == '__main__':
  main()
