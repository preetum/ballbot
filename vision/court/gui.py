import random, time, thread, sys
import math
import cv
from findlines import *
from Tkinter import *

import models, util
from filter import *

class Simulator:

  def __init__(self, particleFilter=None):
    self.pf = particleFilter

    self.width = 800
    self.height = 600
    self.scale = 0.5
    self.border = 20
  
    self.root = Tk()
    self.root.title('Ballbot Simulator')
    self.canvas = Canvas(self.root, width=self.width, height=self.height)
    self.canvas.pack()
    
    self.draw_field()
  
  def draw_robot(self, x0, y0, t=None):
    '''
    Draw a robot at (x0, y0) with optional heading t
    x, y given in centimeters, t given in radians
    '''
    x, y = self.transform(x0, y0)
    self.canvas.create_oval(x-4, y-4, x+4, y+4, outline='orange')
    if t is not None:
      # t represents the angle of the robot frame's x-axis;
      #  draw the y-axis representing the robot's heading
      t = t + math.pi/2
      self.draw_line(x0, y0, x0+12*math.cos(t), y0+12*math.sin(t), 
                     fill='orange')
  
  def refresh(self, beliefs=None):
    '''
    Refresh the field, given a current list of robots
    '''
    if beliefs is None:
      beliefs = self.pf.getBeliefs()

    self.draw_field()
    for x, y, t in beliefs:
      self.draw_robot(x, y, t)
  
  def transform(self, x, y):
    '''
    Flips the coordinate system so that (0,0) is in the bottom left.
    Adds a border and scales the system
    '''
    return (self.border + (x*self.scale), self.height - self.border - (y*self.scale));

  def inverse_transform(self, x, y):
    '''
    Transforms pixel location to simulation coordinate system
    '''
    return (x - self.border) / self.scale, \
        - (y + self.border - self.height) / self.scale
  
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


def update_loop(capture, pf, sim):
  time.sleep(1)
  # Main logic loop
  i = 10
  while True:
    frame = cv.QueryFrame(capture)

    line_segments, corners = find_lines(frame)
    for segment in line_segments:
      pf.observeLine(segment)
    for corner in corners:
      pf.observeCorner(corner)

    pf.resample()

    # Redraw beliefs
    i += 1
    if i > 5:
      i = 0
      sim.refresh()

    # Remove some fraction the particles and replace with random samples
    j = 0
    # Generate numParticles random locations quickly using numpy
    locations = sampleUniform(pf.numParticles, 0, 1189, 0, 1097)
    # Generate numParticles booleans with 1/8 probability
    for b in np.random.randint(0,8,pf.numParticles) == 0:
      if b:
        pf.particles.particles[j] = locations[j]
      j += 1
    pf.elapseTime()

    cv.WaitKey(10)

# For testing: replaces 16 random particles with 16 particles
#  at the click location, in different orientations
def click_callback(event, sim):
  x, y = sim.inverse_transform(event.x, event.y)
  print 'click', x, y
  start = np.random.randint(sim.pf.numParticles)
  for i in range(16):
    sim.pf.particles.particles[start+i] = np.array([x,y,i*np.pi/8])
  sim.refresh()

def main():
  pf = ParticleFilter(numParticles=500)
  sim = Simulator(pf)

  # For testing
  sim.canvas.bind("<Button-1>", lambda evt: click_callback(evt, sim))

  cv.NamedWindow('frame', cv.CV_WINDOW_AUTOSIZE)
  cv.MoveWindow('frame', 10, 10)
  cv.NamedWindow('edges', cv.CV_WINDOW_AUTOSIZE)
  cv.MoveWindow('edges', 600, 10)

  capture = cv.CaptureFromFile(sys.argv[1])
  if capture is None:
    print 'Error opening file'
    return
#  for i in range(2200):
#    cv.GrabFrame(capture)
  
  thread.start_new_thread(update_loop, (capture, pf, sim))

  # Main Tk loop
  sim.root.mainloop()


if __name__ == '__main__':
  main()
