import random, time, thread, sys
import math
import cv
from findlines import *
from Tkinter import *

import models, util
from filter import ParticleFilter

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
      self.draw_line(x0, y0, x0+12*math.cos(t), y0+12*math.sin(t), fill='orange')
  
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
    
    pf.elapseTime()

    # Redraw beliefs
    i += 1
    if i > 10:
      i = 0
      sim.refresh()

    cv.WaitKey(10)

def main():
  pf = ParticleFilter(numParticles=2000)
  sim = Simulator(pf)

  cv.NamedWindow('frame', cv.CV_WINDOW_AUTOSIZE)
  cv.MoveWindow('frame', 10, 10)
  cv.NamedWindow('edges', cv.CV_WINDOW_AUTOSIZE)
  cv.MoveWindow('edges', 600, 10)

  capture = cv.CaptureFromFile(sys.argv[1])
  if capture is None:
    print 'Error opening file'
    return
  for i in range(2000):
    cv.GrabFrame(capture)
  
  thread.start_new_thread(update_loop, (capture, pf, sim))

  # Main Tk loop
  sim.root.mainloop()


if __name__ == '__main__':
  main()
