#!/usr/bin/env python
import roslib; roslib.load_manifest('court_localization')
import rospy

from bb_msgs.msg import PoseArray

import random, time, thread, sys
import math
from findlines import *
from Tkinter import *

import models

class Simulator:

  def __init__(self):
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



# On click, refresh the simulation
def click_callback(event, sim):
  sim.refresh()

def msg_callback(msg, sim):
  sim.refresh(msg.data)

def main():
  # Get config
  rate = float(rospy.get_param('~refresh_rate', 1))
  topic_name = rospy.get_param('~topic', 'filter/particles')

  sim = Simulator()

  # Initialize ROS listener
  rospy.init_node('particle_viewer')
  rospy.Subscriber(topic_name, PoseArray, lambda msg: msg_callback(msg, sim))

  # Start rospy spin thread
  thread.start_new_thread(rospy.spin, ())

  # Start main Tk loop
  sim.canvas.bind('<Button-1>', lambda evt: click_callback(evt, sim))
  sim.root.mainloop()


if __name__ == '__main__':
  main()
