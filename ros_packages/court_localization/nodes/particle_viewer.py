#!/usr/bin/env python
#import roslib; roslib.load_manifest('court_localization')
#import rospy
#from bb_msgs.msg import PoseArray

import random, time, thread, sys
import numpy as np
from Tkinter import *

class Simulator:

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
  
  def refresh(self, beliefs):
    '''
    Refresh the field, given a current list of robots:
    x, y is the position in centimeters
    t is the heading
    '''
    self.draw_field()
    beliefs = np.array(beliefs)

    points = self.transform(beliefs[:,0:2])
    angles = beliefs[:,2] - np.pi/2
    A = np.cos(angles)
    B = np.sin(angles)
    R = np.array([[A, B], [-B, A]])

    lowerLeft = points + np.dot([-4,4], R).T
    center = points + np.dot([0,-4], R).T
    lowerRight = points + np.dot([4,4], R).T

    for l, c, r in zip(lowerLeft, center, lowerRight):
      self.canvas.create_line(l[0], l[1], c[0], c[1], fill='blue')
      self.canvas.create_line(r[0], r[1], c[0], c[1], fill='blue')

  def transform(self, points):
    '''
    Flips the coordinate system so that (0,0) is in the bottom left.
    Adds a border and scales the system
    '''
    points = np.array(points)

    # x' = self.border + (x*self.scale)
    # y' = self.height - self.border - (y*self.scale)
    return np.array([self.scale, -self.scale]) * points + \
        np.array([self.border, self.height-self.border])
  
  def draw_line(self, line, *args, **kwargs):
    '''
    Helper for draw_field
    '''
    (x1, y1), (x2, y2) = self.transform(line)
    self.canvas.create_line(x1, y1, x2, y2, *args, **kwargs)

  def draw_field(self):
    '''
    Redraws the field (use it to clear the simulation)
    '''
    self.canvas.create_rectangle(0, 0, self.width, self.height, fill='#CCFF99')
    
    # Field boundaries (dimensions in cm)
    for line in Simulator.lines:
      self.draw_line(line, width=5, fill='white')
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
    self.draw_line(((0, 0), (0, 1097)), width=4, fill='black')
    
    # Center tick
    self.draw_line(((1150, 1097/2.0), (1189, 1097/2.0)), width=5, fill='white')



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
  #sim.refresh(((500, 500, 0), (600, 500, np.pi/4), (500, 600, -3*np.pi/4)))

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