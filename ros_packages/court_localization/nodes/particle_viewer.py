#!/usr/bin/env python
import roslib; roslib.load_manifest('court_localization')
import rospy
from bb_msgs.msg import PoseArray

import random, time, thread, sys
import numpy as np
from Tkinter import *

class Simulator:

  lines = [
    # Field boundaries
    ((0, 0), (1189, 0)),
    ((0, 1097), (1189, 1097)),
    ((0, 0), (0, 1097)),
    # Singles lines
    ((0, 137), (1189, 137)),
    ((0, 960), (1189, 960)),
    # Center lines
    ((549, 1097/2.0), (1189, 1097/2.0)),
    ((549, 137), (549, 960)),
    ]

  def __init__(self, title='Ballbot Simulator'):
    self.width = 800
    self.height = 600
    self.scale = 0.35
    self.border = 150
  
    self.root = Tk()
    self.root.title(title)
    self.canvas = Canvas(self.root, width=self.width, height=self.height)
    self.canvas.pack()
    
    self.draw_field()
  
  def refresh(self, beliefs, trails=False):
    '''
    Refresh the field, given a current list of robots:
    x, y is the position in centimeters
    t is the heading
    '''
    if not trails:
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
    # Net
    self.draw_line(((1189, 0), (1189, 1097)), width=4, fill='black')    
    # Center tick
    self.draw_line(((0, 1097/2.0), (50, 1097/2.0)), width=5, fill='white')


def msg_callback(msg, sim):
  beliefs = np.array([[p.x, p.y, p.theta] for p in msg.data])
  sim.refresh(beliefs)

def spin_thread(sim):
  # While ROS is alive, wait for messages
  rospy.spin()
  # On exit, kill the mainloop and exit
  sim.root.destroy()

def main():
  # Get config
  topic_name = rospy.get_param('~topic', 'filter/particles')

  sim = Simulator(title='Particles')

  # Initialize ROS listener
  rospy.init_node('particle_viewer', anonymous=True)
  rospy.Subscriber(topic_name, PoseArray, lambda msg: msg_callback(msg, sim),
                   queue_size=1)

  # Start rospy spin thread
  thread.start_new_thread(spin_thread, (sim,))

  # Start main Tk loop
  sim.root.mainloop()


if __name__ == '__main__':
  main()
