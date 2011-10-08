#!/usr/bin/env python
import roslib; roslib.load_manifest('court_localization')
import rospy
from bb_msgs.msg import OdometryStamped
from particle_viewer import Simulator

import random, time, thread, sys, math
import numpy as np

class SingleSimulator(Simulator):

  def refresh(self, belief, trails=False):
    beliefs = np.array([belief])
    points = self.transform(beliefs[:,0:2])
    angles = beliefs[:,2] - np.pi/2
    A = np.cos(angles)
    B = np.sin(angles)
    R = np.array([[A, B], [-B, A]])

    lowerLeft = points + np.dot([-4,4], R).T
    center = points + np.dot([0,-4], R).T
    lowerRight = points + np.dot([4,4], R).T

    l = lowerLeft[0]
    c = center[0]
    r = lowerRight[0]

    if trails:
      # Draw new lines
      self.canvas.create_line(l[0], l[1], c[0], c[1], fill='blue')
      self.canvas.create_line(r[0], r[1], c[0], c[1], fill='blue')
    else:
      # Move old lines
      try:
        self.canvas.coords(self.line1, l[0], l[1], c[0], c[1])
        self.canvas.coords(self.line2, r[0], r[1], c[0], c[1])
      except AttributeError:
        self.line1 = self.canvas.create_line(l[0], l[1], c[0], c[1],
                                             fill='blue')
        self.line2 = self.canvas.create_line(r[0], r[1], c[0], c[1],
                                             fill='blue')


x = 0
y = 0
theta = 0
lastHeading = None

def msg_callback(msg, sim):
  global x, y, theta, lastHeading

  msg = msg.odometry # don't need timestamp header

  if lastHeading is None:
      lastHeading = msg.heading

  dtheta = lastHeading - msg.heading
  dist = msg.distance_delta * 100    # convert from m to cm

  theta += dtheta
  x += dist * np.cos(theta)
  y += dist * np.sin(theta)

  lastHeading = msg.heading   # store last reading to calculate dtheta

  belief = np.array([x, y, theta])
  sim.refresh(belief, trails=False)

def spin_thread(sim):
  # While ROS is alive, wait for messages
  rospy.spin()
  # On exit, kill the mainloop and exit
  sim.root.destroy()

def main():
  # Get config
  topic_name = rospy.get_param('~topic', 'odometry')
  init_pose = rospy.get_param('~initial', '500,1150,-1.3')

  sim = SingleSimulator('Odometry')

  # Initial position
  global x, y, theta
  a = init_pose.split(',')
  x = float(a[0])
  y = float(a[1])
  theta = float(a[2])

  # Initialize ROS listener
  rospy.init_node('pose_viewer', anonymous=True)
  rospy.Subscriber(topic_name, OdometryStamped,
                   lambda msg: msg_callback(msg, sim))

  # Start rospy spin thread
  thread.start_new_thread(spin_thread, (sim,))

  # Start main Tk loop
  sim.root.mainloop()


if __name__ == '__main__':
  main()

