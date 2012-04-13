#!/usr/bin/env python
import roslib; roslib.load_manifest('court_localization')
import rospy
from bb_msgs.msg import Odometry, Pose
from particle_viewer import Simulator

import random, time, thread, sys, math
import numpy as np

x = 0
y = 0
theta = 0
lastHeading = None
i = 0
j = 0

def draw_particle(beliefs, sim, *args, **kwargs):
    points = sim.transform(beliefs[:,0:2])
    angles = beliefs[:,2] - np.pi/2
    A = np.cos(angles)
    B = np.sin(angles)
    R = np.array([[A, B], [-B, A]])

    lowerLeft = points + np.dot([-4,4], R).T
    center = points + np.dot([0,-4], R).T
    lowerRight = points + np.dot([4,4], R).T

    for l, c, r in zip(lowerLeft, center, lowerRight):
      sim.canvas.create_line(l[0], l[1], c[0], c[1], *args, **kwargs)
      sim.canvas.create_line(r[0], r[1], c[0], c[1], *args, **kwargs)


def msg_callback(msg, sim):
  global x, y, theta, lastHeading, i

  if lastHeading is None:
      lastHeading = msg.heading

  dtheta = lastHeading - msg.heading
  dist = msg.distance_delta * 100    # convert from m to cm

  theta += dtheta
  x += dist * np.cos(theta)
  y += dist * np.sin(theta)

  lastHeading = msg.heading   # store last reading to calculate dtheta


  i += 1
  if i == 10:
    i = 0
  else:
    return
  draw_particle(np.array([[x, y, theta]]), sim, fill='blue')

def pose_callback(msg, sim):
  global j
  j += 1
  if j == 10:
    j = 0
  else:
    return

  draw_particle(np.array([[msg.x, msg.y, msg.theta]]), sim, fill='orange')


def spin_thread(sim):
  # While ROS is alive, wait for messages
  rospy.spin()
  # On exit, kill the mainloop and exit
  sim.root.destroy()

def main():
  # Get config
  init_pose = rospy.get_param('~initial', '530,1150,-1.3')

  sim = Simulator('Odometry')

  # Initial position
  global x, y, theta
  a = init_pose.split(',')
  x = float(a[0])
  y = float(a[1])
  theta = float(a[2])

  # Initialize ROS listener
  rospy.init_node('pose_viewer', anonymous=True)
  rospy.Subscriber('odometry', Odometry,
                   lambda msg: msg_callback(msg, sim))
  rospy.Subscriber('pose', Pose, lambda msg: pose_callback(msg, sim))


  draw_particle(np.array([[x, y, np.pi/2]]), sim, fill='blue')

  draw_particle(np.array([[x+100, y, np.pi/2]]), sim, fill='orange')

  # Start rospy spin thread
  thread.start_new_thread(spin_thread, (sim,))

  # Start main Tk loop
  sim.root.mainloop()


if __name__ == '__main__':
  main()

