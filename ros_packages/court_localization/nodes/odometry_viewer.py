#!/usr/bin/env python
import roslib; roslib.load_manifest('court_localization')
import rospy
from bb_msgs.msg import Odometry
from particle_viewer import Simulator

import random, time, thread, sys, math
import numpy as np

x = 0
y = 0
theta = 0
lastHeading = None

def msg_callback(msg, sim):
  global x, y, theta, lastHeading

  if lastHeading is None:
      lastHeading = msg.heading

  dtheta = lastHeading - msg.heading
  dist = msg.distance_delta * 100    # convert from m to cm

  theta += dtheta
  x += dist * np.cos(theta)
  y += dist * np.sin(theta)

  lastHeading = msg.heading   # store last reading to calculate dtheta

  beliefs = np.array([[x, y, theta]])
  sim.refresh(beliefs, trails=True)

def spin_thread(sim):
  # While ROS is alive, wait for messages
  rospy.spin()
  # On exit, kill the mainloop and exit
  sim.root.destroy()

def main():
  # Get config
  topic_name = rospy.get_param('~topic', 'odometry')
  init_pose = rospy.get_param('~initial', '500,1150,-1.3')

  sim = Simulator('Odometry')

  # Initial position
  global x, y, theta
  a = init_pose.split(',')
  x = float(a[0])
  y = float(a[1])
  theta = float(a[2])

  # Initialize ROS listener
  rospy.init_node('pose_viewer', anonymous=True)
  rospy.Subscriber(topic_name, Odometry,
                   lambda msg: msg_callback(msg, sim))

  # Start rospy spin thread
  thread.start_new_thread(spin_thread, (sim,))

  # Start main Tk loop
  sim.root.mainloop()


if __name__ == '__main__':
  main()

