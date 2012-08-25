#!/usr/bin/env python
import roslib; roslib.load_manifest('court_localization')
import rospy
from bb_msgs.msg import Pose
from particle_viewer import Simulator

import random, time, thread, sys
import numpy as np

last_true_pose = None

class SingleSimulator(Simulator):
  def __init__(self, title='Pose', trails=False):
    super(SingleSimulator, self).__init__(title)
    self.trails = trails
    self.pose_lines = []
    self.truth_lines = []

  def draw_pose(self, pose, color='blue'):
    l, c, r = self.get_points(pose)

    if self.trails:
      # Draw new lines
      self.canvas.create_line(l[0], l[1], c[0], c[1], fill=color)
      self.canvas.create_line(r[0], r[1], c[0], c[1], fill=color)
    else:
      # Move old lines
      if not self.pose_lines:
        self.pose_lines.append(
          self.canvas.create_line(l[0], l[1], c[0], c[1], fill=color))
        self.pose_lines.append(
          self.canvas.create_line(r[0], r[1], c[0], c[1], fill=color))
      else:
        self.canvas.coords(self.pose_lines[0], l[0], l[1], c[0], c[1])
        self.canvas.itemconfig(self.pose_lines[0], fill=color)
        self.canvas.coords(self.pose_lines[1], r[0], r[1], c[0], c[1])
        self.canvas.itemconfig(self.pose_lines[1], fill=color)

  def draw_truth(self, pose, color='orange'):
    l, c, r = self.get_points(pose)

    if self.trails:
      # Draw new lines
      self.canvas.create_line(l[0], l[1], c[0], c[1], fill=color)
      self.canvas.create_line(r[0], r[1], c[0], c[1], fill=color)
    else:
      # Move old lines
      if not self.truth_lines:
        self.truth_lines.append(
          self.canvas.create_line(l[0], l[1], c[0], c[1], fill=color))
        self.truth_lines.append(
          self.canvas.create_line(r[0], r[1], c[0], c[1], fill=color))
      else:
        self.canvas.coords(self.truth_lines[0], l[0], l[1], c[0], c[1])
        self.canvas.coords(self.truth_lines[1], r[0], r[1], c[0], c[1])


  def get_points(self, belief, size=4):
    beliefs = np.array([belief])
    points = self.transform(beliefs[:,0:2])
    angles = beliefs[:,2] - np.pi/2
    A = np.cos(angles)
    B = np.sin(angles)
    R = np.array([[A, B], [-B, A]])

    lowerLeft = points + np.dot([-size,size], R).T
    center = points + np.dot([0,-size], R).T
    lowerRight = points + np.dot([size,size], R).T

    l = lowerLeft[0]
    c = center[0]
    r = lowerRight[0]

    return (l, c, r)

def msg_callback(msg, sim):
  pose = np.array([msg.x, msg.y, msg.theta])
  color = 'blue'

  # compare estimated pose to last true; draw in red if off
  if last_true_pose is not None:
    diff = pose - last_true_pose
    dist = np.linalg.norm(diff[0:1])
    if dist > 30.5:  # in cm
      color = 'red'
  sim.draw_pose(pose, color)

def truth_msg_callback(msg, sim):
  pose = np.array([msg.x, msg.y, msg.theta])
  global last_true_pose
  last_true_pose = pose
  sim.draw_truth(pose)

def spin_thread(sim):
  # While ROS is alive, wait for messages
  rospy.spin()
  # On exit, kill the mainloop and exit
  sim.root.destroy()

def main():
  # Get config
  topic_name = rospy.get_param('~pose', 'pose')
  truth_topic_name = rospy.get_param('~true', 'pose_true')

  sim = SingleSimulator(trails=False)

  # Initialize ROS listener
  rospy.init_node('pose_viewer', anonymous=True)
  rospy.Subscriber(topic_name, Pose, lambda msg: msg_callback(msg, sim),
                   queue_size=1)
  rospy.Subscriber(truth_topic_name, Pose,
                   lambda msg: truth_msg_callback(msg, sim), queue_size=1)

  # Start rospy spin thread
  thread.start_new_thread(spin_thread, (sim,))

  # Start main Tk loop
  sim.root.mainloop()


if __name__ == '__main__':
  main()

