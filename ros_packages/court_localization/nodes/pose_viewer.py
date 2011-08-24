#!/usr/bin/env python
import roslib; roslib.load_manifest('court_localization')
import rospy
from bb_msgs.msg import Pose
from odometry_viewer import SingleSimulator

import random, time, thread, sys
import numpy as np

def msg_callback(msg, sim):
  belief = np.array([msg.x, msg.y, msg.theta])
  sim.refresh(belief, trails=True)

def spin_thread(sim):
  # While ROS is alive, wait for messages
  rospy.spin()
  # On exit, kill the mainloop and exit
  sim.root.destroy()

def main():
  # Get config
  topic_name = rospy.get_param('~topic', 'pose')

  sim = SingleSimulator('Pose')

  # Initialize ROS listener
  rospy.init_node('pose_viewer', anonymous=True)
  rospy.Subscriber(topic_name, Pose, lambda msg: msg_callback(msg, sim),
                   queue_size=1)

  # Start rospy spin thread
  thread.start_new_thread(spin_thread, (sim,))

  # Start main Tk loop
  sim.root.mainloop()


if __name__ == '__main__':
  main()

