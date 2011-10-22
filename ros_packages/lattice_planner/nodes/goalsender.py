#!/usr/bin/env python

"""
Simple node to send in goal messages from command line
Publishes to topics:  goal
Subscribes to topics: pose
"""
import roslib; roslib.load_manifest('lattice_planner')
import rospy
import sys
import math

from std_msgs.msg import String
from bb_msgs.msg import Pose,Goal

pub_goal = rospy.Publisher('goal',Goal)
Ballbot_x = 10.0
Ballbot_y = 10.0
Ballbot_theta = math.pi/2

def goalsender():
    argv = rospy.myargv(argv=sys.argv) 
    if(len(argv) == 3):
        d_goal = float(argv[1])*100
        th_goal = float(argv[2])

        th_goal = Ballbot_theta - math.radians(th_goal)     # angle to goal in global coord
                                                     # = angle of car in global coord - angle to goal from car (which is in [-90deg,90deg]
        x2 = Ballbot_x*100.0 + d_goal*math.cos(th_goal)
        y2 = Ballbot_y*100.0 + d_goal*math.sin(th_goal)
        th2 = 0                                   # doesn't matter for goal test

        # publish goal
        goal_msg = Goal()
        goal_msg.goaltype = String("newball")
        goal_msg.pose = Pose(x2,y2,th2)
        pub_goal.publish(goal_msg)
        print "sent goal"
        rospy.signal_shutdown("Exiting")
        
    else:
        rospy.loginfo("Usage: rosrun lattice_planner goalsender.py d_dist[m] d_theta[deg]")
        rospy.signal_shutdown("Exiting") 
        

def received_odometry(data):
    global Ballbot_x,Ballbot_y,Ballbot_theta
    # Coordinate frame conversion from localization frame to planner frame
    Ballbot_x = (data.y + 3.658)
    Ballbot_y = (30.17 - data.x)
    Ballbot_theta = (data.theta - math.pi/2)%(2*math.pi)

if __name__ == '__main__':
    try:
        rospy.init_node('goalsender')
        rospy.Subscriber("pose",Pose, received_odometry)         
        goalsender() 
    except rospy.ROSInterruptException: pass

