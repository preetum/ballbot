#!/usr/bin/env python
"""
simulator.py -  simulates a perfectly moving Ballbot, moving at about 1m/s

Publishes to : /pose
Subscribes to: /path                
"""
import roslib; roslib.load_manifest('lattice_planner')
import rospy
import util
import math
import time
from std_msgs.msg import String
from bb_msgs.msg import Pose,Path

path = None
newPath = False

# ODOMETRY outputs
Ballbot_x = 10
Ballbot_y = 10
Ballbot_theta = math.pi/2

# simulator parameter
Ballbot_speed = 1.0

pub_status = rospy.Publisher('status', String)

def simulator():
    """
    Implements simulator
    """    
    global path,newPath,Ballbot_speed,Ballbot_x,Ballbot_y,Ballbot_theta
    currentindex_inPath = 0 
    pub_odom = rospy.Publisher('pose',Pose)            

    # each point is about 5 cm apart, therefore sleep for (Ballbot_speed/20) seconds
    r = rospy.Rate(20.0/Ballbot_speed)
    #r = rospy.Rate(3.0) # for debugging
    while not rospy.is_shutdown():
        currentindex_inPath = 0
        if newPath == False:
            # wait for first path to arrive
            pub_odom.publish(30.17 - Ballbot_y,Ballbot_x - 3.658,(math.pi/2 + Ballbot_theta)%(2*math.pi))
            r.sleep()
            continue
        while(currentindex_inPath < len(path)):
            Ballbot_speed = 0.0 # set speed            
            if(newPath == True):
                # if there is a new path, restart driving along this path
                newPath = False
                currentindex_inPath = 0                            
                continue
            else:                 
                Ballbot_x = path[currentindex_inPath].pose.x
                Ballbot_y = path[currentindex_inPath].pose.y
                Ballbot_theta = path[currentindex_inPath].pose.theta
                pub_odom.publish(30.17 - Ballbot_y,Ballbot_x - 3.658,(math.pi/2 + Ballbot_theta)%(2*math.pi))
                currentindex_inPath += 1                            
            r.sleep()     
        pub_status.publish("goalreached")
        pub_odom.publish(30.17 - Ballbot_y,Ballbot_x - 3.658,(math.pi/2 + Ballbot_theta)%(2*math.pi))
        

def newPath_arrived(data):
    """
    A new path has been computed. Feed it into path and set newPath to true
    """
    global path,newPath
    path = data.poses    
    newPath = True # commented for debugging MTAstar
    print "newpathseen! length",len(path)

def listener():
    rospy.init_node('Controller',anonymous = True)
    rospy.Subscriber('path', Path, newPath_arrived)        
    simulator()
    rospy.spin()
    
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException: pass
