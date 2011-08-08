#!/usr/bin/env python
"""
controller.py - implements a controller to make Ballbot drive along the planned path
                When the message "ValidPath" is seen on the 'ValidPath' topic, a valid path has been computed/recomputed and must be followed 
                The most recent path is stored in util.path
"""
import roslib; roslib.load_manifest('LatticePlanner')
import rospy
import LatticePlannersim
import math
from std_msgs.msg import String
from bb_msgs.msg import Pose

path = None
newPath = False

# ODOMETRY data
Ballbot_X = 0
Ballbot_Y = 0
Ballbot_TH = 0

# CONTROL data
Ballbot_steering = 0 # radians
Ballbot_speed = 0    # m/s


def controller():
    """
    Implements controller
    """    
    global newPath,Ballbot_steering,Ballbot_speed
    while not rospy.is_shutdown():
        Ballbot_speed = 1
        for point in path:            
            if(newPath == True):
                # if there is a new path, restart driving along this path
                newPath = False
                break
            else:
                
                    

def newPath_arrived(data):
    """
    A new path has been computed. Feed it into newPath and signal the controller by setting newPath = True
    """
    global newPath
    if(data.data == "ValidPath"):
        newPath = True
        path = LatticePlannersim.path

def update_Pose(data):
    """
    Update pose of the car
    """
    global Ballbot_X,Ballbot_Y,Ballbot_TH
    Ballbot_X  = data.x
    Ballbot_Y  = data.y
    Ballbot_TH = data.th

def listener():
    rospy.init_node('Controller',anonymous = True)
    rospy.Subscriber('ValidPath', String, newPath_arrived)    
    rospy.Subscriber('Odometry', Pose, update_Pose)
    controller() 
    rospy.spin()
    
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException: pass
