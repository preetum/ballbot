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
from bb_msgs.msg import Pose,DriveCmd

path = None
newPath = False

# ODOMETRY inputs
Ballbot_X = 0
Ballbot_Y = 0
Ballbot_TH = 0

# CONTROL outputs
Ballbot_steering = 0 # radians
Ballbot_speed = 0    # m/s

# CONTROLLER PARAMETRS
targetlookahead = 15.0 # PD steering control to reach a moving target 15 cm ahead of the car

def nearestNeighbor_inPath((x,y,th),currentindex_inPath):
    """
    Given x,y,th, find the point closest to the robot in the path
    """
    global path
    d_min = float('inf')
    index_min = 0

    index = currentindex_inPath
    for point in path[currentindex_inPath:]:
        """
        Search only from the currentindex.
        """
        d = util.distance_Euclidean(point[0],point[1],x,y)
        if(d < d_min):
            d_min = d
            index_min = index
        elif(d < 5): # if error is less than 5 cm, take this as the nearest point
            d_min = d
            index_min = index
            break
        index +=1
    return index_min


def controller_PD():
    """
    Implements controller
    """    
    pub = rospy.Publisher('vel_cmd', Drive_Cmd)
    global path,newPath,Ballbot_steering,Ballbot_speed
    Ballbot_speed = 0.0
    Ballbot_steering = 0
  
    currentindex_inPath = 0 # path index that the car is closest to
    targetindex_inPath = 0  # path index that is 50 cm ahead of the car

    # PID stuff
    # -- tuning parameters
    steering_P = 0.0
    steering_D = 0.0
    
    # -- state parameters
    error = 0.0
    theta_old = 0.0 # stores previous theta value for D-term

    while not rospy.is_shutdown():
        
        while(not util.goalTest(currentNode)):
            currentNode = util.LatticeNode(Ballbot_X,Ballbot_Y,Ballbot_TH)        
            if(newPath == True):
                # if there is a new path, restart driving along this path
                newPath = False
                currentindex_inPath = 0
                targetindex_inPath = 0
                theta_old = Ballbot_TH
                break
            else:                                    
                currentindex_inPath = nearestNeighbor_inPath((Ballbot_X,Ballbot_Y,Ballbot_TH),currentindex_inPath)
                targetindex_inPath = currentindex_inPath + int(targetlookahead/5.0) # points are at a separation of 5 cm

                # P - Proportional term
                error = Ballbot_TH - path[targetindex_inPath][2]
                Pterm = steering_P * error
                
                # D - Differential term
                Dterm = steering_D * (Ballbot_TH - theta_old)
                theta_old = Ballbot_TH

                # set output
                Ballbot_steering = Pterm - Dterm
                pub.publish(Ballbot_speed,Ballbot_steering)

        # goal reached!
        Ballbot_speed = 0
        Ballbot_steering = 0
        pub.publish(Ballbot_speed,Ballbot_steering)
        while(newPath == False):
            print goal_arrived
            #do nothing

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
    controller_PD() 
    rospy.spin()
    
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException: pass
