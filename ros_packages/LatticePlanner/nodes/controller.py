#!/usr/bin/env python
"""
controller.py - implements a controller to make Ballbot drive along the planned path
                When the message "ValidPath" is seen on the 'ValidPath' topic, a valid path has been computed/recomputed and must be followed 
                The most recent path is stored in util.path
"""
import roslib; roslib.load_manifest('LatticePlanner')
import rospy
import util
import math
from std_msgs.msg import String
from bb_msgs.msg import Pose,DriveCmd,Path

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

pub_velcmd = rospy.Publisher('vel_cmd', DriveCmd)

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
        d = util.distance_Euclidean(point.x,point.y,x,y)
        if(d < d_min):
            d_min = d
            index_min = index
        elif(d < 0.05): # if error is less than 5 cm, take this as the nearest point
            d_min = d
            index_min = index
            break
        index +=1
    return index_min


def controller_PD():
    """
    Implements controller
    """    
    global path,newPath,Ballbot_steering,Ballbot_speed,pub_velcmd
    Ballbot_speed = 1.0
    Ballbot_steering = 0
  
    currentindex_inPath = 0 # path index that the car is closest to
    targetindex_inPath = 0  # path index that is 50 cm ahead of the car

    # PID stuff
    # -- tuning parameters
    steering_P = 1.0
    steering_D = 0.0
    
    # -- state parameters
    error = 0.0
    theta_old = 0.0 # stores previous theta value for D-term
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        if newPath == False:
            continue
        while(currentindex_inPath < len(path)-1) and not (rospy.is_shutdown()):
            Ballbot_speed = 0.0 # set speed                   
            if(newPath == True):
                # if there is a new path, restart driving along this path
                newPath = False
                currentindex_inPath = 0
                targetindex_inPath = 0
                theta_old = Ballbot_TH
                continue
            else:                                    
                currentindex_inPath = nearestNeighbor_inPath((Ballbot_X,Ballbot_Y,Ballbot_TH),currentindex_inPath)
                targetindex_inPath = min(len(path)-1,currentindex_inPath + int(targetlookahead/5.0)) # points are at a separation of 5 cm                
                #print "currentindex",currentindex_inPath,"targetindex",targetindex_inPath,"length",len(path)
                # P - Proportional term                

                heading = (math.atan2(path[targetindex_inPath].y-Ballbot_Y, path[targetindex_inPath].x-Ballbot_X)%(2*math.pi))
                error = Ballbot_TH - heading
                #print "Ballbot",(Ballbot_X,Ballbot_Y,Ballbot_TH),"targetpoint",(path[targetindex_inPath].x,path[targetindex_inPath].y)
                #print "heading",heading,"error",error
                #raw_input()

                Pterm = steering_P * error
                
                # D - Differential term
                Dterm = steering_D * (Ballbot_TH - theta_old)
                theta_old = Ballbot_TH

                # set output
                Ballbot_steering = Pterm - Dterm
                if(Ballbot_steering > math.radians(30)):
                    Ballbot_steering = math.radians(30)
                elif(Ballbot_steering < -math.radians(30)):
                    Ballbot_steering = -math.radians(30)
                pub_velcmd.publish(Ballbot_speed,Ballbot_steering)

	     r.sleep()	
        
        # goal reached!
        print "goalreached"
        Ballbot_speed = 0
        Ballbot_steering = 0
        pub_velcmd.publish(Ballbot_speed,Ballbot_steering)                    
	
def newPath_arrived(data):
    """
    A new path has been computed. Feed it into path and set newPath to true
    """
    global path,newPath
    path = data.poses
    newPath = True
    print "newpathseen! length",len(path)
    print "Hit any key to begin driving"
    raw_input()

def received_odometry(data):
    """
    Update pose of the car
    """
    global Ballbot_X,Ballbot_Y,Ballbot_TH
    Ballbot_X  = data.x
    Ballbot_Y  = data.y
    Ballbot_TH = data.theta


def listener():
    rospy.init_node('Controller',anonymous = True)
    rospy.Subscriber('path', Path, newPath_arrived)    
    rospy.Subscriber('odometry', Pose, received_odometry)
    controller_PD() 
    rospy.spin()
    
if __name__ == '__main__':
    try:
        listener()
    except rospy.ROSInterruptException: pass
