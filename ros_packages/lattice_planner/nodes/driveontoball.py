#!/usr/bin/env python

"""
Drive on to ball planner+controller

Author: Karthik Lakshmanan

Description:
This node receives ball location from the ball finder, and drives the steering to drive directly on to the ball, and pick it up when reached

"""

import roslib; roslib.load_manifest('lattice_planner')
import rospy
from std_msgs.msg import String
from bb_msgs.msg import Goal,Pose,BallPickup,BallPosition,DriveCmd
import math

pub_goal = rospy.Publisher("goal",Goal)
pub_velcmd = rospy.Publisher('vel_cmd', DriveCmd)
pub_ballpickup = rospy.Publisher("ball_pickup",BallPickup)

# Ballbot State variables

Ballbot_x = 10.0
Ballbot_y = 10.0
Ballbot_theta = 10.0
Ballbot_status = None

Ballbot_STATE = "IDLE" # can be IDLE, BALLPICKUP, BALLRETRIEVE

# Ballbot Control variables
Ballbot_steering = 0.0
Ballbot_speed = 0.0

# Ball state variables
Ball_d = -1.0
Ball_theta = -1.0

# Goal related variables
Goal_current = None

def callback(data):
    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)

def received_ballposition(data):
    global Ball_d,Ball_theta
    Ball_d = data.d
    Ball_theta = data.theta
    #rospy.loginfo("Ball_d %f Ball_theta %f",Ball_d,Ball_theta)
    
def CMD_STATEMACHINE():
    """
    implements a state machine for Ballbot    """
    global Ballbot_STATE
    
    while not rospy.is_shutdown():

        if(Ballbot_STATE == "IDLE"):
            if (state_IDLE() == "newballseen"):
                next_STATE = "BALLPICKUP"
            else:
                next_STATE = "IDLE"

        elif(Ballbot_STATE == "BALLPICKUP"):
            if (state_BALLPICKUP() == "pickedup"):
                next_STATE = "BALLDELIVERY"
            else:
                next_STATE = "BALLPICKUP"

        elif(Ballbot_STATE == "BALLDELIVERY"):
            if(state_BALLDELIVERY() == "delivered"):
                next_STATE = "IDLE"
            else:
                next_STATE = "BALLDELIVERY"

        else: # Default transition
            next_STATE = "IDLE"
        
        rospy.sleep(0.1)         
        Ballbot_STATE = next_STATE
    
def state_IDLE():
    global Ball_d,Ball_theta
    if (Ball_d >= 0):
        print "new ball seen. Hit a key to pickup",Ball_d,Ball_theta
        raw_input()        
        return "newballseen"    
    else:
        return "noball"

def state_BALLPICKUP():    
    global Ballbot_speed, Ballbot_steering
    Ballbot_speed = 1.0
    Steering_gain = 1.0
    r = rospy.Rate(10)
    while((Ball_d >= 0.5) and not(rospy.is_shutdown())): 
        Ballbot_steering = Steering_gain * Ball_theta
        if(Ballbot_steering >= math.radians(30)):
            Ballbot_steering = math.radians(30)
        elif(Ballbot_steering <= math.radians(-30)):
            Ballbot_steering = -math.radians(30)

        rospy.loginfo("Ball_d %f Ball_theta %f Ballbot_steering %f",Ball_d,Ball_theta,Ballbot_steering)
        pub_velcmd.publish(Ballbot_speed,Ballbot_steering)
        r.sleep()    

    #--------------------------------------
    Ballbot_speed = 0.5
    Ballbot_steering = 0.0    
    pub_velcmd.publish(Ballbot_speed,Ballbot_steering)

    # activate ball pickup for 5 seconds
    ballpickup_msg = BallPickup()    
    ballpickup_msg.direction = -1
    pub_ballpickup.publish(ballpickup_msg)
    ballpickup_msg = BallPickup()    
    ballpickup_msg.direction = 0
    pub_ballpickup.publish(ballpickup_msg)
    
    
    rospy.sleep(5)
    
    # deactivate ball pickup
    ballpick_msg = BallPickup()
    ballpickup_msg.direction = 1
    pub_ballpickup.publish(ballpickup_msg)                
    
    Ballbot_speed = 0.0
    pub_velcmd.publish(Ballbot_speed,Ballbot_steering)

    #----------------------------------------
    return "pickedup"

def state_BALLDELIVERY():
    global Ballbot_speed,Ballbot_steering
    Ballbot_speed = 1
    Ballbot_steering = math.radians(30)
    pub_velcmd.publish(Ballbot_speed,Ballbot_steering)
    rospy.sleep(5)
    
    Ballbot_speed = 0
    Ballbot_steering = 0
    pub_velcmd.publish(Ballbot_speed,Ballbot_steering)

    # activate ball delivery for 5 seconds
    ballpickup_msg = BallPickup()
    ballpickup_msg.direction = -1
    pub_ballpickup.publish(ballpickup_msg)        
    rospy.sleep(5)
    
    # deactivate ball pickup
    ballpick_msg = BallPickup()
    ballpickup_msg.direction = 1
    pub_ballpickup.publish(ballpickup_msg)   

    return "delivered"


def initialize_commandcenter():
    rospy.init_node('driveontoball', anonymous=True)        
    rospy.Subscriber("ball",BallPosition,received_ballposition,buff_size=1)    

    CMD_STATEMACHINE()
    rospy.spin()
    
if __name__ == '__main__':
    initialize_commandcenter()
