#!/usr/bin/env python

"""
Command center for Ballbot motion planning.

Author: Karthik Lakshmanan

Description:
This node receives ball location from the tennis ball tracker, odometry information from the bot, and sends appropriate commands to the lattice_planner package

"""

import roslib; roslib.load_manifest('lattice_planner')
import rospy
from std_msgs.msg import String
from bb_msgs.msg import Goal,Pose,BallPickup,BallPosition
import math

pub_ball = rospy.Publisher("ball",BallPosition)

# Ballbot State variables

Ballbot_x = 10.0
Ballbot_y = 10.0
Ballbot_theta = 10.0
Ballbot_status = None

Ballbot_STATE = "IDLE" # can be IDLE, BALLPICKUP, BALLRETRIEVE

# Ball state variables
Ball_x = -1.0
Ball_y = -1.0
Ball_theta = -1.0

# Goal related variables
Goal_current = None

def callback(data):
    rospy.loginfo(rospy.get_name()+"I heard %s",data.data)

def received_odometry(data):
    global Ballbot_x,Ballbot_y,Ballbot_theta
    Ballbot_x = data.x
    Ballbot_y = data.y
    Ballbot_theta = data.theta

"""
def received_ballposition(data):
    global Ball_x,Ball_y,Ball_theta
    Ball_x = data.x
    Ball_y = data.y
    Ball_theta = data.theta
"""

def received_ballposition(data):
    global Ball_x,Ball_y
    heading_ball = data.theta
    print "d =",data.d,"th=",heading_ball
    Ball_x = Ballbot_x + data.d*math.cos(heading_ball)
    Ball_y = Ballbot_y + data.d*math.sin(heading_ball)
    print "Ball_x",Ball_x,"Ball_y",Ball_y

def received_status(data):
    global Ballbot_status
    Ballbot_status = data.data

def CMD_STATEMACHINE():
    """
    implements a state machine for Ballbot
    """
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
    global Goal_current,Ball_x,Ball_y,Ball_theta
    if (Ball_x >= 0) and (Ball_y >= 0):
        print "new ball seen. Hit a key to pickup",Ball_x,Ball_y,Ball_theta
        raw_input()
        
        goal_msg = Goal()
        goal_msg.goaltype = String("newball")    
        goal_msg.pose = Pose(Ball_x*100.0,Ball_y*100.0,Ball_theta)            pub_goal.publish(goal_msg)        
        Goal_current = goal_msg
        return "newballseen"    
    else:
        return "noball"

def state_BALLPICKUP():
    global Goal_current,Ballbot_status
    while((Ballbot_status != "goalreached") and not(rospy.is_shutdown())): 
        if(Ball_x >= 0) and (Ball_y >= 0): # if ball is visible
            dist = math.sqrt(math.pow((Goal_current.pose.x/100.0 - Ball_x),2) + math.pow((Goal_current.pose.y/100.0 - Ball_y),2))        
            goal_msg = Goal()
            goal_msg.pose = Pose(Ball_x*100.0,Ball_y*100.0,Ball_theta)

            
            if (dist >= 0.10) and (dist <= 0.5):                
                goal_msg.goaltype = String("updategoal")                                                            
                pub_goal.publish(goal_msg)        
                Goal_current = goal_msg
                #print (Ball_x,Ball_y),goal_msg.goaltype

            elif(dist >= 0.5):
                goal_msg.goaltype = String("newball")                
                pub_goal.publish(goal_msg)        
                Goal_current = goal_msg

                #print (Ball_x,Ball_y),goal_msg.goaltype
            
        rospy.sleep(1.0) # update goal position every 1 s for now    

    Ballbot_status = None   

    #--------------------------------------
    # activate ball pickup for 10 seconds
    ballpickup_msg = BallPickup()
    ballpickup_msg.direction = 1
    pub_ballpickup.publish(ballpickup_msg)
    
    rospy.sleep(10)
    
    # deactivate ball pickup
    ballpick_msg = BallPickup()
    ballpickup_msg.direction = 0
    pub_ballpickup.publish(ballpickup_msg)                
    
    #----------------------------------------

    goal_msg = Goal()
    goal_msg.pose = Pose(10.0 * 100.0,10.0 * 100.0,0.0)
    goal_msg.goaltype = String("gotopose")
    print "Go to (10.0 10.0 0); hit any key to drive"
    raw_input()
    pub_goal.publish(goal_msg)
    Goal_current = goal_msg

    return "pickedup"

def state_BALLDELIVERY():
    global Ballbot_status
    if(Ballbot_status == "goalreached"):
        Ballbot_status = None
        # activate ball delivery for 10 seconds
        ballpickup_msg = BallPickup()
        ballpickup_msg.direction = -1
        pub_ballpickup.publish(ballpickup_msg)        
        rospy.sleep(10)
    
        # deactivate ball pickup
        ballpick_msg = BallPickup()
        ballpickup_msg.direction = 0
        pub_ballpickup.publish(ballpickup_msg)   
        return "delivered"
    else:
        return "notdelivered"

def initialize_commandcenter():
    rospy.init_node('commandcenter', anonymous=True)
    rospy.Subscriber("pose", Pose, received_odometry)
    #rospy.Subscriber("ball",Pose,received_ballposition)
    rospy.Subscriber("ball",BallPosition,received_ballposition)
    rospy.Subscriber("status",String,received_status)

    CMD_STATEMACHINE()
    rospy.spin()
    
if __name__ == '__main__':
    initialize_commandcenter()
