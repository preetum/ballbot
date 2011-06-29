#!/usr/bin/env python

"""
No obstacles, dubins curve based planner
"""

import roslib; roslib.load_manifest('navigation')
import rospy
import math
import sys
from navigation.msg import odom_data
from navigation.msg import goal_msg
from navigation.msg import drive_cmd

ROBOT_RADIUS   = 60.96  #in cm
ROBOT_SPEED    = 50.0  #in cm/s

# Globals
curve_number = -1
Moves = [-1,-1,-1,-1]
cmd_distance = 0
pub = rospy.Publisher('vel_cmd', drive_cmd)
first_loop = 1
init_dist = 0
x = 0
y = 0
dist = 0
theta  = 0
cmd_dist = 0
cmd_theta = 0


new_Plan = 1 # = 1 when we want to generate a new plan, 0 otherwise
isRunning_Plan = 1 # = 1 when a plan is being executed, 0 otherwise
robot_setpoint_steering = 0
robot_setpoint_speed =  0

def fmodr(x,y):
    return x - y*math.floor(x/y)

def mod2pi(theta):
    return fmodr(theta,2*math.pi) 

def dubins(alpha,beta,d):
    # find distance between start and end points
    #print "dubins d ",d," alpha ",alpha, " beta ", beta
    d = d/ROBOT_RADIUS
    lengths = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
    t       = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
    p       = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
    q       = [-1.0,-1.0,-1.0,-1.0,-1.0,-1.0]
    sin_alpha = math.sin(alpha)
    cos_alpha = math.cos(alpha)
    sin_beta = math.sin(beta)
    cos_beta = math.cos(beta)
    cos_alphaminusbeta = math.cos(alpha - beta)

     # find length of curve 0: Left Straight Left                     [Lq(Sp(Lt(0,0,alpha))) = (d,0,beta)]
    tmp0 = d+sin_alpha-sin_beta;
    tmp2 = 2 + (d*d) - (2*cos_alphaminusbeta) + (2*d*(sin_alpha - sin_beta))
    if(tmp2 >= 0) and (tmp0 > 0):
        tmp1 = math.atan((cos_beta - cos_alpha)/tmp0)
        t[0]    = mod2pi(-alpha + tmp1)
        p[0]    = math.sqrt(tmp2)
        q[0]    = mod2pi(beta - tmp1)
        lengths[0] = t[0] + p[0] + q[0]
    #print "LSL ",lengths[0]
    #print "t = ",t[0]," p = ",p[0]," q = ",q[0]

 # find length of curve 1: Right Straight Right                    [Rq(Sp(Rt(0,0,alpha))) = (d,0,beta)]
    tmp0 = d - sin_alpha + sin_beta
    tmp2 = 2 + (d*d) - 2*cos_alphaminusbeta + (2*d*(sin_beta - sin_alpha))
    if(tmp2 >= 0 and tmp0 > 0):
        tmp1 = math.atan((cos_alpha - cos_beta)/tmp0)
        t[1]    = mod2pi(alpha - tmp1)
        p[1]    = math.sqrt(tmp2)
        q[1]    = mod2pi(-beta + tmp1)
        lengths[1] = t[1] + p[1] + q[1]
    #print "RSR ",lengths[1]
    #print "t = ",t[1]," p = ",p[1]," q = ",q[1]

 # find length of curve 2: Left Straight Right                     [Rq(Sp(Lt(0,0,alpha))) = (d,0,beta)]
    tmp1 = -2 + (d*d) + (2*cos_alphaminusbeta) + (2*d*(sin_alpha + sin_beta))
    if(tmp1 >= 0):
        p[2] = math.sqrt(tmp1)
        tmp2 = math.atan((-cos_alpha-cos_beta)/(d+sin_alpha+sin_beta)) - math.atan(-2.0/p[2])
        t[2] = mod2pi(-alpha + tmp2)
        q[2] = mod2pi( - mod2pi(beta) + tmp2)
        lengths[2] = t[2] + p[2] + q[2]
    #print "LSR ",lengths[2]
    #print "t = ",t[2]," p = ",p[2]," q = ",q[2]

  # find length of curve 3: Right Straight Left                      [Lq(Sp(Rt(0,0,alpha))) = (d,0,beta)]
    tmp1 = (d*d) - 2.0 + (2*cos_alphaminusbeta) - (2*d*(sin_alpha + sin_beta))
    if(tmp1 > 0):
        p[3] = math.sqrt(tmp1)
        tmp2 = math.atan((cos_alpha + cos_beta)/(d - sin_alpha - sin_beta)) - math.atan(2.0/p[3])
        t[3] = mod2pi(alpha - tmp2)
        q[3] = mod2pi(beta - tmp2)
        lengths[3] = t[3] + p[3] + q[3]
    #print "RSL ",lengths[3]
    #print "t = ",t[3]," p = ",p[3]," q = ",q[3]

    # find length of curve 4: Right Left Right                         [Rq(Lp(Rt(0,0,alpha))) = (d,0,beta)]
    tmp_rlr = (6.0 - d*d + 2.0*cos_alphaminusbeta + 2.0 * d * (sin_alpha - sin_beta))/8.0
    if(math.fabs(tmp_rlr) < 1):
        p[4] = math.acos(tmp_rlr)
        t[4] = mod2pi(alpha - math.atan2((cos_alpha - cos_beta),(d-sin_alpha + sin_beta)) + mod2pi(p[4]/2.0))
        q[4] = mod2pi(alpha - beta - t[4] + mod2pi(p[4]))
        lengths[4] = t[4] + p[4] + q[4]

 # find length of curve 5: Left Right Left                          [Lq(Rp(Lt(0,0,alpha))) = (d,0,beta)]
    tmp_lrl = (6.0 - d*d + 2*cos_alphaminusbeta + 2*d*(-sin_alpha + sin_beta))/8.0
    if(math.fabs(tmp_lrl) < 1):
        p[5] = mod2pi(math.acos(tmp_lrl))
        t[5] = mod2pi(-alpha - math.atan2((cos_alpha - cos_beta),(d + sin_alpha - sin_beta)) + p[5]/2.0)
        q[5] = mod2pi(mod2pi(beta) - alpha - t[5] + mod2pi(p[5]))
        lengths[5] = t[5] + p[5] + q[5]
    #print "LRL ",lengths[5]
    #print "t = ",t[5]," p = ",p[5]," q = ",q[5]

    # find curve with minimum length
    i = 0
    for length in lengths:
        if(length >= 0): 
            min_length = length
            curve_number = i

    i = 0
    for length in lengths:
        if((length <= min_length) and (length >= 0)):
            min_length = length
            curve_number = i
        i=i+1                                
   
    Moves[0] = curve_number
    Moves[1] = ROBOT_RADIUS * t[curve_number]
    Moves[2] = ROBOT_RADIUS * p[curve_number]
    Moves[3] = ROBOT_RADIUS * q[curve_number]


def drive_straight():
    global first_loop, init_dist,cmd_dist, dist,new_Plan,isRunning_Plan
    
    if first_loop ==1:
        init_dist = dist
        first_loop = 0
    driveStraight()
    while((dist - init_dist)/0.845 < cmd_dist):
        continue
    first_loop =1
    Stop()
    new_Plan = 1
    isRunning_Plan = 0

def drive_circle():
    global first_loop,init_dist,cmd_dist,dist,new_Plan,isRunning_Plan
    if first_loop ==1:
        init_dist = dist
        first_loop = 0

    turnRight()
    rospy.loginfo("calling turnRight")

    while((dist - init_dist)/0.9187 < cmd_dist):
        continue
    
    rospy.loginfo("calling stop because d = %d and goal = %d",(dist-init_dist),cmd_dist)
    first_loop = 1
    Stop()
    new_Plan = 1
    isRunning_Plan = 0
 
def drive_dubins():
    """
    follow calculated dubins curve, stored in Moves[]            
    """
    global first_loop, init_dist,cmd_dist,dist,new_Plan,isRunning_Plan

    if first_loop ==1:
        init_dist = dist
        first_loop = 0

    ticks_per_cm = 1.0
    action = None
    if((Moves[0] == 0) or (Moves[0] == 2) or (Moves[0] == 5)): #LSL,LSR,LRL                                                                              
            # turn left                                                                                                                                     
            action = turnLeft
            ticks_per_cm = 0.8163
    elif((Moves[0] == 1) or (Moves[0] == 3) or (Moves[0] == 4)): #RSR,RSL,RLR                                                                            
            # turn right                                                                                                                                    
            action = turnRight
            ticks_per_cm = 0.9187

    while((dist - init_dist)/ticks_per_cm < Moves[1]):
        action()

    if(Moves[0] <= 3): #LSL,RSR,LSR,RSL                                                                                                                  
            # go forward                                                                                                                                    
            action = driveStraight
            ticks_per_cm = 0.845
    elif(Moves[0] == 4): #RLR                                                                                                                            
            # turn left                                                                                                                                     
            action = turnLeft
            ticks_per_cm = 0.8163
    elif(Moves[0] == 5): #LRL                                                                                                                            
            # turn right                                                                                                                                    
            action = turnRight
            ticks_per_cm = 0.9187
        
    while((dist - init_dist)/ticks_per_cm < Moves[1] + Moves[2]):
        action()

    if((Moves[0] == 0) or (Moves[0] == 3) or (Moves[0] == 5)):
            # turn left                                                                                                                                     
            action = turnLeft
            ticks_per_cm = 0.8163
    elif((Moves[0] == 1) or (Moves[0] == 2) or (Moves[0] == 5)):
            # turn right                                                                                                                                    
            action = turnRight
            ticks_per_cm = 0.9187

    while((dist - init_dist)/ticks_per_cm < Moves[1] + Moves[2] + Moves[3]):
        action()
    
    first_loop = 1
    rospy.loginfo("calling stop because d = %d and goal = %d  Moves=%d",(dist-init_dist),cmd_dist,Moves[1]+Moves[2]+Moves[3])
    Stop()
    new_Plan = 1
    isRunning_Plan = 0

#############################################
def driveStraight():
    global robot_setpoint_speed
    global robot_setpoint_steering
    if((robot_setpoint_steering == 0) and (robot_setpoint_speed == ROBOT_SPEED)):
        return
    else:
        robot_setpoint_speed = ROBOT_SPEED
        robot_setpoint_steering = 0
        pub.publish(ROBOT_SPEED,0)

def turnLeft():
    global robot_setpoint_speed
    global robot_setpoint_steering
    if((robot_setpoint_steering == -40) and (robot_setpoint_speed == ROBOT_SPEED)):
        return
    else:
        robot_setpoint_speed = ROBOT_SPEED
        robot_setpoint_steering = -40
        pub.publish(ROBOT_SPEED,-40)

def turnRight():
    global robot_setpoint_speed
    global robot_setpoint_steering
    if((robot_setpoint_steering == 29) and (robot_setpoint_speed == ROBOT_SPEED)):
        return
    else:
        robot_setpoint_speed = ROBOT_SPEED
        robot_setpoint_steering = 29
        pub.publish(ROBOT_SPEED,29)

def Stop():
    global robot_setpoint_speed
    global robot_setpoint_steering

    if((robot_setpoint_steering == 0) and (robot_setpoint_speed == 0)):
        return
    else:
        robot_setpoint_speed = 0
        robot_setpoint_steering = 0
        pub.publish(0,0)
#####################################################

def received_goal(data):
    global cmd_dist, cmd_theta,Moves,new_Plan,isRunning_Plan
    #if(data.d < 60) and (isRunning_Plan == 1):
    if False: # do not want this right now
        print "ignoring distance < 60"
        new_Plan = 0 # dont replan until current goal is reached
        return

    isRunning_Plan = 1 # bot is running a plan
    cmd_dist = data.d
    cmd_theta = data.th
    rospy.loginfo("in listener %d", data.opt)
    if(data.opt == 1):
        drive_straight()
    elif(data.opt == 2):
        drive_circle()
    elif(data.opt == 3):
        # Calculate current orientation of robot, taking line to target as the x-axis
        if(cmd_theta <0):
            alpha = cmd_theta + 360
        else:
            alpha = cmd_theta
        
        # final orientation is arbitrarily set to 45 degrees
        beta = 45
        # calculate dubins curves

        dubins(math.radians(alpha),math.radians(45),cmd_dist)
        #path = return_path_dubins(Moves[0],Moves[1],Moves[2],Moves[3])
        min_dubins = [Moves[0],Moves[1],Moves[2],Moves[3]]
        min_length = Moves[1] + Moves[2] + Moves[3]
                                                                                                                                                                  
        for beta in range(0,360,10):                                                                                                                                   
            # calculate dubins curves for a variety of approach angles. choose the shortest path                                                                       
            dubins(math.radians(alpha),math.radians(beta),cmd_dist)                                                                                                    
            length = Moves[1] + Moves[2] + Moves[3]                                                                                                                    
            if (length < min_length):                                                                                                                                  
                min_length = length                                                                                                                                    
                min_dubins = [Moves[0],Moves[1],Moves[2],Moves[3]]                                                                                                     
                Moves = min_dubins                                          
        print "dubins ",Moves[0],Moves[1],Moves[2],Moves[3]
        drive_dubins()
        

def odom_callback(data):
    global x,y,dist,theta
    x = data.x
    y = data.y
    dist = data.dist
    theta = data.angle



if __name__ == '__main__':
    rospy.init_node('planner',anonymous = True)
    pub.publish(0,0)
    """
    spins until a goal is received.
    goal is the form goal_msg
    with goal_msg.d = distance to goal (cm)
    goal_msg.th = angle to goal [-90,90] degrees
    """
    #rospy.init_node('goal_listener',anonymous=True)
    rospy.Subscriber("goal",goal_msg,received_goal) # call plan with data = goal_msg when a goal is seen on the "goal" 
    rospy.Subscriber("odom", odom_data, odom_callback)
    rospy.spin()
