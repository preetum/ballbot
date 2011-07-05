#!/usr/bin/env python
"""
When given a goal, call globalplanner to plan path to goal
Then drive along that path
"""
import roslib; roslib.load_manifest('navigation')
import rospy
from navigation.msg import odom_data
from navigation.msg import goal_msg
from navigation.msg import drive_cmd
import globalplanner

""" 
position of Ballbot in global coordinate frame
"""
Ballbot_x = 304.8
Ballbot_y = 304.8
Ballbot_th = 0
Ballbot_encoderticks = 0
pub = rospy.Publisher('vel_cmd', drive_cmd)

globalplan = []
ROBOT_RADIUS   = 60.96  #in cm
ROBOT_SPEED    = 50.0  #in cm/s


def received_goal(data):
    global globalplan,Ballbot_x,Ballbot_y,Ballbot_th,encoder_ticks
    cmd_d = data.d
    cmd_th = data.th
    rospy.loginfo("received goal")
    globalplan = globalplanner.startPlanner(Ballbot_x,Ballbot_y,90.0,cmd_d,cmd_th)
    executeplan()

def odom_callback(data):
    """
    Callback for topic /odom. Update globals relating to Ballbot's odometry
    """
    global Ballbot_x,Ballbot_y,Ballbot_encoderticks,Ballbot_th
    Ballbot_x = 304.8 #+ data.x
    Ballbot_y = 304.8 #+ data.y
    Ballbot_encoderticks = data.dist
    Ballbot_th = data.angle

def executeplan():
    global globalplan
    for segment in globalplan:
        action = segment[0]
        distance = segment[1]

        if(action == 'R'):
            turnRight(distance)
        elif(action == 'L'):
            turnLeft(distance)
        elif(action == 'S'):
            driveStraight(distance)
        else:
            Stop()
    Stop()

#############################################
def driveStraight(distance):
    print "straight for ",distance
    ticks_per_cm = 0.845
    init_encoderticks = Ballbot_encoderticks
    pub.publish(ROBOT_SPEED,0)
    
    while ((Ballbot_encoderticks - init_encoderticks)/ticks_per_cm < distance):
        continue
    Stop()

def turnLeft(distance):
    print "left for ",distance
    ticks_per_cm = 0.8163
    init_encoderticks = Ballbot_encoderticks
    pub.publish(ROBOT_SPEED,-40)
    
    while ((Ballbot_encoderticks - init_encoderticks)/ticks_per_cm < distance):
        continue
    Stop()

def turnRight(distance):
    print "right for ",distance
    ticks_per_cm = 0.9187
    init_encoderticks = Ballbot_encoderticks
    pub.publish(ROBOT_SPEED,40)

    while ((Ballbot_encoderticks - init_encoderticks)/ticks_per_cm < distance):
        continue
    Stop()

def Stop():
    pub.publish(0,0)
#####################################################



if __name__ == '__main__':
    rospy.init_node('navigation',anonymous = True)
    pub.publish(0,0)
    """
    spins until a goal is received.
    goal is the form goal_msg
    with goal_msg.d = distance to goal (cm)
    goal_msg.th = angle to goal [-90,90] degrees
    """
    #rospy.init_node('goal_listener',anonymous=True)
    rospy.Subscriber("goal",goal_msg,received_goal) # call plan with data = goal_msg when a goal is seen on the "goal" topic
    rospy.Subscriber("odom", odom_data, odom_callback)
    rospy.spin()
    main()
