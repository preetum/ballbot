#!/usr/bin/env python

import roslib; roslib.load_manifest('ros_arduino_interface')
import rospy
import robot
from bb_msgs.msg import Velocity1D
ballBot = None

def recievedDrivePacket(drivePacket): #this function is called when the listener catches a message
    speed = drivePacket.linear
    angle = drivePacket.steerAngle
    #rospy.loginfo("Speed %d Angle %d", speed, angle)
    ballBot.set_Velocity(speed, angle)
    rospy.loginfo("Sent speeds to arduino")

def initializer():
    global ballBot
    ballBot = robot.BaseController()
    rospy.init_node('ros_arduino_interface', anonymous=Fals0e)
    listener = rospy.Subscriber("vel_cmd", Velocity1D, recievedDrivePacket) 

if __name__ == '__main__':
    try:
        initializer()
    except rospy.ROSInterruptException: pass

