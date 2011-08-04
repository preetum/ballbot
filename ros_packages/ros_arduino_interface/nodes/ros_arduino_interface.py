#!/usr/bin/env python

import roslib; roslib.load_manifest('ros_arduino_interface')
import rospy
import robot
from bb_msgs.msg import DriveCmd

def recievedDrivePacket(ballBot, drivePacket):
    '''
    This function is called when the listener catches a message
    '''
    speed = int(drivePacket.velocity * 100)
    angle = int(drivePacket.steerAngle * 180 / 3.141592654)
    ballBot.set_velocity(speed, angle)
    rospy.loginfo("Speed %d Angle %d" % (speed, angle))

def initializer():
    ballBot = robot.BaseController()
    rospy.init_node('ros_arduino_interface', anonymous=True)
    rospy.Subscriber("vel_cmd", DriveCmd,
                     lambda pkt: recievedDrivePacket(ballBot, pkt))
    rospy.spin()

if __name__ == '__main__':
    initializer()
