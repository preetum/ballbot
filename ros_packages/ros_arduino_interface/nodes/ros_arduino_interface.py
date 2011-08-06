#!/usr/bin/env python

import roslib; roslib.load_manifest('ros_arduino_interface')
import rospy
import robot
from bb_msgs.msg import DriveCmd
import imu_communicator

def recieved_drive_packet(ballBot, drivePacket):
    '''
    This function is called when the listener catches a message
    '''
    speed = int(drivePacket.velocity * 100)
    angle = int(drivePacket.steerAngle * 180 / 3.141592654)
    ballBot.set_velocity(speed, angle)
    rospy.loginfo("Speed %d Angle %d" % (speed, angle))

def send_heading_to_arduino(ballbot, imu):
    waiter = rospy.rate(60)
    while True:
        ballbot.sync_odometry(imu.headingBAMS)
        waiter.sleep

def initializer():
    ballBot = robot.BaseController()
    rospy.init_node('ros_arduino_interface', anonymous=True)
    rospy.Subscriber("vel_cmd", DriveCmd,
                     lambda pkt: recieved_drive_packet(ballBot, pkt))
    imu = imu_communicator.IMU() # Initialize IMU
    send_heading_to_arduino(ballbot, imu)

if __name__ == '__main__':
    initializer()
