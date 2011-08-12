#!/usr/bin/env python

import roslib; roslib.load_manifest('ros_arduino_interface')
import rospy
import robot
from bb_msgs.msg import DriveCmd, Odometry
import imu_communicator

pi  = 3.141592654

def binaryangle_to_radians(BAMS):
    return float(BAMS)*pi/32768.0

def counts_to_meters(counts):
    return float(counts)*0.0049

def recieved_drive_packet(ballBot, drivePacket):
    '''
    This function is called when the listener catches a message
    '''
    speed = int(drivePacket.velocity * 100)
    angle = int(drivePacket.steerAngle * 180 / 3.141592654)
    ballBot.set_velocity(speed, angle)
    rospy.loginfo("Speed %d Angle %d" % (speed, angle))

def send_heading_to_arduino(ballBot, imu, odometryPublisher):
    waiter = rospy.Rate(60)
    while True:
        ballBot.sync_odometry(imu.headingBAMS)
        odometry_broadcast(ballBot, odometryPublisher, imu.gyro_z)
        waiter.sleep()

def odometry_broadcast(ballBot, odometryPublisher, imu_angular_velocity):
    heading = binaryangle_to_radians(ballBot.angle)
    distance = counts_to_meters(ballBot.counts)
    distance_delta = counts_to_meters(ballBot.counts_delta)

    odometryPublisher.publish(distance, distance_delta, \
                              heading, imu_angular_velocity)
    
def initializer():
    ballBot = robot.BaseController()
    rospy.init_node('ros_arduino_interface', anonymous=True)
    #odometryPublisher = rospy.Publisher('odometry', Odometry)
    rospy.Subscriber("vel_cmd", DriveCmd,
                     lambda pkt: recieved_drive_packet(ballBot, pkt))
    #imu = imu_communicator.IMU() # Initialize IMU
    #send_heading_to_arduino(ballBot, imu, odometryPublisher)
    rospy.spin()		
if __name__ == '__main__':
    initializer()
