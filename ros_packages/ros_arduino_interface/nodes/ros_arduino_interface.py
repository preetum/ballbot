#!/usr/bin/env python

import roslib; roslib.load_manifest('ros_arduino_interface')
import rospy
from robot import BaseController
from bb_msgs.msg import DriveCmd, Odometry, OdometryStamped, BallPickup
from std_msgs.msg import Header
#import imu_communicator
import struct, thread, math

def binaryangle_to_radians(BAMS):
    return float(BAMS)*math.pi/32768.0

def counts_to_meters(counts):
    return float(counts)*0.0049

def recieved_drive_packet(ballBot, drivePacket):
    '''
    This function is called when the listener catches a message
    '''
    speed = int(drivePacket.velocity * 100)
    angle = int(drivePacket.steerAngle * 180.0 / math.pi)
    ballBot.set_velocity(speed, angle)
    #rospy.loginfo("Speed %d Angle %d" % (speed, angle))

def recieved_ballpickup_packet(ballBot,ballpickupPacket):
    '''
    This function is called when a ballpickup message is seen
    '''
    ballBot.set_pickup(ballpickupPacket.direction)
    #rospy.loginfo("Ball Pickup! Direction %d" % ballpickupPacket.direction)

lastHeading = None
def odometry_callback(packet, pub):
  '''
  Called whenever a full packet is received back from the arduino
  '''

  if len(packet.data) > 0 and \
        ord(packet.data[0]) == BaseController.CMD_SYNC_ODOMETRY:
    cmd, counts, counts_delta, yaw, timestamp = \
        struct.unpack('>BllhL', packet.data)
    #print '%d\t%d\t%d' % (counts, counts_delta, timestamp)

    # Calculate change in heading
    global lastHeading
    heading = yaw #binaryangle_to_radians(imu.headingBAMS)

    if lastHeading is None:
        lastHeading = heading
    heading_delta = heading - lastHeading
    lastHeading = heading

    # Publish odometry message
    dist = counts_to_meters(counts)
    dist_delta = counts_to_meters(counts_delta)
    msg = OdometryStamped()
    msg.header.stamp = rospy.Time.now()
    msg.odometry = Odometry(dist, dist_delta, heading, heading_delta)
    pub.publish(msg)


def main():
    rospy.init_node('ros_arduino_interface', anonymous=True)

    ballBot = BaseController(port='/dev/ttyO3')
    odometryPublisher = rospy.Publisher('odometry', OdometryStamped)
    rospy.Subscriber("vel_cmd", DriveCmd,
                     lambda pkt: recieved_drive_packet(ballBot, pkt))

    rospy.Subscriber("ball_pickup",BallPickup,
                     lambda pkt: recieved_ballpickup_packet(ballBot,pkt))

    #imu = imu_communicator.IMU() # Initialize IMU

    # Start serial read thread
    thread.start_new_thread(ballBot.serial_read_thread,
        (lambda pkt: odometry_callback(pkt, odometryPublisher),))

    # Spin, process callbacks
    rospy.spin()

    # On quit, stop the robot and turn off odometry sending
    ballBot.reset()

if __name__ == '__main__':
    main()
