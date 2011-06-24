#!/usr/bin/env python
import roslib; roslib.load_manifest('arduino')
import rospy
from std_msgs.msg import Int16MultiArray, MultiArrayDimension
import serial

def talker():
    pub = rospy.Publisher('/arduino1/analogs', Int16MultiArray)
    rospy.init_node('analogpublisher')
    ser = serial.Serial("/dev/ttyUSB1", 115200)
    while not rospy.is_shutdown():
        f = ser.readline().split()
        if len(f) == 7 and f[0] == "ANALOGS":
            msg = Int16MultiArray()
            msg.layout.dim = [MultiArrayDimension('sensors', 6, 6)]
            msg.data = [int(v) for v in f[1:67]]
            pub.publish(msg)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
