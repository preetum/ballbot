#!/usr/bin/env python

import roslib; roslib.load_manifest('odom_xytheta')
import rospy
import serial
from odom_xytheta.msg import odom_data
import struct
import math

def arduino_talker():
    pub = rospy.Publisher('odom', odom_data)
    rospy.init_node('odom_publisher')
    x = 0
    y = 0
    angle = 0
    ser = serial.Serial("/dev/ttyUSB0", 115200)
    
    while not rospy.is_shutdown():
        init_byte = ser.read()
        if(ord(init_byte) == 0xFF):
            tcks = (ord(ser.read()) << 8) | ord(ser.read())  
            agl = ser.read(2)
            agl = struct.unpack("<h",agl)
            angle = agl[0]
            
            x += tcks*math.sin(agl[0])*100/75  # the 100/75 is the scaling factor for ticks->cm
            y += tcks*math.cos(agl[0])*100/75 
           
            pub.publish(x,y,angle)
            # x,y in cm | angle in degrees


if __name__ == '__main__':
    try:
        arduino_talker()
    except rospy.ROSInterruptException: pass
