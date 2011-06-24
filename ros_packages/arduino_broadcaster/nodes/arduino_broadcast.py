#!/usr/bin/env python

import roslib; roslib.load_manifest('arduino_broadcaster')
import rospy
import serial
from arduino_broadcaster.msg import arduino_data
import struct

def arduino_talker():
    pub = rospy.Publisher('ard_topic', arduino_data)
    rospy.init_node('arduino_talker')
    
    ser = serial.Serial("/dev/ttyUSB0", 115200)
    
    while not rospy.is_shutdown():
        init_byte = ser.read()
        if(ord(init_byte) == 0xFF):
            tcks = (ord(ser.read()) << 8) | ord(ser.read())  
            agl = ser.read(2)
            agl = struct.unpack("<h",agl)
                    
            pub.publish(tcks, agl[0])



if __name__ == '__main__':
    try:
        arduino_talker()
    except rospy.ROSInterruptException: pass
