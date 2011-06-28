#!/usr/bin/env python

import roslib; roslib.load_manifest('ros_ard_to_fro')
import rospy
import serial
import time
import struct
import math
from ros_ard_to_fro.msg import odom_data
from ros_ard_to_fro.msg import drive_cmd
import robot_class
robot = None 
 
def recieved_velocity(nav_velocity_msg): #this function is called when the listener catches a message
    drv_spd = nav_velocity_msg.drive_speed
    str_agl = nav_velocity_msg.steer_angle
    rospy.loginfo("speed %d angle %d",drv_spd,str_agl)
    robot.set_steering(str_agl)
    robot.set_velocity(drv_spd)
    robot.send_arduino_packet()
    rospy.loginfo("sent speeds to arduino")


def initializer():
    global robot
    rospy.init_node('ros_ard_to_fro',anonymous=False)
   
    #the serial initialization goes here

    subs = rospy.Subscriber("vel_cmd", drive_cmd, recieved_velocity)
    pub = rospy.Publisher('odom', odom_data)
    robot = robot_class.Robot()

    x = 0.0
    y = 0.0
    angle = 0
    dist = 0.0
    
    waiter = rospy.Rate(60)

    while not rospy.is_shutdown():   #infinite loop
        robot.send_getpacket()
        #rospy.loginfo("Data requested")

        try:
            init_byte = robot.ser.read()
            if len(init_byte) != 0:
                if(ord(init_byte) == 0xFF):  #checking for byte from arduino
                    s = robot.ser.read(4)
                    if len(s) < 4:
                        continue
                    tcks,agl = struct.unpack(">hh",s)
                    
                    if agl>=360:
                        angle = agl%360
                    elif agl < 0:
                        angle = ((agl%360) - 360)
                    else:
                        angle = agl            
                    x += tcks*math.sin(agl)*100/75  # the 100/75 is the scaling factor for ticks->cm
                    y += tcks*math.cos(agl)*100/75 
                    dist += tcks*100.0/75.0
            
                    pub.publish(x,y,dist,angle)
                    waiter.sleep()
                #rospy.loginfo("Distance moved %f",dist)
        except serial.serialutil.SerialException:
            print "serial exception"
        except OSError:
            print "OS Error!"
            pass

if __name__ == '__main__':
    try:
        initializer()
    except rospy.ROSInterruptException: pass


#/////////////////////////////////////////////////////////
