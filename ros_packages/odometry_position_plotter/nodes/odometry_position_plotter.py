#!/usr/bin/env python

import roslib; roslib.load_manifest('odometry_position_plotter')
import rospy
import math
from bb_msgs.msg import Odometry, Scatter
first = True
x=0
y=0
z=0
pi  = 3.141592654

def received_odometry(odometry_data, scatterPublisher):
    '''
    This function is called when the listener catches a message
    '''
    global first, x, y, z
    if(first):
	x = 0
	y = 0
	z = 0
	first = False
	scatterPublisher.publish(x,y,z)
    else:
        x += odometry_data.distance_delta*math.cos(odometry_data.heading)
	y += odometry_data.distance_delta*math.sin(odometry_data.heading)
	
	scatterPublisher.publish(x, y, z)
    
def initializer():
    rospy.init_node('odometry_position_plotter', anonymous=True)
    scatterPublisher = rospy.Publisher('scatter_data', Scatter)
    rospy.Subscriber("odometry", Odometry, \
                     lambda odometry_data: \
                     received_odometry(odometry_data, scatterPublisher))
    rospy.spin()

if __name__ == '__main__':
    initializer()
