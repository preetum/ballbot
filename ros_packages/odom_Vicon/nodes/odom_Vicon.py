#!/usr/bin/env python
"""
This node publishes pose (x,y,theta) of the rear axle's center as observed by the Vicon system.
The pose is published to the topic 'Pose' with message type Pose.msg
"""
import roslib; roslib.load_manifest('odom_Vicon')
import rospy
import math
from bb_msgs.msg import Pose
from std_msgs.msg import String
from vicon_mocap.msg import Markers
from vicon_mocap.msg import Marker

pub = rospy.Publisher('odometry', Pose )
def callback(data):
    """
    receives an array of markers and publishes pose of the rear axle's center
    """             
    for marker in data.markers:
        if marker.marker_name == "Marker1":
            x1 = marker.translation.x/10.0
            y1 = marker.translation.y/10.0
        elif marker.marker_name == "Marker2":
            x2 = marker.translation.x/10.0
            y2 = marker.translation.y/10.0
        elif marker.marker_name == "Marker3":
            x3 = marker.translation.x/10.0
            y3 = marker.translation.y/10.0
    length13 = math.sqrt((x1-x3)*(x1-x3) + (y1-y3)*(y1-y3))
    xmid = (x1 + x2)/2
    ymid = (y1 + y2)/2

    y21 = y2 - y1
    y31 = y3 - y1
    x21 = x2 - x1
    x31 = x3 - x1
    
    # calculate (x_car,y_car,theta_car) for the center of the rear axle
    x_car = xmid - 29.21*y21*length13/(y31*x21 - y21*x31)
    y_car = ymid - 29.21*x21*length13/(x31*y21 - x21*y31)
    theta_car = math.atan2(y1-y3,x1-x3)%(2*math.pi)

    # transform such that center of vicon system is at (10.0,10.0)
    x_car += 1000.0
    y_car += 1000.0
    pub.publish(x_car/100.0,y_car/100.0,theta_car)  
    #print (x_car/100.0,y_car/100.0,theta_car)    

def listener():    
    rospy.init_node('Vicon_pose')        
    rospy.Subscriber("vicon_recv_direct/markers",Markers , callback)
    print "initialized subscriber"
    rospy.spin()

if __name__ == '__main__':
    listener()
