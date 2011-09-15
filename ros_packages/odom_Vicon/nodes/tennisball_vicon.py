#!/usr/bin/env python
"""
This node publishes position of the tennis ball.
The pose is published to the topic 'Pose' with message type Pose.msg
"""
import roslib; roslib.load_manifest('odom_Vicon')
import rospy
import math
from bb_msgs.msg import Pose
from std_msgs.msg import String
from vicon_mocap.msg import Markers
from vicon_mocap.msg import Marker

pub = rospy.Publisher('ball', Pose )

def callback(data):
    """
    receives an array of markers and publishes pose of the rear axle's center
    """            
    pub.publish(9.099,8.667,0.0)
    """
    for marker in data.markers:
        if marker.marker_name == "ball1":
            x = marker.translation.x/10.0
            y = marker.translation.y/10.0
            pub.publish((x + 1000)/100.0,(y+ 1000)/100.0,0.0)
    """
    #print (x_car/100.0,y_car/100.0,theta_car)    

def listener():    
    rospy.init_node('tennisball_vicon')        
    rospy.Subscriber("vicon_recv_direct/markers",Markers,callback)
    print "initialized subscriber"
    rospy.spin()

if __name__ == '__main__':
    listener()
