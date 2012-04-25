#!/usr/bin/env python
"""
This node publishes pose (x,y,theta) of the rear axle's center as observed by the Vicon system.
The pose is published to the topic 'pose' with message type Pose.msg

This is for the "new" markers (subject name Ballbot_new)
where "yaw" axis is the heading
x, y are in centimeters
theta is in radians [0,2*pi] where 0 == facing down the x axis
"""
import roslib; roslib.load_manifest('odom_Vicon')
import rospy
import math
from bb_msgs.msg import Pose
from std_msgs.msg import String
from geometry_msgs.msg import TransformStamped
from tf.transformations import euler_from_quaternion

pub = rospy.Publisher('pose', Pose)

def callback(data):
    x = data.transform.translation.x * 100.0
    y = data.transform.translation.y * 100.0
    q = [data.transform.rotation.x, data.transform.rotation.y,
         data.transform.rotation.z, data.transform.rotation.w]
    angles = euler_from_quaternion(q)
    yaw = angles[2] + math.pi

    pub.publish(x, y, yaw)

def listener():    
    rospy.init_node('Vicon_pose')
    rospy.Subscriber("vicon_recv_direct/output", TransformStamped, callback)
    print "initialized subscriber"
    rospy.spin()

if __name__ == '__main__':
    listener()
