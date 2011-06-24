#!/usr/bin/env python

import roslib; roslib.load_manifest('cmd_vel_simulator')
import rospy
from geometry_msgs.msg import Twist

def talker():
    pub = rospy.Publisher('cmd_vel', Twist)
    rospy.init_node('talker_simu_cmd_vel')
    t = Twist()
    while not rospy.is_shutdown():
        #str = "hello world %s"%rospy.get_time()
        #rospy.loginfo(str)
        t.linear.x = 1.0
        t.linear.y = -0.577
        t.linear.z = 0
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = 0
        pub.publish(t)
        rospy.sleep(1.0)

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
