#!/usr/bin/env python

import roslib; roslib.load_manifest('cmd_vel_simulator')
import rospy
#from geometry_msgs.msg import Twist
from cmd_vel_simulator.msg import goal_msg

def talker():
    pub = rospy.Publisher('goal', goal_msg)
    rospy.init_node('talker_simu_goal')
    #t = Twist()
    pub.publish(320,45,2)
    rospy.loginfo("sent goal")
    rospy.spin()
    """
    while not rospy.is_shutdown():
        
        '''
        t.linear.x = 1.0
        t.linear.y = -0.577
        t.linear.z = 0
        t.angular.x = 0
        t.angular.y = 0
        t.angular.z = 0
        '''
        drv_speed = 1
        steer = -40

        pub.publish(drv_speed,steer)
        rospy.sleep(1.0)
      """
if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException: pass
