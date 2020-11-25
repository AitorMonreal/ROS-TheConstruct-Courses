#!/usr/bin/env python

# This code uses a Publisher to the '/cmd_vel' topic and allows you to move the 
# robot either forwards, right, left, backwards, or stop - but entering the 
# commands into the code

import rospy
from geometry_msgs.msg import Twist

class MotionPublisher(object):

    def __init__(self):
        self._motion_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self._motion = Twist()
        self._linearspeed = 0.5
        self._angularspeed = 0.1

    def move_robot(self, direction):
        if direction == 'forwards':
            self._motion.linear.x = self._linearspeed
            self._motion.angular.z = 0.0
        elif direction == 'backwards':
            self._motion.linear.x = - self._linearspeed
            self._motion.angular.z = 0.0
        elif direction == 'right':
            self._motion.linear.x = 0.0
            self._motion.angular.z = - self._angularspeed
        elif direction == 'left':
            self._motion.linear.x = 0.0
            self._motion.angular.z = self._angularspeed
        elif direction == 'stop':
            self._motion.linear.x = 0.0
            self._motion.angular.z = 0.0
        else:
            pass
        
        self._motion_pub.publish(self._motion)

if __name__ == '__main__':
    rospy.init_node('motion_publisher')
    motionpublisher = MotionPublisher()

    rate = rospy.Rate(1)

    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        
        rospy.loginfo('Shutdown time!')
        ctrl_c = True
        motionpublisher.move_robot('stop')

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        motionpublisher.move_robot('forwards')
        rate.sleep()
        rate.sleep()
        rate.sleep()
        motionpublisher.move_robot('backwards')
        rate.sleep()
        rate.sleep()
        rate.sleep()
        rate.sleep()