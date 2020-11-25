#!/usr/bin/env python

# This code uses a Publisher to the '/cmd_vel' topic and allows you to move the 
# robot either forwards, right, left, backwards, or stop - but entering the 
# commands into the code

import rospy
from geometry_msgs.msg import Twist

class CmdVelPub(object):

    def __init__(self):
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size = 1)
        self.motion = Twist()
        self.linearspeed = 0.5
        self.angularspeed = 0.1

    def move_robot(self, direction):
        if direction == 'forwards':
            self.motion.linear.x = self.linearspeed
            self.motion.angular.z = 0.0
        elif direction == 'backwards':
            self.motion.linear.x = -self.linearspeed
            self.motion.angular.z = 0.0
        if direction == 'right':
            self.motion.linear.x = 0.0
            self.motion.angular.z = self.angularspeed
        if direction == 'left':
            self.motion.linear.x = 0.0
            self.motion.angular.z = self.angularspeed
        if direction == 'stop':
            self.motion.linear.x = 0.0
            self.motion.angular.z = 0.0
        else:
            pass
        
        self.cmd_pub.publish(self.motion)

if __name__ == '__main__':
    rospy.init_node('cmd_vel_publisher')
    cmd_motion = CmdVelPub()

    rate = rospy.Rate(1)

    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        global pub

        rospy.loginfo('Shutdown time!')
        ctrl_c = True
        cmd_motion.move_robot('stop')

    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        cmd_motion.move_robot('forwards')
        rate.sleep()
        rate.sleep()
        rate.sleep()
        cmd_motion.move_robot('backwards')
        rate.sleep()
        rate.sleep()
        rate.sleep()
        rate.sleep()