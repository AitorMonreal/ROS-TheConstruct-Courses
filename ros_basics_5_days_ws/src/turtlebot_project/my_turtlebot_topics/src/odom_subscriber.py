#!/usr/bin/env python

import rospy

from nav_msgs.msg import Odometry
from motion_publisher import MotionPublisher

class OdomSubscriber(object):
    def __init__(self):
        self._sub = rospy.Subscriber('/odom', Odometry, self.odom_cb)
        self._odom_data = Odometry()

    def odom_cb(self, msg):
        self._odom_data = msg.pose.pose.position.x
        print self._odom_data

if __name__ == '__main__':
    rospy.init_node('odom_subscriber')
    motionpublisher = MotionPublisher()
    odomsubscriber = OdomSubscriber()

    rate = rospy.Rate(1)

    ctrl_c = False
    def shutdownhook():
        # works better than the rospy.is_shut_down()
        global ctrl_c
        global pub

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

