#!/usr/bin/env python

# This code uses a Subscriber to the '/odom' topic and outputs the 
# robot's odometry readings every 1 seconds through loginfo

import rospy
from nav_msgs.msg import Odometry

class OdomSubscriber(object):

    def odom_cb(self, msg): 
        self._odom_data = msg

    def __init__(self, topic_name='/odom'):
        self._topic_name = topic_name
        self._odom_data = Odometry()
        self._sub = rospy.Subscriber(self._topic_name, Odometry, self.odom_cb)

    def get_odomdata(self):
        return self._odom_data

if __name__ == '__main__':
    rospy.init_node('odom_subscriber')
    odomsubscriber = OdomSubscriber()
    rate = rospy.Rate(1)

    ctrl_c = False
    def shutdownhook():
        global ctrl_c

        rospy.loginfo('Shutdown time!')
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        data = odomsubscriber.get_odomdata()
        rospy.loginfo(data)
        rate.sleep()
    
