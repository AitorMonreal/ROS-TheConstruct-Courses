#!/usr/bin/env python

# This code uses a Subscriber to the '/odom' topic and outputs the 
# robot's odometry readings every 1 seconds through loginfo

import rospy
from nav_msgs.msg import Odometry

class OdomTopicReader(object):

    def callback(self, msg): 
        self.odom_data = msg
        #rospy.logdebug(msg)

    def __init__(self, topic_name='/odom'):
        self.topic_name = topic_name
        self.odom_data = Odometry()
        self.sub = rospy.Subscriber(self.topic_name, Odometry, self.callback)

    def get_odomdata(self):
        return self.odom_data

if __name__ == '__main__':
    rospy.init_node('odom_subscriber')
    read_imu = OdomTopicReader()
    rate = rospy.Rate(1)

    ctrl_c = False
    def shutdownhook():
        global ctrl_c

        rospy.loginfo('Shutdown time!')
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        data = read_imu.get_odomdata()
        rospy.loginfo(data)
        rate.sleep()
    