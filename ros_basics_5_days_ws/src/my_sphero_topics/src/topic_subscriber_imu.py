#!/usr/bin/env python

# This code uses a Subscriber to the '/sphero/imu/data3' topic and outputs the 
# robot's imu readings every 1 seconds through loginfo

import rospy
from sensor_msgs.msg import Imu
from my_sphero_topics.src.topic_subscriber_imu import IMUTopicReader

class IMUTopicReader(object):

    def callback(self, msg): 
        self.imu_data = msg
        #rospy.logdebug(msg)

    def __init__(self, topic_name='/sphero/imu/data3'):
        self.topic_name = topic_name
        self.imu_data = Imu()
        self.sub = rospy.Subscriber(self.topic_name, Imu, self.callback)

    def get_imudata(self):
        return self.imu_data

if __name__ == '__main__':
    rospy.init_node('imu_subscriber')
    read_imu = IMUTopicReader()
    rate = rospy.Rate(1)

    ctrl_c = False
    def shutdownhook():
        global ctrl_c

        rospy.loginfo('Shutdown time!')
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        data = read_imu.get_imudata()
        rospy.loginfo(data)
        rate.sleep()
    