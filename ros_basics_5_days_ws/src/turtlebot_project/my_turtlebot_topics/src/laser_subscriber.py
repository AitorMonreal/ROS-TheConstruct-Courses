#!/usr/bin/env python

import rospy

from sensor_msgs.msg import LaserScan
from motion_publisher import MotionPublisher

class LaserSubscriber(object):
    def __init__(self):
        self._sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.laser_cb)
        self._laser_data = LaserScan()

    def laser_cb(self, msg):
        self._laser_data = msg.ranges
        # print self._laser_data[180]
    
    def get_laser_data(self):
        return self._laser_data

if __name__ == '__main__':
    rospy.init_node('lidar_subscriber')
    motionpublisher = MotionPublisher()
    lasersubscriber = LaserSubscriber()

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

