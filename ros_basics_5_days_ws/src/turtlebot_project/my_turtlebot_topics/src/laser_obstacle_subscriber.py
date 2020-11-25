#!/usr/bin/env python

import rospy
from sensor_msgs.msg import LaserScan
from motion_publisher import MotionPublisher

class LaserSubscriber(object):
    def __init__(self):
        self._sub = rospy.Subscriber('/kobuki/laser/scan', LaserScan, self.laser_cb)
        self._laser_data = LaserScan()
    
    def laser_cb(self, msg):
        self._laser_data = msg

    def obstacle_direction(self):
        size = len(self._laser_data.ranges)
        right_laser = self._laser_data.ranges[0]
        front_laser = self._laser_data.ranges[int(size/2)]
        left_laser = self._laser_data.ranges[size-1]
        print [left_laser, front_laser, right_laser]
        
        #significative = (left_laser < 1.5) or (front_laser < 0.5) or (right_laser < 0.5)
        #print significative
        
        #if significative:
        front_msg = False
        left_msg = False
        right_msg = False
        if front_laser < 1.0:
            # The obstacle is in front of the robot
            front_msg = True            
            
        if left_laser < 0.5:
            # The obstacle is to the left of the robot
            left_msg = True
            
        if right_laser < 0.5:
            # The obstacle is to the left of the robot
            right_msg = True

        else:
            message = "nothing"
        
        diff = left_laser-right_laser
        
        return self.convert_to_dict(front_msg, left_msg, right_msg, diff)

    def convert_to_dict(self, front_msg, left_msg, right_msg, diff):
        # Converts the given message to a dictionary telling in which direction there is a detection
        detect_dict = {}
        detect_dict = {"front":front_msg,
                        "left":left_msg,
                        "right":right_msg,
                        "diff":diff}
        return detect_dict

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

