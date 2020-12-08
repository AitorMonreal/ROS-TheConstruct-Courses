#!/usr/bin/env python

import rospy
from std_msgs.msg import Float64

class GurdyMotion(object):
    def __init__(self):
        self.long_r = rospy.Rate(1)
        self.short_r = rospy.Rate(1)
        self._pub_upper_M1 = rospy.Publisher('/gurdy/head_upperlegM1_joint_position_controller/command', Float64, queue_size = 1)
        self._pub_lower_M1 = rospy.Publisher('/gurdy/upperlegM1_lowerlegM1_joint_position_controller/command', Float64, queue_size = 1)
        self._pub_upper_M2 = rospy.Publisher('/gurdy/head_upperlegM2_joint_position_controller/command', Float64, queue_size = 1)
        self._pub_lower_M2 = rospy.Publisher('/gurdy/upperlegM2_lowerlegM2_joint_position_controller/command', Float64, queue_size = 1)
        self._pub_upper_M3 = rospy.Publisher('/gurdy/head_upperlegM3_joint_position_controller/command', Float64, queue_size = 1)
        self._pub_lower_M3 = rospy.Publisher('/gurdy/upperlegM3_lowerlegM3_joint_position_controller/command', Float64, queue_size = 1)
        self.upper_pub_value = Float64()
        self.lower_pub_value = Float64()
        self.ctrl_c = False

    def move_robot(self):
        self.stand_still()
        self.long_r.sleep()
        
        ctrl_c = False
        def shutdownhook():
            global ctrl_c

            rospy.loginfo('Shutdown time!')
            ctrl_c = True

        rospy.on_shutdown(shutdownhook)

        while not ctrl_c:
            self.close_joints()
            self.short_r.sleep()
            self.open_joints()
            self.short_r.sleep()

    def joint_publish(self, upper_val, lower_val):
        self._pub_upper_M1.publish(upper_val)
        self._pub_lower_M1.publish(lower_val)
        self._pub_upper_M2.publish(upper_val)
        self._pub_lower_M2.publish(lower_val)
        self._pub_upper_M3.publish(upper_val)
        self._pub_lower_M3.publish(lower_val)

    def stand_still(self):
        self.upper_pub_value.data = -1.55
        self.lower_pub_value.data = -1.57
        self.joint_publish(self.upper_pub_value, self.lower_pub_value)
    
    def open_joints(self):
        self.upper_pub_value.data = -1.2
        self.lower_pub_value.data = -0.785
        self.joint_publish(self.upper_pub_value, self.lower_pub_value)
    
    def close_joints(self):
        self.upper_pub_value.data = -1.55
        self.lower_pub_value.data = -2.356
        self.joint_publish(self.upper_pub_value, self.lower_pub_value)


if __name__ == '__main__':
    rospy.init_node('gurdy_motion')
    gurdymotion = GurdyMotion()
    
    ctrl_c = False
    def shutdownhook():
        global ctrl_c

        rospy.loginfo('Shutdown time!')
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        gurdymotion.move_robot()
