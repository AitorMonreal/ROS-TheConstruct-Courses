#! /usr/bin/env python

import rospy
from std_msgs.msg import Float64
from geometry_msgs.msg import Twist

class GurdyWalk(object):
    def __init__(self, topic='/cmd_vel'):
        self.long_r = rospy.Rate(1)
        self.short_r = rospy.Rate(1)
        self._pub_z_joint_M1 = rospy.Publisher('/gurdy/Z_M1_joint_position_controller/command', Float64, queue_size = 1)
        self._pub_upper_M1 = rospy.Publisher('/gurdy/head_upperlegM1_joint_position_controller/command', Float64, queue_size = 1)
        self._pub_lower_M1 = rospy.Publisher('/gurdy/upperlegM1_lowerlegM1_joint_position_controller/command', Float64, queue_size = 1)
        self._pub_z_joint_M2 = rospy.Publisher('/gurdy/Z_M2_joint_position_controller/command', Float64, queue_size = 1)
        self._pub_upper_M2 = rospy.Publisher('/gurdy/head_upperlegM2_joint_position_controller/command', Float64, queue_size = 1)
        self._pub_lower_M2 = rospy.Publisher('/gurdy/upperlegM2_lowerlegM2_joint_position_controller/command', Float64, queue_size = 1)
        self._pub_z_joint_M3 = rospy.Publisher('/gurdy/Z_M3_joint_position_controller/command', Float64, queue_size = 1)
        self._pub_upper_M3 = rospy.Publisher('/gurdy/head_upperlegM3_joint_position_controller/command', Float64, queue_size = 1)
        self._pub_lower_M3 = rospy.Publisher('/gurdy/upperlegM3_lowerlegM3_joint_position_controller/command', Float64, queue_size = 1)
        self.upper_pub_value = Float64()
        self.lower_pub_value = Float64()
        self.z_joint_pub_value = Float64()
        self.ctrl_c = False
        self.topic_name = topic
        self._sub_vel = rospy.Subscriber(self.topic_name, Twist, self.teleop_cb)
    
    def teleop_cb(self, msg):
        forwards_vel = msg.linear.x
        self.move_forwards(forwards_vel)
    
    def move_forwards(self):
        self.move_leg_forward(1)
        self.move_leg_forward(3)
        self.lift_leg(2)
        self.short_r.sleep()
        self.rotate_backwards_leg(1)
        self.rotate_backwards_leg(3)
        self.short_r.sleep()
        self.drop_leg(2)

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

    def single_leg_publish(self, leg_number, z_joint_val, upperleg_val, lowerleg_val):
        if leg_number == 1:
            self.M1_leg_publish(z_joint_val, upperleg_val, lowerleg_val)
        elif leg_number == 2:
            self.M2_leg_publish(z_joint_val, upperleg_val, lowerleg_val)
        elif leg_number == 3:
            self.M3_leg_publish(z_joint_val, upperleg_val, lowerleg_val)

    def all_legs_publish(self, z_joint_val, upperleg_val, lowerleg_val):
        self.M1_leg_publish(z_joint_val, upperleg_val, lowerleg_val)
        self.M2_leg_publish(z_joint_val, upperleg_val, lowerleg_val)
        self.M3_leg_publish(z_joint_val, upperleg_val, lowerleg_val)
    
    def M1_leg_publish(self, z_joint_val, upperleg_val, lowerleg_val):
        self._pub_z_joint_M1.publish(z_joint_val)
        self._pub_upper_M1.publish(upper_val)
        self._pub_lower_M1.publish(lower_val)
    def M2_leg_publish(self, z_joint_val, upperleg_val, lowerleg_val):
        self._pub_z_joint_M2.publish(z_joint_val)
        self._pub_upper_M2.publish(upper_val)
        self._pub_lower_M2.publish(lower_val)
    def M3_leg_publish(self, z_joint_val, upperleg_val, lowerleg_val):
        self._pub_z_joint_M3.publish(z_joint_val)
        self._pub_upper_M3.publish(upper_val)
        self._pub_lower_M3.publish(lower_val)

    def move_leg_forward(self, leg_number):
        self.lift_leg(leg_number)
        self.short_r.sleep()
        self.rotate_forwards_leg(leg_number)
        self.short_r.sleep()
        self.drop_leg(leg_number)
        self.short_r.sleep()
    
    def lift_leg(self, leg_number):
        self.upper_pub_value.data = -0.7
        self.lower_pub_value.data = -0.9
        self.vertical_publish(leg_number, self.upper_pub_value, self.lower_pub_value)
    def drop_leg(self, leg_number):
        self.upper_pub_value.data = -0.5
        self.lower_pub_value.data = -0.7
        self.vertical_publish(leg_number, self.upper_pub_value, self.lower_pub_value)

    def vertical_publish(self, leg_number, upperleg_val, lowerleg_val):
        if leg_number == 1:
            self.M1_vert_leg_publish(upperleg_val, lowerleg_val)
        elif leg_number == 2:
            self.M2_vert_leg_publish(upperleg_val, lowerleg_val)
        elif leg_number == 3:
            self.M3_vert_leg_publish(upperleg_val, lowerleg_val)
    def M1_vert_leg_publish(self, upperleg_val, lowerleg_val):
        self._pub_upper_M1.publish(upperleg_val)
        self._pub_lower_M1.publish(lowerleg_val)
    def M2_vert_leg_publish(self, upperleg_val, lowerleg_val):
        self._pub_upper_M2.publish(upperleg_val)
        self._pub_lower_M2.publish(lowerleg_val)
    def M3_vert_leg_publish(self, upperleg_val, lowerleg_val):
        self._pub_upper_M3.publish(upperleg_val)
        self._pub_lower_M3.publish(lowerleg_val)
    
    def rotate_forwards_leg(self, leg_number):
        self.z_joint_pub_value.data = 0.75
        self.rotational_publish(leg_number, self.z_joint_pub_value)
    def rotate_backwards_leg(self, leg_number):
        self.z_joint_pub_value.data = -0.75
        self.rotational_publish(leg_number, self.z_joint_pub_value)
    
    def rotational_publish(self, leg_number, z_joint_val):
        if leg_number == 1:
            self.M1_rot_leg_publish(z_joint_val)
        elif leg_number == 2:
            self.M2_rot_leg_publish(z_joint_val)
        elif leg_number == 3:
            self.M3_rot_leg_publish(z_joint_val)
    def M1_rot_leg_publish(self, z_joint_val):
        self._pub_z_joint_M1.publish(z_joint_val)
    def M2_rot_leg_publish(self, z_joint_val):
        self._pub_z_joint_M2.publish(z_joint_val)
    def M3_rot_leg_publish(self, z_joint_val):
        self._pub_z_joint_M3.publish(z_joint_val)

    def stand_still(self):
        self.z_joint_pub_value.data = 0
        self.upper_pub_value.data = -0.5
        self.lower_pub_value.data = -0.7
        self.all_joint_publish(self.z_joint_pub_value, self.upper_pub_value, self.lower_pub_value)
    
    def open_joints(self):
        self.z_joint_pub_value.data = -1.57
        self.upper_pub_value.data = -1.2
        self.lower_pub_value.data = -0.785
        self.joint_publish(self.z_joint_pub_value, self.upper_pub_value, self.lower_pub_value)
    
    def close_joints(self):
        self.z_joint_pub_value.data = -1.57
        self.upper_pub_value.data = -1.55
        self.lower_pub_value.data = -2.356
        self.joint_publish(self.z_joint_pub_value, self.upper_pub_value, self.lower_pub_value)


if __name__ == '__main__':
    rospy.init_node('gurdy_motion')
    gurdywalk = GurdyWalk()
    
    ctrl_c = False
    def shutdownhook():
        global ctrl_c

        rospy.loginfo('Shutdown time!')
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        gurdywalk.move_forwards()
