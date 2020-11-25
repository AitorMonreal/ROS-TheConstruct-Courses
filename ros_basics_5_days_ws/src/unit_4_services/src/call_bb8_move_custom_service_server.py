#! /usr/bin/env python

import rospy
import sys
from my_custom_srv_msg_pkg.srv import MyCustomServiceMessage, MyCustomServiceMessageRequest

rospy.init_node('service_client')
rospy.wait_for_service('/move_bb8_in_circle_custom')
s = rospy.ServiceProxy('/move_bb8_in_circle_custom', MyCustomServiceMessage)
s_object = MyCustomServiceMessageRequest()
s_object.duration = 5.0
result = s(s_object)
print result