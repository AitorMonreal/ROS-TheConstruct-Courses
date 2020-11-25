#! /usr/bin/env python

import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageRequest
import sys

rospy.init_node('service_client')
rospy.wait_for_service('/move_bb8_in_square_custom')
s = rospy.ServiceProxy('/move_bb8_in_square_custom', BB8CustomServiceMessage)
s_object = BB8CustomServiceMessageRequest()
s_object.side = 3
s_object.repetitions = 2
result = s(s_object)
print result

s_object.side = 4.4
s_object.repetitions = 1
result = s(s_object)
print result