#!/usr/bin/env python

import rospy
from std_srvs.srv import Trigger, TriggerRequest
import sys

rospy.init_node('crash_service_client', log_level=rospy.WARN)
rospy.wait_for_service('/crash_service')
service = rospy.ServiceProxy('/crash_service', Trigger)
service_object = TriggerRequest()

rate = rospy.Rate(10)

ctrl_c = False
def shutdownhook():
    global ctrl_c 
    rospy.loginfo('Shutdown time!')
    ctrl_c = True

rospy.on_shutdown(shutdownhook)

while not ctrl_c:
    result = service(service_object)
    if result.success:
        rospy.logwarn("Success =="+str(result.success)) # print the result given by the service called
        rospy.logwarn("Direction To Go=="+str(result.message)) # print the result given by the service called
    else:
        rospy.loginfo("Success =="+str(result.success)) # print the result given by the service called
        rospy.loginfo("Direction To Go=="+str(result.message)) # print the result given by the service called
    #rate.sleep()