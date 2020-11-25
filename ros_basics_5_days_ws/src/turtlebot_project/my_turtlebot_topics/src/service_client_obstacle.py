#!/usr/bin/env python
import rospy
from std_srvs.srv import Trigger, TriggerRequest
from std_msgs.msg import Empty
import sys

rospy.init_node('obstacle_service_client', log_level=rospy.WARN)
rospy.wait_for_service('/obstacle')
service = rospy.ServiceProxy('/obstacle', Trigger)
service_object = TriggerRequest() # it is an Empty request, hence no additional data is needed

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