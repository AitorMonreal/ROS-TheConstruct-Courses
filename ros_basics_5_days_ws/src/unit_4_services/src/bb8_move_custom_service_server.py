#! /usr/bin/env python

import rospy
from my_custom_srv_msg_pkg.srv import MyCustomServiceMessage, MyCustomServiceMessageResponse
from geometry_msgs.msg import Twist

def my_callback(request):
    print "Request Data==> duration="+str(request.duration)
    i = 0
    while i < request.duration: #request.duration is the Argument taken by the service message when called, as detailed in MyCustomServiceMessage
        move_circle.linear.x = 0.2
        move_circle.angular.z = 0.2
        my_pub.publish(move_circle)
        rate.sleep()
        i = i+1
    else:
        move_circle.linear.x = 0
        move_circle.angular.z = 0
        my_pub.publish(move_circle)
    my_response = MyCustomServiceMessageResponse()
    my_response.success = True
    return my_response

rospy.init_node('service_server')
my_service = rospy.Service('/move_bb8_in_circle_custom', MyCustomServiceMessage, my_callback)
move_circle = Twist()
my_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
rate = rospy.Rate(1)
rospy.spin()
