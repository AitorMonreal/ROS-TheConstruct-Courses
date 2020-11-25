#! /usr/bin/env python

import rospy
from services_quiz.srv import BB8CustomServiceMessage, BB8CustomServiceMessageResponse
from geometry_msgs.msg import Twist

def my_callback(request):
    for j in range(request.repetitions):
        for k in range(4):
            i = 0
            square_motion.linear.x = 0.2
            my_pub.publish(square_motion)
            while i < request.side:
                i = i+1
                rate.sleep()
            square_motion.linear.x = 0.0
            my_pub.publish(square_motion)
            rate.sleep()
            square_motion.angular.z = 0.45
            my_pub.publish(square_motion)
            rate.sleep()
            rate.sleep()
            square_motion.angular.z = 0.0
            my_pub.publish(square_motion)
            rate.sleep()

    my_response = BB8CustomServiceMessageResponse()
    my_response.success = True
    return my_response
    
rospy.init_node('service_server')
my_service = rospy.Service('/move_bb8_in_square_custom', BB8CustomServiceMessage, my_callback)
my_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
square_motion = Twist()
rate = rospy.Rate(1)
rospy.spin()