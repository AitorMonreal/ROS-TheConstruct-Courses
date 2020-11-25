#! /usr/bin/env python

import rospy
from std_srvs.srv import Empty, EmptyResponse # you import the service message python classes generated from Empty.srv.
from geometry_msgs.msg import Twist

def my_callback(request):
    print 'Hello'
    rospy.loginfo("The Service move_bb8_in_circle has been called")
    move_circle.linear.x = 0.2
    move_circle.angular.z = 0.2
    my_pub.publish(move_circle)
    rospy.loginfo("Finished service move_bb8_in_circle")
    return EmptyResponse() # the service Response class, in this case EmptyResponse
    #return MyServiceResponse(len(request.words.split())) 

rospy.init_node('bb8_circle') 
my_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
move_circle = Twist()
my_service = rospy.Service('/move_bb8_in_circle', Empty , my_callback) # create the Service called /move_bb8_in_circle with the defined callback
rospy.loginfo("Service /move_bb8_in_circle Ready")
rospy.spin() # maintain the service open.

# By running 'roslaunch' on the launch file that starts the 'bb8_circle' node, we create a ROS Service called '/move_bb8_in_circle'. 
# From the Shell we then call the service using 'rosservice call /move_bb8_in_circle'. When this service is called it prints
# "The Service move_bb8_in_circle has been called" and publishes the new 'move_circle' parameter values to the previously created Publisher,
# 'my_pub', which publishes to the '/cmd_vel' message