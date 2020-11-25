#! /usr/bin/env python

import rospy
# Import the service message used by the service /trajectory_by_name
from std_srvs.srv import Empty, EmptyRequest
import sys

# Initialise a ROS node with the name service_client
rospy.init_node('bb8_circle_client')
# Wait for the service client /move_bb8_in_circle to be running
rospy.wait_for_service('/move_bb8_in_circle')
# Create the connection to the service
s = rospy.ServiceProxy('/move_bb8_in_circle', Empty)
# Create an object of type TrajByNameRequest
s_object = EmptyRequest()
# Fill the variables of this object with the desired values
# none necessary since the service takes no Args
# Send through the connection the name of the trajectory to be executed by the robot
result = s(s_object)
# Print the result given by the service called
print result