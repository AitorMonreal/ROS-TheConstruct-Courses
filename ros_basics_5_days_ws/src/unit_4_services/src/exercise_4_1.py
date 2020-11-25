#! /usr/bin/env python

import rospy
# Import the service message used by the service /trajectory_by_name
from iri_wam_reproduce_trajectory.srv import ExecTraj, ExecTrajRequest
import sys

# Initialise a ROS node with the name unit_4_srv
rospy.init_node('unit_4_srv')
# Wait for the service client /execute_trajectory to be running - started by the first node in the launch file
rospy.wait_for_service('/execute_trajectory')

import rospkg
rospack = rospkg.RosPack()
# This rospack.get_path() works in the same way as $(find name_of_package) in the launch files.
# Get path to the trajectory name
traj = rospack.get_path('iri_wam_reproduce_trajectory') + "/config/get_food.txt"

# Create the connection to the service
traj_by_name_service = rospy.ServiceProxy('/execute_trajectory', ExecTraj)
# Create an object of type ExecTrajRequest
traj_by_name_object = ExecTrajRequest()
# Fill the variable file of this object with the desired file path of the desired motion
traj_by_name_object.file = traj
# Send through the connection the name of the trajectory to be executed by the robot
result = traj_by_name_service(traj_by_name_object)
# Print the result given by the service called
print result