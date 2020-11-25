#! /usr/bin/env python
import rospy
import time
import actionlib
from ardrone_as.msg import ArdroneAction, ArdroneGoal, ArdroneResult, ArdroneFeedback
from geometry_msgs.msg import Twist
from std_msgs.msg import Empty

PENDING = 0
ACTIVE = 1
DONE = 2
WARN = 3
ERROR = 4

nImage = 1

# definition of the feedback callback. This will be called when feedback
# is received from the action server
# it just prints a message indicating a new message has been received
def feedback_callback(feedback):
    global nImage
    print('[Feedback] image n.%d received'%nImage)
    nImage += 1

# initializes the action client node
rospy.init_node('drone_action_client')
rate = rospy.Rate(0.8)

pub_takeoff = rospy.Publisher('drone/takeoff', Empty, queue_size=1)
# empty_message = Empty()
while True: # repeat until break
    rospy.Rate(0.5).sleep() # wait a bit for the connection to establish
    connections = pub_takeoff.get_num_connections()
    print connections # when connections=0, the publisher hasn't yet made the connection to the topic; when connections=1, we have, and are ready to publish
    if connections > 0:
        # pub_takeoff.publish(empty_message)
        pub_takeoff.publish()
        rospy.Rate(0.5).sleep()
        rospy.loginfo("Taken Off")
        break
    else:
        rospy.Rate(0.5).sleep() # if the connection hasn't been established, wait a bit longer

'''
pub_takeoff = rospy.Publisher('/drone/takeoff', Empty, queue_size=1)
pub_takeoff.publish()
rate.sleep()
'''

pub_landing = rospy.Publisher('/drone/land', Empty, queue_size=1)

# create the connection to the action server
client = actionlib.SimpleActionClient('/ardrone_action_server', ArdroneAction)
# waits until the action server is up and running
client.wait_for_server()

# creates a goal to send to the action server
goal = ArdroneGoal()
goal.nseconds = 10 # indicates, take pictures along 10 seconds

# sends the goal to the action server, specifying which feedback function
# to call when feedback received
client.send_goal(goal, feedback_cb=feedback_callback)

move = Twist()
pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

state_result = client.get_state()
while state_result < DONE:
    move.linear.x = 0.5
    move.linear.y = 0
    pub.publish(move)
    rate.sleep()
    move.linear.x = 0
    move.linear.y = 0.5
    pub.publish(move)
    rate.sleep()
    move.linear.x = -0.5
    move.linear.y = 0
    pub.publish(move)
    rate.sleep()
    move.linear.x = 0
    move.linear.y = -0.5
    pub.publish(move)
    rate.sleep()
    state_result = client.get_state()
    rospy.loginfo("state_result: "+str(state_result))

move.linear.x = 0
move.linear.y = 0
pub.publish(move)

rospy.loginfo("[Result] State: "+str(state_result))
if state_result == ERROR:
    rospy.logerr("Something went wrong in the Server Side")
if state_result == WARN:
    rospy.logwarn("There is a warning in the Server Side")

pub_landing.publish()

rospy.loginfo("--- Finished ---")