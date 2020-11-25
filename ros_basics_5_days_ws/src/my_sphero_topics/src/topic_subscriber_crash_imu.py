#!/usr/bin/env python

# Modified from the IMU Subscriber code:
# This code uses a Subscriber to the '/sphero/imu/data3' topic and tell us from which 
# direction the crash has occurred. It does this by reading the maximum linear acceleration
# direction (x, y, z) - only if larger than a threshold?

import rospy
from sensor_msgs.msg import Imu

class IMUTopicReader(object):

    def callback(self, msg): 
        self._imudata = msg
        #rospy.logdebug(msg)

    def __init__(self, topic_name='/sphero/imu/data3'):
        self._topic_name = topic_name
        self._imudata = Imu()
        self._sub = rospy.Subscriber(self._topic_name, Imu, self.callback)
        self._threshold = 7.0

    # def get_imudata(self):
    #     return self._imudata

    def crash_direction_detection(self):
        x_accel = self._imudata.linear_acceleration.x
        y_accel = self._imudata.linear_acceleration.y
        z_accel = self._imudata.linear_acceleration.z
        
        acc_list = [x_accel, y_accel, z_accel]
        max_acc_index = acc_list.index(max(acc_list))
        positive = acc_list[max_acc_index] >= 0  # returns True if true, and False if false
        significative = abs(acc_list[max_acc_index]) > self._threshold
        
        # If the current max acceleration reading is above our threshold, then we have a crash

        if significative:
            if max_acc_index == 0:
                # The crash as been along the x-axis, eg from the left or from the right
                #rospy.logwarn("[X="+str(x_accel))
                #rospy.loginfo("Y="+str(y_accel)+", Z="+str(z_accel)+"]")
                if positive: 
                    # The acceleration felt by sphero from the crash is positive, hence towards the
                    # left, hence sphero has crashed onto the right wall
                    message = 'right'
                else:
                    message = 'left'
            
            if max_acc_index == 1:
                # The crash as been along the y-axis, eg from the back or from the front
                #rospy.logwarn("[Y="+str(y_accel))
                #rospy.loginfo("X="+str(x_accel)+", Z="+str(z_accel)+"]")
                if positive: 
                    # The acceleration felt by sphero from the crash is positive, hence towards the
                    # front, hence sphero has crashed onto the back wall
                    message = 'back'
                else:
                    message = 'front'
            
            if max_acc_index == 2:
                # The crash as been along the z-axis, eg from above or from below - a jump
                #rospy.logwarn("[Z="+str(z_accel))
                #rospy.loginfo("X="+str(x_accel)+", Y="+str(y_accel)+"]")
                if positive: 
                    # The acceleration felt by sphero from the crash is positive, hence towards the
                    # top, hence sphero has crashed onto something below it
                    message = 'below'
                else:
                    message = 'above'
        else:
            #rospy.loginfo("X="+str(x_accel)+", Y="+str(y_accel)+", Z="+str(z_accel)+"]")
            message = "nothing"
        
        return self.convert_to_dict(message)

    def convert_to_dict(self, message):
        # Converts the given message to a dictionary telling in which direction there is a detection
        detect_dict = {}
        # We consider that when there is a big Z axis component there has been a very big front crash
        detect_dict = {"front":(message=="front" or message=="up" or message=="down"),
                          "left":message=="left",
                          "right":message=="right",
                          "back":message=="back"}
        return detect_dict

if __name__ == '__main__':
    rospy.init_node('imu_subscriber', log_level=rospy.ERROR)
    read_imu = IMUTopicReader()
    rate = rospy.Rate(20)

    ctrl_c = False
    def shutdownhook():
        global ctrl_c

        rospy.loginfo('Shutdown time!')
        ctrl_c = True
    
    rospy.on_shutdown(shutdownhook)

    while not ctrl_c:
        data = read_imu.crash_direction_detection()
        #rospy.loginfo(data)
        if True in data.values():
            key_list = list(data.keys()) 
            val_list = list(data.values())
            crash_direction = key_list[val_list.index(True)]
            #rospy.logerr(crash_direction)
        #rate.sleep()
    