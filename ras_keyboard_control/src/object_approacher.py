#!/usr/bin/env python

import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import std_msgs.msg
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import time
from datetime import datetime

import tf

rospy.init_node('path_follower_node', anonymous=True)
rate = rospy.Rate(20)

class ObjectApproacher():
    def __init__(self):

        # current robot position (updated by odom subscriber continously)
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.theta_odom = 0.0

        self.delta_x = 0
        self.abort_object_approaching = False
        self.approach_object = True

        self.obj_x = 999
        self.obj_y = 999

        self.last_update = datetime.now()

    #####################################
    #             Callbacks             #
    #####################################
    def objectDeltaCallback(self, msg):
        self.delta_x = msg
        self.last_update = datetime.now()


    def objectPositionCallback(self, msg):
        self.obj_x = msg.pose.position.x
        self.obj_y = msg.pose.position.y

    def flagCallback(self, msg):
        flag = msg.data
        if flag == "STOP":
            self.abort_object_approaching = True

        if flag == "APPROACH":
            self.approach_object = True

    def filterCallback(self, msg):
        self.x_odom = msg.pose.pose.position.x
        self.y_odom = msg.pose.pose.position.y
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.theta_odom = y

    #####################################
    #             Publishers            #
    #####################################
    def send_velocity(self, LINEAR_VELOCITY, ANGULAR_VELOCITY):
        VEL = geometry_msgs.msg.Twist()
        VEL.linear.x = LINEAR_VELOCITY
        VEL.angular.z = ANGULAR_VELOCITY
        pub_path_following_VEL.publish(VEL)
    
    def send_flag(self, flag_message):
        flag = std_msgs.msg.String()
        flag.data = flag_message
        pub_flag_done.publish(flag)


    #####################################
    #             Alogrithm             #
    #####################################

    def dist_to_goal(self):
        return math.sqrt((self.x_odom - self.obj_x)**2 + (self.y_odom - self.obj_y)**2) 


    def main(self):
        while not rospy.is_shutdown():

            if self.approach_object:    
   
                # send the velocity commands
                lin_vel = 0.045
                ang_vel = self.delta_x * 0.005
                if abs(self.delta_x):
                    lin_vel = 0
                
                self.send_velocity(lin_vel, ang_vel)
                #print("velocity sent")

                now_time = datetime.now()

                # check if abort flag received or too close to target
                if self.abort_object_approaching == True or self.dist_to_goal() < 0.15 or (now_time - self.last_update).seconds > 3:
                    print("ABORTED OBJECT APPROACHING")
                    self.approach_object = False

                    # object Approaching done
                    self.send_velocity(0, 0) # stop robot
                    time.sleep(2)



if __name__ == '__main__':
    print("object approacher started")

    oa = ObjectApproacher()

    # publishers
    pub_path_following_VEL = rospy.Publisher('/keyboard/vel', geometry_msgs.msg.Twist, queue_size=1)
    pub_flag_done = rospy.Publisher('/flag_done', std_msgs.msg.String, queue_size=1)

    # subscribers
    rospy.Subscriber("/robot_filter", Odometry, oa.filterCallback)
    rospy.Subscriber("/object_delta", Odometry, oa.objectDeltaCallback)
    rospy.Subscriber("/object_position", Odometry, oa.objectPositionCallback)
    rospy.Subscriber("/path_follower_flag", String, oa.flagCallback)

    oa.main()
