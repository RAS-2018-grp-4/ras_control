#!/usr/bin/env python

import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import std_msgs.msg
from std_msgs.msg import String
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import itertools
import tf

rospy.init_node('path_follower_node', anonymous=True)
rate = rospy.Rate(50)

show_animation = True

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw


class PathFollower():
    def __init__(self):
        self.D = 0.15  # look-ahead distance
        self.L = 0.21  # robot base length

        # current robot position (received by odom publisher continously)
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.theta_odom = 0.0

        # parameters
        self.follow_new_path = False
        self.abort_path_following = False

        # current path
        self.path_x = []
        self.path_y = []

    #####################################
    #             Callbacks             #
    #####################################
    def odomCallback(self, msg):
        self.x_odom = msg.pose.pose.position.x
        self.y_odom = msg.pose.pose.position.y
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.theta_odom = y

    def pathCallback(self, msg):
        path = msg.poses
        print("new path received, length:", len(path))

        self.path_x = []
        self.path_y = []
        for i in range(len(path)):
            self.path_x.append(path[i].pose.position.y -0.2)
            self.path_y.append(-path[i].pose.position.x + 0.2)

        # set flags to start executing 
        self.follow_new_path = True
        self.abort_path_following = False

    def flagCallback(self, msg):
        flag = msg.data
        if flag == "STOP":
            self.abort_path_following = True

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

    def override_velocities(self, ang_vel_temp, alpha):
        lin_vel = 0
        ang_vel = ang_vel_temp

        # motor controller commands
        if alpha > 3.141/1.5:
            lin_vel = 0
            ang_vel = -1.2
        elif alpha < -3.141/1.5:
            lin_vel = 0
            ang_vel = 1.2
        else:
            lin_vel = 0.08 - abs(alpha)*0.08/(3.141/1.5)

        return lin_vel, ang_vel

    def pure_pursuit_control(self, state, path_x, path_y, t_ind_prev):
        # get the target index to steer against
        t_ind = self.calc_target_index(state, path_x, path_y)

        # if the previous target was further away on the path than now, use that instead
        if t_ind_prev >= t_ind:
            t_ind = t_ind_prev

        # target index to coordinates
        if t_ind < len(path_x):
            tx = path_x[t_ind]
            ty = path_y[t_ind]
        else:
            tx = path_x[-1]
            ty = path_y[-1]
            t_ind = len(path_x) - 1

        # calculate the angle to the target point (relative to heading angle)
        alpha = math.atan2(ty - state.y, tx - state.x) - state.yaw

        # if reversing, flip the steering angle
        # if state.v < 0:
        #    alpha = math.pi - alpha

        # calculate an appropriate steering angle
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / self.D, 1.0)
        return delta, t_ind, alpha


    def calc_target_index(self, state, path_x, path_y):
        # find the index of the path closest to the robot
        dx = [state.x - icx for icx in path_x]
        dy = [state.y - icy for icy in path_y]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        t_ind = d.index(min(d))

        # search length
        Ls = 0.0

        # find the index of the look head point on the path (look ahead is the distance ALONG the path, not straight line)
        path_length = len(path_x)
        while Ls < self.D and (t_ind + 1) < path_length:
            dx = path_x[t_ind + 1] - path_x[t_ind]
            dy = path_y[t_ind + 1] - path_y[t_ind]
            Ls += math.sqrt(dx**2 + dy**2)
            t_ind += 1

        return t_ind

    def main(self):
        while not rospy.is_shutdown():
            # if follow_new_path == true --> start following new path
            if self.follow_new_path:    
                self.follow_new_path = False

                # current robot state
                state = State(x=self.x_odom, y=self.y_odom, yaw=self.theta_odom)

                lastIndex = len(self.path_x) - 1
                x = [state.x]
                y = [state.y]
                yaw = [state.yaw]
                target_ind = self.calc_target_index(state, self.path_x, self.path_y)

                # run the algorithm until the the target index reaches the last index in the path
                while lastIndex > target_ind:
                    # check if abort flag received
                    if self.abort_path_following == True:
                        break

                    # update state
                    state.x = self.x_odom
                    state.y = self.y_odom
                    state.yaw = self.theta_odom

                    # default command: stay still
                    lin_vel = 0
                    ang_vel = 0

                    # calculate appropriate steering
                    ang_vel, target_ind, alpha = self.pure_pursuit_control(state, self.path_x, self.path_y, target_ind)

                    # override the velocities if needed (e.g. in a sharp turn)
                    lin_vel, ang_vel = self.override_velocities(ang_vel, alpha)

                    # send the velocity commands
                    self.send_velocity(lin_vel, ang_vel)

                    # animation below
                    x.append(self.x_odom)
                    y.append(self.y_odom)
                    yaw.append(self.theta_odom)

                    if show_animation:
                        plt.cla()
                        plt.plot(self.path_x, self.path_y, ".r", label="course")
                        plt.plot([self.x_odom, self.x_odom + math.cos(self.theta_odom)], [self.y_odom, self.y_odom + math.sin(self.theta_odom)], "g", label="angle")
                        plt.plot(x, y, "-b", label="trajectory")
                        plt.plot(self.path_x[target_ind], self.path_y[target_ind], "xg", label="target")
                        plt.axis("equal")
                        plt.grid(True)
                        plt.pause(0.001)

                # path follower done
                self.send_velocity(0, 0) # stop robot

                # send a flag to the state machine
                self.send_flag("path_following_done")


            rate.sleep()


if __name__ == '__main__':
    print("path tracker started")

    pf = PathFollower()

    # publishers
    pub_path_following_VEL = rospy.Publisher('/keyboard/vel', geometry_msgs.msg.Twist, queue_size=1)
    pub_flag_done = rospy.Publisher('/flag_done', std_msgs.msg.String, queue_size=1)

    # subscribers
    rospy.Subscriber("/robot_odom", Odometry, pf.odomCallback)
    rospy.Subscriber("/aPath", Path, pf.pathCallback)
    rospy.Subscriber("/path_follower_flag", String, pf.flagCallback)

    pf.main()
