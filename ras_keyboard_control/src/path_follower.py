#!/usr/bin/env python

import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import std_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from nav_msgs.msg import Path
from nav_msgs.msg import Odometry
import itertools
import tf

rospy.init_node('path_follower_node', anonymous=True)
rate = rospy.Rate(50)

show_animation = False

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw


class PathFollower():
    def __init__(self):
        self.D = 0.15  # look-ahead distance
        self.L = 0.21  # 24 before

        # current robot position (received by odom publisher continously)
        self.x_odom = 0.0
        self.y_odom = 0.0
        self.theta_odom = 0.0

        '''
        # current target position
        self.x_target = 0.0
        self.y_target = 0.0
        self.theta_target = 0.0
        '''

        # parameters
        self.follow_new_path = False

        # current path
        self.path_x = []
        self.path_y = []

    #####################################################
    #             /robot_odom Callback          #
    #####################################################
    def odomCallback(self, msg):
        self.x_odom = msg.pose.pose.position.x
        self.y_odom = msg.pose.pose.position.y
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        self.theta_odom = y

    '''
    def targetCallback(self, msg):
        self.x_target = msg.position.x
        self.y_target = msg.position.y
        (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
        self.theta_target = y
        print("path follow received new target", msg)
        self.follow_new_path = True
    '''

    def pathCallback(self, msg):
        path = msg.poses
        print("new path received, length:", len(path))

        self.path_x = []
        self.path_y = []
        for i in range(len(path)):
            self.path_x.append(path[i].pose.position.y -0.2)
            self.path_y.append(-path[i].pose.position.x + 0.2)
        self.follow_new_path = True


    def send_message(self, LINEAR_VELOCITY, ANGULAR_VELOCITY):
        VEL = geometry_msgs.msg.Twist()
        VEL.linear.x = LINEAR_VELOCITY
        VEL.angular.z = ANGULAR_VELOCITY
        pub_path_following_VEL.publish(VEL)


    def pure_pursuit_control(self, state, path_x, path_y, t_ind_prev):
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
        #if state.v < 0:
        #    alpha = math.pi - alpha

        D_tot = self.D

        # calculate an appropriate steering angle
        delta = math.atan2(2.0 * self.L * math.sin(alpha) / D_tot, 1.0)
        return delta, t_ind, alpha


    def calc_target_index(self, state, path_x, path_y):
        # find the index of the path closest to the robot
        dx = [state.x - icx for icx in path_x]
        dy = [state.y - icy for icy in path_y]
        d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
        t_ind = d.index(min(d))

        # total look ahead distance (taking the speed into consideration)
        D_tot = self.D

        # search length
        L = 0.0

        # find the index of the look head point on the path (look ahead is the distance ALONG the path, not straight line)
        path_length = len(path_x)
        while L < D_tot and (t_ind + 1) < path_length:
            dx = path_x[t_ind + 1] - path_x[t_ind]
            dy = path_y[t_ind + 1] - path_y[t_ind]
            L += math.sqrt(dx**2 + dy**2)
            t_ind += 1

        return t_ind

    def main(self):
        while not rospy.is_shutdown():
            # if follow_new_path == true --> reset and start following new path
            if self.follow_new_path:    
                self.follow_new_path = False

                # current robot state
                state = State(x=self.x_odom, y=self.y_odom, yaw=self.theta_odom)

                lastIndex = len(self.path_x) - 1
                x = [state.x]
                y = [state.y]
                yaw = [state.yaw]
                target_ind = self.calc_target_index(state, self.path_x, self.path_y)

                while lastIndex > target_ind:
                    # update state
                    state.x = self.x_odom
                    state.y = self.y_odom
                    state.yaw = self.theta_odom

                    # default command: stay still
                    lin_vel = 0
                    ang_vel = 0

                    # calculate appropriate steering
                    ang_vel, target_ind, alpha = self.pure_pursuit_control(state, self.path_x, self.path_y, target_ind)

                    # motor controller commands
                    if alpha > 3.141/1.5:
                        lin_vel = 0
                        ang_vel = -0.8
                    elif alpha < -3.141/1.5:
                        lin_vel = 0
                        ang_vel = 0.8
                    else:
                        lin_vel = 0.08

                    steering_gain = 1.0

                    self.send_message(lin_vel, ang_vel*steering_gain)

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
                self.send_message(0, 0) # stop robot

                flag = std_msgs.msg.String()
                flag.data = "path_following_done"
                pub_flag_done.publish(flag)

            rate.sleep()


if __name__ == '__main__':
    print("path tracker started")

    pf = PathFollower()

    # publishers
    pub_path_following_VEL = rospy.Publisher('/keyboard/vel', geometry_msgs.msg.Twist, queue_size=1)
    pub_flag_done = rospy.Publisher('/flag_done', std_msgs.msg.String, queue_size=1)

    # subscribers
    rospy.Subscriber("/robot_odom", Odometry, pf.odomCallback)
    #rospy.Subscriber("/target_pose", Pose, pf.targetCallback)
    rospy.Subscriber("/aPath", Path, pf.pathCallback)

    pf.main()
