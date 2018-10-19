#!/usr/bin/env python

import numpy as np
import math
import matplotlib.pyplot as plt
import rospy
import std_msgs.msg
import geometry_msgs.msg
from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry
import itertools
import tf

D = 0.15  # look-ahead distance
L = 0.21  # 24 before
show_animation = False

# current robot position
x_odom = 0.0
y_odom = 0.0
theta_odom = 0.0

# current target position
x_target = 0.5
y_target = -0.5
theta_target = 0

# HEADING DIFFERENCE
alpha = 0

# parameters
follow_new_path = True
currently_running = False
#####################################################
#             /robot_odom Callback          #
#####################################################
def odomCallback(msg):
    global x_odom, y_odom, theta_odom
    x_odom = msg.pose.pose.position.x
    y_odom = msg.pose.pose.position.y
    (r, p, y) = tf.transformations.euler_from_quaternion([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
    theta_odom = y

def targetCallback(msg):
    global x_target, y_target, theta_targetm, follow_new_path
    x_target = msg.position.x
    y_target = msg.position.y
    (r, p, y) = tf.transformations.euler_from_quaternion([msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    theta_target = y
    print("path follow received new target", msg)

    follow_new_path = True

#####################################################
#               Initialize Publisher                #
#####################################################
rospy.init_node('path_follower_node', anonymous=True)
rate = rospy.Rate(50)

pub_path_following_VEL = rospy.Publisher('/keyboard/vel', geometry_msgs.msg.Twist, queue_size=1)
pub_flag_done = rospy.Publisher('/flag_done', std_msgs.msg.String, queue_size=1)

# odom subscriber
rospy.Subscriber("/robot_odom", Odometry, odomCallback)
rospy.Subscriber("/target_pose", Pose, targetCallback)

class State:
    def __init__(self, x=0.0, y=0.0, yaw=0.0):
        self.x = x
	self.y = y
	self.yaw = yaw

def send_message(LINEAR_VELOCITY, ANGULAR_VELOCITY):
    VEL = geometry_msgs.msg.Twist()
    VEL.linear.x = LINEAR_VELOCITY
    VEL.angular.z = ANGULAR_VELOCITY
    pub_path_following_VEL.publish(VEL)


def pure_pursuit_control(state, path_x, path_y, t_ind_prev):
    global alpha

    t_ind = calc_target_index(state, path_x, path_y)

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

    D_tot = D

    # calculate an appropriate steering angle
    delta = math.atan2(2.0 * L * math.sin(alpha) / D_tot, 1.0)
    return delta, t_ind


def calc_target_index(state, path_x, path_y):
    # find the index of the path closest to the robot
    dx = [state.x - icx for icx in path_x]
    dy = [state.y - icy for icy in path_y]
    d = [abs(math.sqrt(idx ** 2 + idy ** 2)) for (idx, idy) in zip(dx, dy)]
    t_ind = d.index(min(d))

    # total look ahead distance (taking the speed into consideration)
    D_tot = D

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

def main():
    global follow_new_path, currently_running, x_target, y_target, alpha

    while not rospy.is_shutdown():
        if follow_new_path and not currently_running:
            follow_new_path = False
            print("following new track")
            state = State(x=x_odom,y=y_odom,yaw=theta_odom)
            #  target course
            path_x = np.linspace(state.x, x_target, num=100)
            path_y = np.linspace(state.y, y_target, num=100)
            #path_x = np.arange(state.x, x_target, 0.01)
            #path_y = np.arange(state.y, y_target, 0.01)

            #path_x = np.arange(0, 3, 0.01)
            #path_y = [0.5*math.cos(ix / 0.3)-0.5 for ix in path_x]

            '''
            path_x1 = np.arange(0, 1, 0.01)
            path_x2 = np.empty(100)
            path_x2.fill(1)
            path_y1 = np.empty(100)
            path_y1.fill(0)
            path_y2 = np.arange(0, 1, 0.01)
            
            path_x = np.append(path_x1, path_x2) 
            path_y = np.append(path_y1, path_y2) 
            print(path_x)
            print(path_y)
            print(len(path_x))
            print(len(path_y))
            '''
 
            print(path_x, path_y)

            lastIndex = len(path_x) - 1
            x = [state.x]
            y = [state.y]
            yaw = [state.yaw]
            t = [0.0]
            state.x = x_odom
            state.y = y_odom
            state.yaw = theta_odom
            target_ind = calc_target_index(state, path_x, path_y)

            while lastIndex > target_ind:
                state.x = x_odom
                state.y = y_odom
                state.yaw = theta_odom


                lin_vel = 0
                ang_vel = 0

                ang_vel, target_ind = pure_pursuit_control(state, path_x, path_y, target_ind)

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

                send_message(lin_vel, ang_vel*steering_gain)
                #rate.sleep()
                print("ang_vel", ang_vel*steering_gain)

                x.append(x_odom)
                y.append(y_odom)
                yaw.append(theta_odom)

                if show_animation:
                    plt.cla()
                    plt.plot(path_x, path_y, ".r", label="course")
                    plt.plot([x_odom, x_odom + math.cos(theta_odom)], [y_odom, y_odom + math.sin(theta_odom)], "g", label="angle")
                    plt.plot(x, y, "-b", label="trajectory")
                    plt.plot(path_x[target_ind], path_y[target_ind], "xg", label="target")
                    plt.axis("equal")
                    plt.grid(True)
                    plt.pause(0.001)

            send_message(0, 0)
            #print("PATH FOLLOWING DONE")
            currently_running = False

            flag = std_msgs.msg.String()
            flag.data = "path_following_done"
            pub_flag_done.publish(flag)


            assert lastIndex >= target_ind, "Cannot goal"

            if show_animation:
                plt.plot(path_x, path_y, ".r", label="course")
                plt.plot(x, y, "-b", label="trajectory")
                plt.legend()
                plt.xlabel("x[m]")
                plt.ylabel("y[m]")
                plt.axis("equal")
                plt.grid(True)
                plt.show()
        rate.sleep()


if __name__ == '__main__':
    print("path tracker started")
    main()
