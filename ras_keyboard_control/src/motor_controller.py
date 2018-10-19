#!/usr/bin/env python

import rospy
import std_msgs.msg
import phidgets.msg
import geometry_msgs.msg
import numpy as np
import matplotlib.pyplot as plt

'''
CLASS: motor_controller()
'''
class motor_controller():
    '''Member variables'''
    def __init__(self):
        self.ENCODER_LEFT = 0
        self.ENCODER_RIGHT = 0
        self.LINEAR_VELOCITY = 0.0
        self.ANGULAR_VELOCITY = 0.0
        self.control_frequency = 10  # in Hz, so dt = 1/control_frequency
        self.ticks_per_rev = 897.96
        #self.ticks_per_rev = 3591.84
        # vehicle parameters
        self.dt = 1.0/self.control_frequency
        self.base = 0.21
        self.wheel_radius = 0.0485

        # PID parameters
        self.Kp_left = 6.0
        self.Kp_right = 7.0
        self.Ki_left = 4.0
        self.Ki_right = 4.0
        self.Kd_left = 0.0
        self.Kd_right = 0.0

        self.int_error_left = 0.0
        self.int_error_right = 0.0

        # LOW PASS VARIABLES
        self.GAMMA = 0.3
        self.LP_estimated_w_left = 0.0
        self.LP_estimated_w_right = 0.0

        # plotting
        self.show_animation = False
        self.t = 0
        self.times = [0]
        self.des_l = [0]
        self.des_r = [0]
        self.est_l = [0]
        self.est_l_true = [0]
        self.est_r = [0]
        self.PWM_l = [0]
        self.PWM_r = [0]
    
        #####################################################
        #               Initialize Publisher                #
        #####################################################
        rospy.init_node('motor_control_node', anonymous=True)
        self.pub_LEFT_MOTOR = rospy.Publisher(
            '/left_motor/cmd_vel', std_msgs.msg.Float32, queue_size=1)
        self.pub_RIGHT_MOTOR = rospy.Publisher(
            '/right_motor/cmd_vel', std_msgs.msg.Float32, queue_size=1)
        self.rate = rospy.Rate(self.control_frequency)
    
    '''Member functions'''

    #####################################################
    #             /left_motor/encoder Callback          #
    #####################################################
    def update_feedback_enc_left(self, feedback_enc):
        #global ENCODER_LEFT, ENCODER_LEFT_TEMP, has_updated_left
        self.ENCODER_LEFT = self.ENCODER_LEFT + feedback_enc.count_change

    #####################################################
    #             /right_motor/encoder Callback         #
    #####################################################
    def update_feedback_enc_right(self, feedback_enc):
        #global ENCODER_RIGHT, ENCODER_RIGHT_TEMP, has_updated_right
        # NOTE THE MINUS SIGN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        self.ENCODER_RIGHT = self.ENCODER_RIGHT -feedback_enc.count_change

    #####################################################
    #             /keyboard/vel Callback                #
    #####################################################
    def update_feedback_keyboard_vel(self, feedback_enc):
        #global LINEAR_VELOCITY, ANGULAR_VELOCITY
        self.LINEAR_VELOCITY = feedback_enc.linear.x
        self.ANGULAR_VELOCITY = feedback_enc.angular.z

    #####################################################
    #            Controller Function                    #
    #####################################################
    def controller(self):
            PWM = std_msgs.msg.Float32()

            while not rospy.is_shutdown():
                #####################################################
                #            Left Wheel                             #
                #####################################################
                estimated_w_l = (0.33)*(self.ENCODER_LEFT * 2 * np.pi * self.control_frequency) / (self.ticks_per_rev)
                desired_w_l = (self.LINEAR_VELOCITY - (self.base / 2.0) * self.ANGULAR_VELOCITY) / self.wheel_radius	
		        
                self.LP_estimated_w_left = self.GAMMA*estimated_w_l + (1-self.GAMMA)*self.LP_estimated_w_left
                print("est, LP, desired left", estimated_w_l, self.LP_estimated_w_left, desired_w_l)
                error = desired_w_l - self.LP_estimated_w_left
                print("Error left", error)

                self.int_error_left = self.int_error_left + error * self.dt
		print(self.int_error_left)
                PWM_LEFT = (int)(self.Kp_left * error + self.Ki_left * self.int_error_left)

                #####################################################
                #            Right Wheel                            #
                #####################################################

                estimated_w_r = (0.33)*(self.ENCODER_RIGHT * 2 * np.pi * self.control_frequency) / (self.ticks_per_rev)
                desired_w_r = (self.LINEAR_VELOCITY + (self.base / 2.0) * self.ANGULAR_VELOCITY) / self.wheel_radius


		self.LP_estimated_w_right = self.GAMMA*estimated_w_r + (1-self.GAMMA)*self.LP_estimated_w_right
                print("est, LP, desired right", estimated_w_r, self.LP_estimated_w_right, desired_w_r)
                error = desired_w_r - self.LP_estimated_w_right
                print("Error right", error)

                self.int_error_right = self.int_error_right + error * self.dt

                PWM_RIGHT = (int)(self.Kp_right * error + self.Ki_right * self.int_error_right)

                print("encoder ", self.ENCODER_LEFT, self.ENCODER_RIGHT)
                print("PWM", PWM_LEFT, PWM_RIGHT)

                if (abs(self.LINEAR_VELOCITY) < 0.001 and abs(self.ANGULAR_VELOCITY) < 0.001):
                    PWM_LEFT = 0
                    PWM_RIGHT = 0
                    self.int_error_left = 0
                    self.int_error_right = 0 

                PWM.data = PWM_LEFT
                self.pub_LEFT_MOTOR.publish(PWM)
                PWM.data = -PWM_RIGHT
                self.pub_RIGHT_MOTOR.publish(PWM)

                # flush the encoders
                self.ENCODER_LEFT = 0
                self.ENCODER_RIGHT = 0

                self.times.append(self.t)
                self.des_l.append(desired_w_l)
                self.des_r.append(desired_w_r)
                self.est_l.append(self.LP_estimated_w_left)
                self.est_l_true.append(estimated_w_l)
                self.est_r.append(self.LP_estimated_w_right)
                self.PWM_l.append(PWM_LEFT)
                self.PWM_r.append(-PWM_RIGHT)
                self.t = self.t + 1

                if self.show_animation:
                    plt.cla()
                    plt.plot(self.times, self.des_l, "-r", label="desired l")
                    plt.plot(self.times, self.est_l, "-b", label="estimated l LP")
                    plt.plot(self.times, self.est_l_true, "-m", label="estimated true")
                    
                    plt.plot(self.times, self.PWM_l, "-g", label="PWM l")
                    plt.grid(True)
                    plt.pause(0.001)

                self.rate.sleep()



#####################################################
#                Main Function                      #
#####################################################
if __name__ == "__main__":
    try:
        mc = motor_controller()
        rospy.Subscriber('/left_motor/encoder',
                 phidgets.msg.motor_encoder, mc.update_feedback_enc_left)
        rospy.Subscriber('/right_motor/encoder',
                        phidgets.msg.motor_encoder, mc.update_feedback_enc_right)
        rospy.Subscriber('/keyboard/vel', geometry_msgs.msg.Twist,
                        mc.update_feedback_keyboard_vel)

        mc.controller()
    except rospy.ROSInterruptException:
        pass
