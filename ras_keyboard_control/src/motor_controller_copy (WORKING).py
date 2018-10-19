#!/usr/bin/env python

import rospy
import std_msgs.msg
import phidgets.msg
import geometry_msgs.msg

#####################################################
#               Initialize Variables                #
#####################################################
ENCODER_LEFT = 0
ENCODER_RIGHT = 0

LINEAR_VELOCITY = 0.0
ANGULAR_VELOCITY = 0.0


#####################################################
#             /left_motor/encoder Callback          #
#####################################################
def update_feedback_enc_left(feedback_enc):
    global ENCODER_LEFT, ENCODER_LEFT_TEMP, has_updated_left
    ENCODER_LEFT = feedback_enc.count_change

#	self.FEEDBACK_ENC_UPDATED = True 

#####################################################
#             /right_motor/encoder Callback         #
#####################################################
def update_feedback_enc_right(feedback_enc):
    global ENCODER_RIGHT, ENCODER_RIGHT_TEMP, has_updated_right
    # NOTE THE MINUS SIGN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    ENCODER_RIGHT = -feedback_enc.count_change

#####################################################
#             /keyboard/vel Callback                #
#####################################################
def update_feedback_keyboard_vel(feedback_enc):
    global LINEAR_VELOCITY, ANGULAR_VELOCITY
    LINEAR_VELOCITY = feedback_enc.linear.x
    ANGULAR_VELOCITY = feedback_enc.angular.z


#####################################################
#               Initialize Publisher                #
#####################################################
rospy.init_node('motor_control_node', anonymous=True)
pub_LEFT_MOTOR = rospy.Publisher('/left_motor/cmd_vel', std_msgs.msg.Float32, queue_size=1)
pub_RIGHT_MOTOR = rospy.Publisher('/right_motor/cmd_vel', std_msgs.msg.Float32, queue_size=1)
rate = rospy.Rate(10)

rospy.Subscriber('/left_motor/encoder', phidgets.msg.motor_encoder, update_feedback_enc_left)
rospy.Subscriber('/right_motor/encoder', phidgets.msg.motor_encoder, update_feedback_enc_right)
rospy.Subscriber('/keyboard/vel', geometry_msgs.msg.Twist, update_feedback_keyboard_vel)


#####################################################
#            Controller Function                    #
#####################################################
def controller():
    global LINEAR_VELOCITY, ANGULAR_VELOCITY,ENCODER_LEFT, ENCODER_RIGHT

    # global parameters
    pi = 3.14
    control_frequency = 10
    ticks_per_rev = 897.96*5

    # vehicle parameters
    dt = 0.1
    base = 0.24
    wheel_radius = 0.0485

    # error integral part
    int_error_left = 0.0
    int_error_right = 0.0

    # PID parameters
    Kp_left = 30.0
    Kp_right = 35.0
    Ki_left = 400.0
    Ki_right = 400.0
    Kd_left = 0
    Kd_right = 0

    PWM = std_msgs.msg.Float32()

    while not rospy.is_shutdown():
        #####################################################
        #            Left Wheels                           #
        #####################################################
        estimated_w = (ENCODER_LEFT * 2 * pi * control_frequency) / (ticks_per_rev)
        desired_w = 0.2*0.25*(LINEAR_VELOCITY - (base / 2.0) * ANGULAR_VELOCITY) / wheel_radius

	print("est,desired left", estimated_w, desired_w) 
        error = desired_w - estimated_w
	print("Error left", error)

        int_error_left = int_error_left + error * dt

        PWM_LEFT = (int)(Kp_left * error + Ki_left * int_error_left)
	

        #####################################################
        #            Right Wheels                           #
        #####################################################

        estimated_w = (ENCODER_RIGHT * 2 * pi * control_frequency) / (ticks_per_rev)
        desired_w = 0.33*0.2*(LINEAR_VELOCITY + (base / 2.0) * ANGULAR_VELOCITY) / wheel_radius
	print("est,desired right", estimated_w, desired_w)

        error = desired_w - estimated_w
	print("Error right", error)

        int_error_right = int_error_right + error * dt

        PWM_RIGHT = (int)(Kp_right * error + Ki_right * int_error_right)

	print("encoder ", ENCODER_LEFT, ENCODER_RIGHT)
	print("PWM", PWM_LEFT, PWM_RIGHT)

	if (abs(LINEAR_VELOCITY) < 0.001 and abs(ANGULAR_VELOCITY) < 0.001):
		PWM_LEFT = 0
		PWM_RIGHT = 0

        PWM.data = PWM_LEFT
        pub_LEFT_MOTOR.publish(PWM)
        PWM.data = -PWM_RIGHT
        pub_RIGHT_MOTOR.publish(PWM)
        rate.sleep()


#####################################################
#                Main Function                      #
#####################################################
if __name__ == "__main__":
    try:
        controller()
    except rospy.ROSInterruptException:
        pass
