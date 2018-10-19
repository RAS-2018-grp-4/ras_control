#!/usr/bin/env python

import os
import rospy
import readchar
import threading
import std_msgs.msg
import phidgets.msg
#####################################################
#               Initialize Variables                #
#####################################################
VEL_LEFT = 0.0
VEL_RIGHT = 0.0
ENCODER_LEFT = 0
ENCODER_RIGHT = 0
KEY = ''
STATE_UPDATED = False
STEP_VEL = 1



#####################################################
#             /left_motor/encoder Callback          #
#####################################################
def update_feedback_enc_left(feedback_enc):
        global ENCODER_LEFT
        ENCODER_LEFT = feedback_enc.count_change
#	self.FEEDBACK_ENC_UPDATED = True

#####################################################
#             /right_motor/encoder Callback          #
#####################################################
def update_feedback_enc_right(feedback_enc):
        global ENCODER_RIGHT
        ENCODER_RIGHT = feedback_enc.count_change
#	self.FEEDBACK_ENC_UPDATED = True


#####################################################
#               Initialize Publisher                #
#####################################################
rospy.init_node('keyboard_control_node', anonymous=True)
pub_LEFT_MOTOR = rospy.Publisher('/left_motor/cmd_vel', std_msgs.msg.Float32, queue_size=1)
pub_RIGHT_MOTOR = rospy.Publisher('/right_motor/cmd_vel', std_msgs.msg.Float32, queue_size=1)
rate = rospy.Rate(10)



rospy.Subscriber('/left_motor/encoder',phidgets.msg.motor_encoder, update_feedback_enc_left)
rospy.Subscriber('/right_motor/encoder',phidgets.msg.motor_encoder, update_feedback_enc_right)
#rospy.spin()





#####################################################
#               Initialize Threading                #
#####################################################
class ThreadedFunction(threading.Thread):

    def __init__(self, fcn_to_thread):
        threading.Thread.__init__(self)

        self.runnable = fcn_to_thread
        self.daemon = True

    def run(self):
        self.runnable()




####################################################
#          Publish STOP to PWM_INTERFACE            #
#####################################################
def publish_vel():
    global VEL_LEFT, VEL_RIGHT,STATE_UPDATED
    VEL = std_msgs.msg.Float32()
    
    while not rospy.is_shutdown():
        #VEL.data = 'data:'+ str(VEL_LEFT)
	VEL.data = VEL_LEFT
	#print("publish")
        pub_LEFT_MOTOR.publish(VEL)
        #VEL.data = 'data:'+ str(VEL_RIGHT)
	VEL.data = VEL_RIGHT
        pub_RIGHT_MOTOR.publish(VEL)
        STATE_UPDATED = True
	#rospy.spin()
	rate.sleep()

#####################################################
#                   Clear Screen                    #
#####################################################
def cls():
    os.system("clear")


#####################################################
#   Initialize Control Interface for Terminal       #
#####################################################
def control():
    global VEL_LEFT, VEL_RIGHT,ENCODER_LEF, ENCODER_RIGHT,KEY
    global STATE_UPDATED, VEL_STEP

    #####################################################
    #               WHILE LOOP for User Input           #
    #####################################################
    while KEY != 'q' and not rospy.is_shutdown():
	
        while not STATE_UPDATED:
            pass
	
        #####################################################
        #        Print Control Interface to Terminal        #
        #####################################################
        #cls()
        print("#####################################")
        print("LEFT_VELOCITY:    [     w | s    ]")
        print("RIGHT_VELOCITY:   [     e | d    ]")
        print("QUIT:             [      'q'     ]")
        print("#####################################")
        print('Input:   LEFT_VEL: {0} | RIGHT_VEL: {1}'.format(VEL_LEFT, VEL_RIGHT))
        print('Encoder: LEFT_VEL: {0} | RIGHT_VEL: {1}'.format(ENCODER_LEFT, ENCODER_RIGHT))
	print("Control input: " + KEY)
        KEY = readchar.readchar()
	cls()


        
        #####################################################
        #        READ KEY for updating LEFT                 #
        #####################################################
        if KEY == 'w':
            VEL_LEFT = VEL_LEFT + STEP_VEL
        elif KEY == 's':
            VEL_LEFT = VEL_LEFT - STEP_VEL
        else:
            pass




        #####################################################
        #        READ KEY for updating RIGHT                #
        #####################################################
        if KEY == 'e':
            VEL_RIGHT = VEL_RIGHT + STEP_VEL
        elif KEY == 'd':
            VEL_RIGHT = VEL_RIGHT - STEP_VEL
        else:
            pass



	STATE_UPDATED = False


#####################################################
#                Main Function                      #
#####################################################
if __name__ == "__main__":
    try:
        Thread = ThreadedFunction(publish_vel)
        Thread.start()
        control()
    except rospy.ROSInterruptException:
        pass




