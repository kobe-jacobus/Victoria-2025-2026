# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       RubenCheyns                                                  #
# 	Created:      4/28/2025, 1:04:28 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *
from nonspecificFunc import *
from PID import *

# vex device config
brain = Brain()
left_1 = Motor(Ports.PORT20, GearSetting.RATIO_18_1, False)
left_2 = Motor(Ports.PORT19, GearSetting.RATIO_18_1, False)
left_3 = Motor(Ports.PORT18, GearSetting.RATIO_18_1, False)
left = MotorGroup(left_1, left_2, left_3)
right_1 = Motor(Ports.PORT10, GearSetting.RATIO_18_1, True)
right_2 = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
right_3 = Motor(Ports.PORT8, GearSetting.RATIO_18_1, True)
right = MotorGroup(right_1, right_2, right_3)
gyro = Inertial(Ports.PORT17)
controller_1 = Controller()


# PID setup
rotateTune = turnPID(yourSensor= gyro, brain = brain ,leftMotorGroup=left, rightMotorGroup=right)
rotateTune.KP = 1
rotateTune.KI = 0
rotateTune.KD = 0

# autonomous function
def auton():
    # place automonous code here
    for i in range(0, 360, 30):
        rotateTune.tune(i, 0.2, 'turnPID' + str(i) + '.csv') 

# user control function
    
def user_control():
    brain.screen.clear_screen()
    brain.screen.print("user control code")
    while True:
        arcadeDrive(left, right, controller_1)
        wait(20, MSEC)

# create competition instance
comp = Competition(user_control, auton)


# actions to do when the program starts
brain.screen.clear_screen()