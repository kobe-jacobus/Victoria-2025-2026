from vex import *
from main import brain, left, right, controller_1
from PID import PID , turnPID

# ---------------------------------------------------------------------------- #

rotateTune = turnPID(leftMotorGroup=left, rightMotorGroup=right)


# ---------------------------------------------------------------------------- #

def auton():
    brain.screen.clear_screen()
    brain.screen.print("autonomous code")
    # place automonous code here
    for i in range(0, 360, 30):
        rotateTune.tune(i, 0.2, f"turnPID{i}.csv") 

