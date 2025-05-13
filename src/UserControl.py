from vex import *
from main import left, right, controller_1, brain

# ---------------------------------------------------------------------------- #

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("user control code")
    while True:
        arcadeDrive(left, right, controller_1)
        wait(20, MSEC)

# ---------------------------------------------------------------------------- #

def arcadeDrive(left, right, controller):
    """
    Arcade drive using the left joystick for forward/backward movement and the right joystick for turning.

    motorgroups: Left, Right
    controller: controller_1

    usecase:
        repeat in loop when driver control is active
    """
    left.set_velocity((controller.axis3.position() + controller.axis1.position()), PERCENT)
    right.set_velocity((controller.axis3.position() - controller.axis1.position()), PERCENT)
    left.spin(FORWARD)
    right.spin(FORWARD)
    

