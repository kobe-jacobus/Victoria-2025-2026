from vex import *

# ---------------------------------------------------------------------------- #


def arcadeDrive(left: MotorGroup, right: MotorGroup, controller: Controller):
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
    
