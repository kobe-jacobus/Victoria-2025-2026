# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       RubenCheyns                                                  #
# 	Created:      4/28/2025, 1:04:28 PM                                        #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

"""
Main VEX V5 robot program for Team 49956A (Push Back 2025-2026).

Contents:
- device configuration
- PID and turnPID classes for closed-loop control and tuning
- autonomous helper functions
- autonomous code
- user-control helper functions
- simple touchscreen autonomous selector UI
- competition instance creation
"""

# Library imports
from vex import *

#-------------------#
# vex device config #
#-------------------#
brain = Brain()
gyro = Inertial(Ports.PORT17)
controller_1 = Controller()
controller_2 = Controller()

left_1 = Motor(Ports.PORT20, GearSetting.RATIO_6_1, False)
left_2 = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
left_3 = Motor(Ports.PORT18, GearSetting.RATIO_6_1, True)
left = MotorGroup(left_1, left_2, left_3)

right_1 = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)
right_2 = Motor(Ports.PORT9, GearSetting.RATIO_6_1, True)
right_3 = Motor(Ports.PORT8, GearSetting.RATIO_6_1, False)
right = MotorGroup(right_1, right_2, right_3)

intakeMotor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
storageMotor = Motor(Ports.PORT11, GearSetting.RATIO_18_1, False)
outMotor = Motor(Ports.PORT16, True)



#-------------#
# PID classes #
#-------------#
class PID:
    """Generic PID controller for reading a sensor and computing an output.

    Attributes:
        yourSensor: callable returning current sensor value (e.g., gyro.heading)
        brain: Brain instance (used for SD card, screen, etc.)
        KP, KI, KD: PID constants
        output: last computed output value
        stopButton: if True, tune() will show an on-screen stop button
    """

    def __init__(self, yourSensor, brain: Brain, KP: float = 1, KI: float = 0, KD: float = 0):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.yourSensor = yourSensor
        self.brain = brain
        self.output: float = 0

    def run(self, desiredValue: int, tollerance: float):
        """Run PID loop until the sensor reaches desiredValue within tolerance.

        This method updates self.output. It does not apply the output to motors —
        subclasses or callers should use self.output as required.
        """
        previousError = 0
        totalError = 0
        i = 0
        while abs(desiredValue - self.yourSensor()) > tollerance:
            i += 1
            error = desiredValue - self.yourSensor()
            derivative = (error - previousError) / (i*50)
            totalError += error
            self.output = error * self.KP + derivative * self.KD + (totalError * (i*50)) * self.KI
            wait(50)
            previousError = error

    def tune(self, desiredValue: int, tollerance: float, sd_file_name = "pidData.csv", stopButton = False):
        """Run PID loop and save tuning data to SD card.

        Produces CSV with columns:
            time, error, derivative, totalError, output, desiredValue

        If stopButton is True, displays a red 'terminate' button on the brain screen
        allowing the operator to abort and save partial data.
        """
        if stopButton:
            stop = button(60, 220, 250, 10, Color.RED, "terminate")
            stop.draw()

        csvHeaderText = "time, error, derivative, totalError, output, desiredValue"
        data_buffer = csvHeaderText + "\n"

        previousError = 0
        totalError = 0
        i = 0

        while abs(desiredValue - self.yourSensor()) > tollerance:
            i += 1
            error = desiredValue - self.yourSensor()
            derivative = (error - previousError) / (i*50)
            totalError += error
            self.output = error * self.KP + derivative * self.KD + (totalError * (i*50)) * self.KI
            wait(50)
            previousError = error

            # append one row of data to buffer
            data_buffer +=  str(i * 50) + ","
            data_buffer += "%.3f" % error + ","
            data_buffer += "%.3f" % derivative + ","
            data_buffer += "%.3f" % (totalError * (i*50)) + ","
            data_buffer += "%.3f" % self.output + ","
            data_buffer += str(desiredValue) + "\n"
            
            # allow user to abort when using touchscreen stop button
            if stopButton and stop.isPressed(self.brain.screen.x_position(),self.brain.screen.y_position()):
                break

        # save CSV to SD card (brain.sdcard)
        self.brain.sdcard.savefile(sd_file_name, bytearray(data_buffer, 'utf-8'))

class turnPID(PID):
    """PID controller specialized for turning a drivetrain (left/right motor groups).

    It computes a rotational output and sets velocities on left and right MotorGroups.

    Constructor parameters:
        yourSensor: callable returning current heading/angle
        brain: Brain instance
        leftMotorGroup, rightMotorGroup: MotorGroup instances to apply rotation
        KP, KI, KD: PID gains
        stopButton: enable touchscreen terminate button during tune()
    """

    def __init__(self, yourSensor, brain: Brain, leftMotorGroup: MotorGroup, rightMotorGroup: MotorGroup, KP: float = 1, KI: float = 0, KD: float = 0):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.left = leftMotorGroup
        self.right = rightMotorGroup
        self.yourSensor = yourSensor
        self.brain = brain
        self.output:float = 0

    def run (self, desiredValue: int, tollerance: float):
        """Run turn PID and set motor velocities until target heading reached."""
        previousError = 0
        totalError = 0
        i = 0
        while abs(desiredValue - self.yourSensor()) > tollerance:
            i += 1
            error:float = desiredValue - self.yourSensor() if desiredValue - self.yourSensor() <= 180 else desiredValue - self.yourSensor() - 180
            derivative = (error - previousError) / (i*50)
            totalError += error
            self.output = error * self.KP + derivative * self.KD + (totalError * (i*50)) * self.KI
            # set velocities such that robot rotates in place
            self.left.set_velocity(self.output, PERCENT)
            self.right.set_velocity(-self.output, PERCENT)
            wait(50)
            previousError = error

    def tune(self, desiredValue: int, tollerance: float, sd_file_name = "pidData.csv", stopButton = False):
        """Run tuning loop similar to PID.tune but apply outputs to drivetrain and save CSV.

        CSV columns:
            time, proportional, derivative, integral, output, desiredValue
        """
        if stopButton:
            stop = button(60, 220, 250, 10, Color.RED, "terminate")
            stop.draw()

        csvHeaderText:str = "time, proportional, derivative, integral, output, desiredValue"
        data_buffer:str = csvHeaderText + "\n"

        previousError = 0
        totalError = 0
        i = 0

        while abs(desiredValue - self.yourSensor()) > tollerance:
            i += 1
            error:float = desiredValue - self.yourSensor() if desiredValue - self.yourSensor() <= 180 else desiredValue - self.yourSensor() - 180
            derivative = (error - previousError) / (i*50)
            totalError += error
            self.output = error * self.KP + derivative * self.KD + (totalError * (i*50)) * self.KI
            self.left.set_velocity(self.output, PERCENT)
            self.right.set_velocity(-self.output, PERCENT)
            wait(50)
            previousError = error

            # save one row of data
            data_buffer += str(i * 50) + ","
            data_buffer += "%.3f" % (error*self.KP) + ","
            data_buffer += "%.3f" % (derivative * self.KD)  + ","
            data_buffer += "%.3f" % (totalError * (i*50)*self.KI) + ","
            data_buffer += "%.3f" % self.output + ","
            data_buffer += str(desiredValue) + "\n"

            if stopButton and stop.isPressed(self.brain.screen.x_position(),self.brain.screen.y_position()):
                break

        self.brain.sdcard.savefile(sd_file_name, bytearray(data_buffer, 'utf-8'))


# --------------------
# PID setup
# --------------------
# create a turnPID instance for drivetrain rotation tuning
rotatePID = turnPID(yourSensor= gyro.heading , brain = brain, leftMotorGroup=left, rightMotorGroup=right,
                     KP = 0.2,
                     KI = 0.00000007,
                     KD = 0.02
                     )


# --------------------
# autonomous helpers
# --------------------
def forward(mm: int, speed: int= 20):
    diameter = 82.55 # wheel diameter in mm
    deg = mm*(360/(diameter*3.1416)) # calculates degrees to spin based on mm input
    right.set_velocity(speed, PERCENT)
    left.set_velocity(speed, PERCENT)
    right.spin_for(FORWARD, deg, wait= False)
    left.spin_for(FORWARD, deg, wait= True)

# --------------------
# autonomous routines
# --------------------
def tune():
    """Autonomous routine to tune turn PID for various angles.
    """
    for i in range(30, 360, 30):
        right.spin(FORWARD, 0)
        left.spin(FORWARD, 0)
        rotatePID.tune(i, 2, 'turnPID'+ str(i) + '.csv', stopButton=True)
        right.stop(HOLD)
        left.stop(HOLD)
        wait(2, SECONDS)

def Left():
    """temporary autonomous routine for scrimage 1
    """
    forward(800)
    right.spin(FORWARD, 0, PERCENT)
    left.spin(FORWARD, 0, PERCENT)
    rotatePID.run(265, 2)
    right.spin(REVERSE,20,PERCENT)
    left.spin(REVERSE,20,PERCENT)
    wait(1000,MSEC)
    right.stop(HOLD)
    left.stop(HOLD)
    outDown.spin(FORWARD, 100, PERCENT)
    outUp.spin(REVERSE, 100, PERCENT)
    wait(2, SECONDS)
    outDown.stop(COAST)
    outUp.stop(COAST)

def Right():
    """temporary autonomous routine for scrimage 1
    """
    forward(800)
    right.spin(FORWARD, 0, PERCENT)
    left.spin(FORWARD, 0, PERCENT)
    rotatePID.run(95, 2)
    right.spin(REVERSE,20,PERCENT)
    left.spin(REVERSE,20,PERCENT)
    wait(1000,MSEC)
    right.stop(HOLD)
    left.stop(HOLD)
    outDown.spin(FORWARD, 100, PERCENT)
    outUp.spin(REVERSE, 100, PERCENT)
    wait(2, SECONDS)
    outDown.stop(COAST)
    outUp.stop(COAST)

# --------------------
# user control helpers
# --------------------
k = 2  # exponent constant for drive graph

def changeDriveGraph(controller: Controller):
    """Adjust exponent constant 'k' for drive graph via controller up/down buttons.

    """
    global k
    if controller.buttonUp.pressed:
        k += 1
    elif controller.buttonDown.pressed:
        k -= 1
    controller.screen.clear_screen()
    controller.screen.print(k)
    return k


def driveGraph(x, k):
    """Map joystick input x with exponent k to a velocity curve.

    Positive and negative inputs are handled symmetrically.
    """
    if x > 0:
        return (x**k)/10**((k-1)*2)
    else:
        return -(x**k)/10**((k-1)*2)


def arcadeDriveGraph(left: MotorGroup, right: MotorGroup, controller: Controller, torqueOn: bool = False):
    """Arcade drive: forward/back from left joystick axis3 (processed by driveGraph),
    turn from right joystick axis1. Sets motor velocities and starts spinning.
    this version includes quadratic drive graph for finer control at low speeds.
    If torqueOn is True, limits max speed to 60% for more torque.
    """
    
    if torqueOn:
        right.set_velocity((driveGraph(controller.axis3.position(),2) - driveGraph(controller.axis1.position(),2))*6/10, PERCENT)
        left.set_velocity((driveGraph(controller.axis3.position(),2) + driveGraph(controller.axis1.position(),2))*6/10, PERCENT)
    else:
        right.set_velocity((driveGraph(controller.axis3.position(),2) - driveGraph(controller.axis1.position(),2)), PERCENT)
        left.set_velocity((driveGraph(controller.axis3.position(),2) + driveGraph(controller.axis1.position(),2)), PERCENT)
    

    left.spin(FORWARD)
    right.spin(FORWARD)


def inOutControl():
    """Control intake motors using controller buttons:
    - L1:   intake
    - L2:   score low
    - R1:   score mid
    - R2:   score high
    - none: brake both motors
    """
    if controller_1.buttonL1.pressing():
        intakeMotor.spin(FORWARD, 100, PERCENT)
        storageMotor.spin(FORWARD, 100, PERCENT)
        outMotor.stop(BRAKE)
    elif controller_1.buttonL2.pressing():
        intakeMotor.spin(REVERSE, 100, PERCENT)
        storageMotor.spin(REVERSE, 100, PERCENT)
        outMotor.stop(BRAKE)
    elif controller_1.buttonR1.pressing():
        intakeMotor.spin(FORWARD, 100, PERCENT)
        storageMotor.spin(REVERSE, 100, PERCENT)
        outMotor.spin(FORWARD, 100, PERCENT)
    elif controller_1.buttonR2.pressing():
        intakeMotor.spin(FORWARD, 100, PERCENT)
        storageMotor.spin(REVERSE, 100, PERCENT)
        outMotor.spin(REVERSE, 100, PERCENT)
    else:
        intakeMotor.stop(BRAKE)
        storageMotor.stop(BRAKE)
        outMotor.stop(BRAKE)

# --------------------
# UI classes
# --------------------
class button:
    """touchscreen button object

    Parameters:
        height, width: size of rectangle
        posX, posY: top-left position on brain screen
        color: VEX Color
        text: label shown on the button
    """

    def __init__(self, height:int, width:int, posX:int, posY:int, color, text:str) -> None:
        self.height = height
        self.width = width
        self.posX = posX
        self.posY = posY
        self.Pressed = False
        self.color = color
        self.text = text

    def draw(self):
        """Draw the button on the brain screen."""
        brain.screen.set_pen_color(self.color)
        brain.screen.draw_rectangle(self.posX, self.posY, self.width, self.height, self.color)
        brain.screen.set_pen_color(Color.BLACK)
        brain.screen.print_at(self.text, x = self.posX + 10, y = self.posY + self.height//2 - 5, opaque = False)

    def isPressed(self, touchX:int, touchY:int) -> bool:
        """Return True if the provided touch coordinates are inside this button."""
        if touchX > self.posX and touchX < self.posX + self.width and touchY > self.posY and touchY < self.posY + self.height:
            self.Pressed = True
        else:
            self.Pressed = False
        return self.Pressed


class autonSelector:
    """Simple touchscreen autonomous routine selector.

    Parameters:
        autons: list of callable autonomous functions
        names: list of display names for each auton (strings)
        doc: list of short descriptions shown on confirm screen
        background: filename of background image to draw on the brain screen

    Usage:
        selector = autonSelector([auton1, auton2], ["A1","A2"], ["desc1","desc2"], "background.png")
        selected = selector.display()  # blocks until user confirms selection
    """

    def __init__(self, autons:list, names:list, doc:list, background) -> None:
        self.autons = autons
        self.names = names
        self.doc = doc
        self.background = background
        self.selected = None

    def display(self):
        """Show the selector UI and return the selected autonomous function.

        The method blocks until the user confirms an auton on the touchscreen.
        """
        buttons = []
        brain.screen.draw_image_from_file(self.background, 0, 0)

        # create a vertical list of buttons from provided names
        for i in range(1, len(self.autons) + 1):
            buttons.append(button(50, 220, 10, 10 + (i-1)*60, Color.GREEN, str(self.names[i-1])))
            buttons[i-1].draw()
        brain.screen.render()

        # wait for touches and handle confirm/cancel dialogs
        while True:
            if brain.screen.pressing():
                touchX = brain.screen.x_position()
                touchY = brain.screen.y_position()
                for i in range(len(buttons)):
                    if buttons[i].isPressed(touchX, touchY):
                        # show confirm/cancel with description
                        brain.screen.clear_screen()
                        brain.screen.draw_image_from_file(self.background, 0, 0)
                        brain.screen.render()
                        confirm = button(60, 220, 10, 10, Color.GREEN, "Confirm")
                        cancel = button(60, 220, 250, 10, Color.RED, "Cancel")
                        confirm.draw()
                        cancel.draw()
                        brain.screen.set_pen_color(Color.WHITE)
                        brain.screen.set_cursor(5, 1)
                        #brain.screen.print_at(self.doc[i], x = 10, y = 80, opaque = False,sep='\n')
                        for txt in self.doc[i].split('\n'):
                            brain.screen.print(txt, opaque = False, sep='\n')
                            brain.screen.new_line()
                        wait(1, SECONDS)
                        brain.screen.render()

                        while True:
                            if brain.screen.pressing():
                                touchX = brain.screen.x_position()
                                touchY = brain.screen.y_position()
                                if confirm.isPressed(touchX, touchY):
                                    # user confirmed selection: store function and return it
                                    self.selected = self.autons[i]
                                    brain.screen.clear_screen()
                                    brain.screen.draw_image_from_file("background.png", 0, 0)
                                    brain.screen.render()
                                    return self.selected
                                elif cancel.isPressed(touchX, touchY):
                                    # go back to main selection screen
                                    brain.screen.clear_screen()
                                    brain.screen.draw_image_from_file(self.background, 0, 0)
                                    for j in range(len(buttons)):
                                        buttons[j].draw()
                                    brain.screen.render()
                                    break
                            wait(100)
            wait(100)


# --------------------
# UI setup and competition
# --------------------
selector = autonSelector(
    [Left, Right, tune],
    ["Left", "Right","Tune"],
    ["placement:\n  paralel with wall\n  contacting Left side of park zone\n  with right back", "placement:\n  paralel with wall\n  contacting Right side of park zone\n  with right back",""],
    "background.png"
    )

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("user control code")
    while True:
        arcadeDriveGraph(left, right, controller_1, torqueOn=controller_1.buttonX.pressing())
        inOutControl()
        wait(20, MSEC)

# show selector and create competition instance
selector.display()

# create competition instance
comp = Competition(user_control, selector.selected)

