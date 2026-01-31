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

intakeMotor = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
storageMotor = Motor(Ports.PORT11, GearSetting.RATIO_18_1, True)
outMotor = Motor(Ports.PORT16, True)

loaderPiston = Pneumatics(brain.three_wire_port.a)
descorePiston = Pneumatics(brain.three_wire_port.h)
outPiston = Pneumatics(brain.three_wire_port.b)

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

    def __init__(self, yourSensor, brain: Brain, leftMotorGroup: MotorGroup, rightMotorGroup: MotorGroup, speedCap: int = 100, KP: float = 1, KI: float = 0, KD: float = 0):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.left = leftMotorGroup
        self.right = rightMotorGroup
        self.yourSensor = yourSensor
        self.brain = brain
        self.output:float = 0
        self.speedCap:int = speedCap

    def run (self, desiredValue: int, tollerance: float, settleTime: float = 0.5):
        """Run turn PID and set motor velocities until target heading stabilised."""

        self.right.spin(FORWARD, 0)
        self.left.spin(FORWARD, 0)

        totalError:float = 0.0
        i = 0
        if desiredValue - self.yourSensor() > 0:
            if desiredValue - self.yourSensor() <= 180:
                error:float = desiredValue - self.yourSensor()
            elif desiredValue - self.yourSensor() > 180:
                error:float = desiredValue - self.yourSensor() - 360
        else:
            if desiredValue - self.yourSensor() >= -180:
                error:float = desiredValue - self.yourSensor()
            elif desiredValue - self.yourSensor() < -180:
                error:float = 360 + (desiredValue - self.yourSensor())

        errorList = [error]
        previousError:float = error

        while abs(max(errorList, key=abs)) > tollerance:
            i += 1
            if desiredValue - self.yourSensor() > 0:
                if desiredValue - self.yourSensor() <= 180:
                    error:float = desiredValue - self.yourSensor()
                elif desiredValue - self.yourSensor() > 180:
                    error:float = desiredValue - self.yourSensor() - 360
            else:
                if desiredValue - self.yourSensor() >= -180:
                    error:float = desiredValue - self.yourSensor()
                elif desiredValue - self.yourSensor() < -180:
                    error:float = 360 + (desiredValue - self.yourSensor())

            derivative = (error - previousError) / 0.050
            totalError += error
            self.output = min(error * self.KP + derivative * self.KD + (totalError * 0.050) * self.KI, (self.speedCap if error * self.KP + derivative * self.KD + (totalError * 0.050) * self.KI > 0 else -self.speedCap), key=abs)
            self.left.set_velocity(self.output, PERCENT)
            self.right.set_velocity(-self.output, PERCENT)
            wait(50)
            previousError = error
            errorList.append(error)
            if len(errorList) > settleTime/0.050:
                errorList.pop(0)

    def tune(self, desiredValue: int, tollerance: float, settleTime: float = 0.5, sd_file_name = "pidData.csv", stopButton = False):
        """Run tuning loop similar to PID.tune but saves a CSV containing PID data.

        CSV columns:
            time, proportional, derivative, integral, output
        """
        if stopButton:
            stop = button(60, 220, 250, 10, Color.RED, "terminate")
            stop.draw()
            brain.screen.render()

        csvHeaderText:str = "time, proportional, derivative, integral, output, desiredValue, angle"
        data_buffer:str = csvHeaderText + "\n"
        self.right.spin(FORWARD, 0)
        self.left.spin(FORWARD, 0)

        totalError:float = 0.0
        i = 0
        if desiredValue - self.yourSensor() > 0:
            if desiredValue - self.yourSensor() <= 180:
                error:float = desiredValue - self.yourSensor()
            elif desiredValue - self.yourSensor() > 180:
                error:float = desiredValue - self.yourSensor() - 360
        else:
            if desiredValue - self.yourSensor() >= -180:
                error:float = desiredValue - self.yourSensor()
            elif desiredValue - self.yourSensor() < -180:
                error:float = 360 + (desiredValue - self.yourSensor())

        errorList = [error]
        previousError:float = error

        while abs(max(errorList, key=abs)) > tollerance:
            i += 1
            if desiredValue - self.yourSensor() > 0:
                if desiredValue - self.yourSensor() <= 180:
                    error:float = desiredValue - self.yourSensor()
                elif desiredValue - self.yourSensor() > 180:
                    error:float = desiredValue - self.yourSensor() - 360
            else:
                if desiredValue - self.yourSensor() >= -180:
                    error:float = desiredValue - self.yourSensor()
                elif desiredValue - self.yourSensor() < -180:
                    error:float = 360 + (desiredValue - self.yourSensor())

            derivative = (error - previousError) / 0.050
            totalError += error
            self.output = min(error * self.KP + derivative * self.KD + (totalError * 0.050) * self.KI, (self.speedCap if error * self.KP + derivative * self.KD + (totalError * 0.050) * self.KI > 0 else -self.speedCap), key=abs)
            self.left.set_velocity(self.output, PERCENT)
            self.right.set_velocity(-self.output, PERCENT)
            wait(50)
            previousError = error
            errorList.append(error)
            if len(errorList) > settleTime/0.050:
                errorList.pop(0)

            # save one row of data
            data_buffer += str(i * 0.050) + ","
            data_buffer += "%.3f" % (error*self.KP) + ","
            data_buffer += "%.3f" % (derivative * self.KD)  + ","
            data_buffer += "%.3f" % (totalError * 0.050 * self.KI) + ","
            data_buffer += "%.3f" % self.output + ","
            data_buffer += str(desiredValue) + ","
            data_buffer += "%.3f" % self.yourSensor() + "\n"

            if stopButton and stop.isPressed(self.brain.screen.x_position(),self.brain.screen.y_position()):
                break

        self.brain.sdcard.savefile(sd_file_name, bytearray(data_buffer, 'utf-8'))


# --------------------
# PID setup
# --------------------
# create a turnPID instance for drivetrain rotation
rotatePID = turnPID(yourSensor= gyro.heading , brain = brain, leftMotorGroup=left, rightMotorGroup=right, speedCap=20,
                     KP = 0.42,
                     KI = 0.02,
                     KD = 0.07
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

def Longgoal():
    intakeMotor.spin(FORWARD, 60, PERCENT)
    storageMotor.spin(FORWARD, 80, PERCENT)
    outMotor.spin(FORWARD, -80, PERCENT)

def Stopallmotors():
    intakeMotor.stop()
    storageMotor.stop()
    outMotor.stop()

def stopdrivetrain(sec):
    wait(sec, SECONDS)
    left.stop
    right.stop

# --------------------
# autonomous routines
# --------------------
def tune():
    """Autonomous routine to tune turn PID for various angles.
    """
    for i in range(30, 360, 30):
        right.spin(FORWARD, 0)
        left.spin(FORWARD, 0)
        rotatePID.tune(i, 2,sd_file_name='turnPID'+ str(i) + '.csv', stopButton=True)
        right.stop(HOLD)
        left.stop(HOLD)
        wait(2, SECONDS)
        right.spin(FORWARD, 0)
        left.spin(FORWARD, 0)
        rotatePID.tune(0, 2,sd_file_name='turnPID'+ str(-i) + '.csv', stopButton=True)
        wait(2, SECONDS)
        right.stop(HOLD)
        left.stop(HOLD)
    

def Left():
    outPiston.open()
    intakeMotor.spin(FORWARD, 80, PERCENT)
    storageMotor.spin(REVERSE, 100, PERCENT)
    forward(320, 10)
    rotatePID.tune(340, 2)
    forward(300, 10)
    rotatePID.tune(225, 2)
    forward(-400, 10)
    storageMotor.spin(FORWARD, 80, PERCENT)
    intakeMotor.spin(FORWARD, 60, PERCENT)
    outMotor.spin(FORWARD, 80, PERCENT)
    wait(2.5, SECONDS)
    forward(1200, 10)
    rotatePID.tune(180, 2)
    forward(-500, 10)

def Right():
    outPiston.open()
    intakeMotor.spin(FORWARD, 80, PERCENT)
    storageMotor.spin(REVERSE, 100, PERCENT)
    forward(320, 10)
    rotatePID.tune(45, 2)
    forward(300, 10)
    rotatePID.tune(135, 2)
    forward(850, 10)
    rotatePID.tune(180, 2)
    forward(-1000, 10)
    storageMotor.spin(FORWARD, 80, PERCENT)
    intakeMotor.spin(FORWARD, 60, PERCENT)
    outMotor.spin(FORWARD, 80, PERCENT)

def FullautonV1():
    # start
    outPiston.open()                                    # Extension outtake
    #start to preload in long goal
    forward(-795, 15)                                   # drive backwards
    rotatePID.tune(-90, 2)                              # turn to -90°
    forward(-555, 25)                                   # drive backwards to long goal
    forward(-40, 5)
    stopdrivetrain(2)
    Longgoal()                                          # outake preload long goal
    wait(0.7, SECONDS)                                  # wait for preload to be scored
    Stopallmotors()
    # loader 1
    loaderPiston.open()                                 # open the loader mech
    intakeMotor.spin(FORWARD, 80, PERCENT)              # spin intake and storage inwards
    storageMotor.spin(REVERSE, 100, PERCENT)
    forward(720, 20)                                    # drive forward to the loader
    wait(1.5, SECONDS)                                  # wait for a couple of blocks to come out the loader
    Stopallmotors()                                     # stop intake and outtake
    # score red blocks loader 1
    forward(-700, 25)                                   # drive backwards to long goal
    forward(-40, 5)
    stopdrivetrain(1)
    loaderPiston.close()                                # close the loader mech
    storageMotor.spin(FORWARD, 80, PERCENT)             # outtake the blue blocks
    intakeMotor.spin(FORWARD, -80, PERCENT)
    wait(0.85, SECONDS)                                 # time to outtake blue blocks
    Longgoal()                                          # score in the long goal
    wait(3.5, SECONDS)
    Stopallmotors()
    # push blocks in control zone
    forward(180, 15)                                    # drive away from long goal
    rotatePID.tune(0, 2)                                # turn to get to the side of long goal
    forward(270, 15)
    wait(0.2, SECONDS)
    rotatePID.tune(-90, 2)
    descorePiston.open()                                # open the descore mech
    wait(1, SECONDS)
    descorePiston.close()
    forward(-850, 25)                                   # push blocks in control zone
    # go to second loader
    descorePiston.open()
    forward(800, 20)
    turnPID(0, 2)
    forward(2000, 20)
    turnPID(-90, 2)
    forward(-600, 20)
    forward(-40, 5)
    loaderPiston.open()
    forward(700, 20)



def fullautonV2():
    # start
    outPiston.open()                                    # Extension outtake
    #start to preload in long goal
    forward(-795, 15)                                   # drive backwards
    rotatePID.tune(90, 2)                               # turn to -90°
    forward(-555, 25)                                   # drive backwards to long goal
    forward(-40, 5)
    stopdrivetrain(2)
    Longgoal()                                          # outake preload long goal
    wait(0.7, SECONDS)                                  # wait for preload to be scored
    Stopallmotors()
    # loader 1
    loaderPiston.open()                                 # open the loader mech
    intakeMotor.spin(FORWARD, 80, PERCENT)              # spin intake and storage inwards
    storageMotor.spin(REVERSE, 100, PERCENT)
    forward(720, 20)                                    # drive forward to the loader
    wait(1.5, SECONDS)                                  # wait for blocks to come out the loader
    Stopallmotors()                                     # stop intake and outtake
    # score blocks loader 1
    forward(-700, 25)                                   # drive backwards to long goal
    intakeMotor.spin(FORWARD, 60, PERCENT)
    storageMotor.spin(FORWARD, 30, PERCENT)             # slower so that the blocks come out 1 by 1
    outMotor.spin(FORWARD, -80, PERCENT)


def backupauton():
    intakeMotor.spin(FORWARD, 60, PERCENT)
    wait(2, SECONDS)
    intakeMotor.stop()   



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
        return 3/4*((x**k)/10**((k-1)*2))
    else:
        return -3/4*((x**k)/10**((k-1)*2))


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
        intakeMotor.spin(FORWARD, 60, PERCENT)
        storageMotor.spin(FORWARD, 100, PERCENT)
        outMotor.spin(REVERSE, 80, PERCENT) 
    elif controller_1.buttonL2.pressing():
        intakeMotor.spin(FORWARD, 60, PERCENT)
        storageMotor.spin(FORWARD, 80, PERCENT)
        outMotor.spin(FORWARD, 80, PERCENT)
    elif controller_1.buttonR1.pressing():
        intakeMotor.spin(FORWARD, 60, PERCENT)
        storageMotor.spin(REVERSE, 80, PERCENT)
        outMotor.stop(BRAKE)
    elif controller_1.buttonR2.pressing():
        intakeMotor.spin(REVERSE, 60, PERCENT)
        storageMotor.spin(FORWARD, 80, PERCENT)
        outMotor.stop(BRAKE)
    else:
        intakeMotor.stop(BRAKE)
        storageMotor.stop(BRAKE)
        outMotor.stop(BRAKE)

def loaderMechControl():
    """Toggles loader piston using controller button B.
    """
    if controller_1.buttonB.pressing():
        if loaderPiston.value() == 1:
            loaderPiston.close()
        else:
            loaderPiston.open()
    while controller_1.buttonB.pressing():
        wait(1, MSEC)

def descoreMechControl():
    """Toggles descore piston using controller button Down.
    """
    if controller_1.buttonDown.pressing():
        if descorePiston.value() == 1:
            descorePiston.close()
        else:
            descorePiston.open()
    while controller_1.buttonDown.pressing():
        wait(1, MSEC)

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
        self.selected = lambda: None

    def display(self):
        """Show the selector UI and return the selected autonomous function.

        The method blocks until the user confirms an auton on the touchscreen.
        """
        buttons = []
        brain.screen.draw_image_from_file(self.background, 0, 0)

        # create a vertical list of buttons from provided names
        for i in range(1, len(self.autons) + 1):
            if i < 5:
                buttons.append(button(50, 220, 10, 10 + (i-1)*60, Color.GREEN, str(self.names[i-1])))
            else:
                buttons.append(button(50, 220, 250, 10 + (i-5)*60, Color.GREEN, str(self.names[i-1])))
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
                        # print description text
                        brain.screen.set_pen_color(Color.WHITE)
                        brain.screen.set_cursor(5, 1)
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
    [Left, Right, tune, Fullautongoed, lambda: None, lambda: None, lambda: None, lambda: None],
    ["Left", "Right", "Tune", "Fullauton", "empty", "empty", "empty", "empty"],
    ["LEFT\n placement:\n  paralel with wall\n  contacting start of Left park zone corner\n  with right back", "RIGHT\n placement:\n  paralel with wall\n  contacting start of Right park zone corner\n  with left back","", "", "", "", "", ""],
    "background.png"
    )

def user_control():
    brain.screen.clear_screen()
    brain.screen.print("user control code")
    outPiston.open()
    while True:
        arcadeDriveGraph(left, right, controller_1)
        inOutControl()
        loaderMechControl()
        descoreMechControl()
        wait(20, MSEC)

# show selector COMMENT OUT IF NOT USING AUTON
# selector.display()

# create competition instance
comp = Competition(user_control, FullautonV1)

