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

# vex device config
brain = Brain()
left_1 = Motor(Ports.PORT20, GearSetting.RATIO_6_1, False)
left_2 = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
left_3 = Motor(Ports.PORT18, GearSetting.RATIO_6_1, False)
left = MotorGroup(left_1, left_2, left_3)
right_1 = Motor(Ports.PORT10, GearSetting.RATIO_6_1, True)
right_2 = Motor(Ports.PORT9, GearSetting.RATIO_6_1, True)
right_3 = Motor(Ports.PORT8, GearSetting.RATIO_6_1, True)
right = MotorGroup(right_1, right_2, right_3)
intake = Motor(Ports.PORT1, GearSetting.RATIO_18_1, False)
intakeSort = Motor(Ports.PORT2, False)
out = Motor(Ports.PORT11, False)
gyro = Inertial(Ports.PORT17)
controller_1 = Controller()

# PID classes
class PID:
    "a beautiful well made pid system for all vex uses"

    def __init__(self ,yourSensor ,brain: Brain, KP: float = 1, KI: float = 0, KD: float = 0):
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.yourSensor = yourSensor
        self.brain = brain

    def run(self, desiredValue: int, tollerance: float):
        previousError = 0
        totalError = 0
        i = 0
        while abs(desiredValue - self.yourSensor) > tollerance:
            i += 1
            error = desiredValue - self.yourSensor
            derivative = error - previousError
            self.output = error * self.KP + derivative * self.KD + totalError * self.KI
            totalError += error
            wait(50)

    def tune(self, desiredValue: int, tollerance: float, sd_file_name = "pidData.csv"):
        csvHeaderText = "time, error, derivative, totalError, output, desiredValue"

        data_buffer = csvHeaderText + "\n"

        errorGraph = []
        derivativeGraph = []
        totalErrorGraph = []
        tGraph = []

        previousError = 0
        totalError = 0
        i = 0

        while abs(desiredValue - self.yourSensor) > tollerance:
            i += 1
            error = desiredValue - self.yourSensor
            derivative = error - previousError
            self.output = error * self.KP + derivative * self.KD + totalError * self.KI
            totalError += error
            wait(50)

            tGraph.append(i * 50)
            data_buffer += "%1.3f" % i * 50 + ","
            data_buffer += "%1.3f" % error + ","
            data_buffer += "%1.3f" % derivative + ","
            data_buffer += "%1.3f" % totalError + ","
            data_buffer += "%1.3f" % self.output + ","
            data_buffer += "%1.3f" % desiredValue + "\n"

            errorGraph.append(error)

            derivativeGraph.append(derivative)
            totalErrorGraph.append(totalError)
        self.brain.sdcard.savefile(sd_file_name, bytearray(data_buffer, 'utf-8'))

class turnPID(PID):
    
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
        previousError = 0
        totalError = 0
        i = 0
        while abs(desiredValue - self.yourSensor) > tollerance:
            i += 1
            error = desiredValue - self.yourSensor
            derivative = error - previousError
            self.output = error * self.KP + derivative * self.KD + totalError * self.KI
            self.left.set_velocity(self.output, PERCENT)
            self.right.set_velocity(-self.output, PERCENT)
            totalError += error
            wait(50)

    def tune(self, desiredValue: int, tollerance: float, sd_file_name = "pidData.csv"):
        data_buffer: str = ""
        csvHeaderText:str = "time, error, derivative, totalError, output, desiredValue"

        data_buffer:str = csvHeaderText + "\n"

        previousError = 0
        totalError = 0
        i = 0

        while abs(desiredValue - self.yourSensor) > tollerance:
            i += 1
            error:float = desiredValue - self.yourSensor
            derivative = error - previousError
            self.output = error * self.KP + derivative * self.KD + totalError * self.KI
            self.left.set_velocity(self.output, PERCENT)
            self.right.set_velocity(-self.output, PERCENT)
            totalError += error
            wait(50)

            data_buffer += str(i * 50) + ","
            data_buffer += "%1.3f" % error + ","
            data_buffer += "%1.3f" % derivative + ","
            data_buffer += "%1.3f" % totalError + ","
            data_buffer += "%1.3f" % self.output + ","
            data_buffer += str(desiredValue) + "\n"

        self.brain.sdcard.savefile(sd_file_name, bytearray(data_buffer, 'utf-8'))

# PID setup    
rotateTune = turnPID(yourSensor= gyro, brain = brain ,leftMotorGroup=left, rightMotorGroup=right)
rotateTune.KP = 100
rotateTune.KI = 0
rotateTune.KD = 0

# autonomous 
def auton():
    """
    place automonous code here
    """
    for i in range(0, 360, 30):
        right.spin(FORWARD, 0)
        left.spin(FORWARD, 0)
        rotateTune.tune(i, 0.2, 'turnPID'+ str(i) + '.csv')

# user control functions
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

def intakeControl():
    """
    Controls the intake motor using the controller buttons.

    usecase:
        repeat in loop when driver control is active
    """
    if controller_1.buttonR1.pressing():
        intake.spin(FORWARD, 100, PERCENT)
        intakeSort.spin(FORWARD, 100, PERCENT)
    elif controller_1.buttonR2.pressing():  
        intake.spin(REVERSE, 100, PERCENT)
        intakeSort.spin(REVERSE, 100, PERCENT)
    else:
        intake.stop(HOLD)
        intakeSort.stop(HOLD)

# UI classes
class button:
    def __init__(self, height:int, width:int, posX:int, posY:int, color, text:str) -> None:
        self.height = height
        self.width = width
        self.posX = posX
        self.posY = posY
        self.Pressed = False
        self.color = color
        self.text = text

    def draw(self):
        brain.screen.set_pen_color(self.color)
        brain.screen.draw_rectangle(self.posX, self.posY, self.width, self.height, self.color)
        brain.screen.set_pen_color(Color.BLACK)
        brain.screen.print_at(self.text,x= self.posX + 10,y= self.posY + self.height//2 - 5,opaque= False)

    def isPressed(self, touchX:int, touchY:int) -> bool:
        if touchX > self.posX and touchX < self.posX + self.width and touchY > self.posY and touchY < self.posY + self.height:
            self.Pressed = True
        else:
            self.Pressed = False
        return self.Pressed
    
class autonSelector:
    def __init__(self, autons:list,names:list,doc:list, background) -> None:
        self.autons = autons
        self.names = names
        self.doc = doc
        self.background = background
        self.selected = None

    def display(self):
        buttons = []
        brain.screen.draw_image_from_file(self.background, 0, 0)
        for i in range(1,len(self.autons)+1):
            buttons.append(button(50, 220, 10, 10 + (i-1)*60, Color.GREEN, str(self.names[i-1])))
            buttons[i-1].draw()
        brain.screen.render()

        while True:
            if brain.screen.pressing():
                touchX = brain.screen.x_position()
                touchY = brain.screen.y_position()
                for i in range(len(buttons)):
                    if buttons[i].isPressed(touchX, touchY):
                        brain.screen.clear_screen()
                        brain.screen.draw_image_from_file(self.background, 0, 0)
                        brain.screen.render()
                        confirm = button(60, 220, 10, 10, Color.GREEN, "Confirm")
                        cancel = button(60, 220, 250, 10, Color.RED, "Cancel")
                        confirm.draw()
                        cancel.draw()
                        brain.screen.print_at(self.doc[i],x= 10,y= 80,opaque= False)
                        wait(1,SECONDS)
                        brain.screen.render()

                        while True:
                            if brain.screen.pressing():
                                touchX = brain.screen.x_position()
                                touchY = brain.screen.y_position()
                                if confirm.isPressed(touchX, touchY):
                                    self.selected = self.autons[i]
                                    return self.selected
                                elif cancel.isPressed(touchX, touchY):
                                    brain.screen.clear_screen()
                                    brain.screen.draw_image_from_file(self.background, 0, 0)
                                    for j in range(len(buttons)):
                                        buttons[j].draw()
                                    brain.screen.render()
                                    break
                            wait(100)
            wait(100)

# UI setup
selector = autonSelector([auton],["auton"],["placement enzo blablabla"], "background.png")

# user control 
def user_control():
    brain.screen.clear_screen()
    brain.screen.print("user control code")
    while True:
        arcadeDrive(left, right, controller_1)
        intakeControl()
        wait(20, MSEC)

# call auton selector
selector.display()

# create competition instance
comp = Competition(user_control, selector.selected)

# actions to do when the program starts
brain.screen.clear_screen()
brain.screen.draw_image_from_file("background.png", 0, 0)
brain.screen.render()
