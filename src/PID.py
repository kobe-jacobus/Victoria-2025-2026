# ---------------------------------------------------------------------------- #
#                                                                              #
# 	Module:       main.py                                                      #
# 	Author:       RubenCheyns                                                  #
# 	Created:      2/20/2025, 12:47:16 PM                                       #
# 	Description:  V5 project                                                   #
#                                                                              #
# ---------------------------------------------------------------------------- #

# Library imports
from vex import *

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

    """def tuneOld(self, desiredValue, tollerance):
        def on_close(event):
            global close
            plt.close()
            close = True

        errorGraph = []
        derivativeGraph = []
        totalErrorGraph = []
        tGraph = []

        plt.ion()
        graphError = plt.plot(tGraph, errorGraph, color = 'r')[0]
        graphDerivative = plt.plot(tGraph, derivativeGraph, color = 'g')[0]
        graphTotalError = plt.plot(tGraph, totalErrorGraph, color = 'b')[0]
        graphError.canvas.mpl_connect('close_event', on_close)
        plt.legend(["Error", "Derivative", "Total Error"])
        plt.xlabel("Time (ms)")
        plt.title("PID Tuning")
        plt.grid()
        plt.show()

        previousError = 0
        totalError = 0
        i = 0

        while abs(desiredValue - self.yourSensor) > tollerance or close == False:
            i += 1
            error = desiredValue - self.yourSensor
            derivative = error - previousError
            self.output = error * self.KP + derivative * self.KD + totalError * self.KI
            totalError += error
            wait(50)
            errorGraph.append(error)
            derivativeGraph.append(derivative)
            totalErrorGraph.append(totalError)
            tGraph.append(i * 50)
            plt.xlim(tGraph[0], tGraph[-1])
            plt.ylim(min(min(errorGraph), min(derivativeGraph), min(totalErrorGraph)), max(max(errorGraph), max(derivativeGraph), max(totalErrorGraph)))
            if len(tGraph) > 100:
                tGraph.pop(0)
                errorGraph.pop(0)
                derivativeGraph.pop(0)
                totalErrorGraph.pop(0)
                graphError.remove()
                graphDerivative.remove()
                graphTotalError.remove()
            graphError = plt.plot(tGraph, errorGraph, color = 'r')[0]
            graphDerivative = plt.plot(tGraph, derivativeGraph, color = 'g')[0]
            graphTotalError = plt.plot(tGraph, totalErrorGraph, color = 'b')[0]
            plt.legend(["Error", "Derivative", "Total Error"])
            plt.xlabel("Time (s)")
            plt.title("PID Tuning")

        self.KP = float(input("Enter KP: "))
        self.KI = float(input("Enter KI: "))
        self.KD = float(input("Enter KD: "))    
"""

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

        while abs(desiredValue - self.yourSensor) > tollerance == False:
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
        csvHeaderText = "time, error, derivative, totalError, output, desiredValue"

        data_buffer = csvHeaderText + "\n"

        errorGraph = []
        derivativeGraph = []
        totalErrorGraph = []
        tGraph = []

        previousError = 0
        totalError = 0
        i = 0

        while abs(desiredValue - self.yourSensor) > tollerance == False:
            i += 1
            error = desiredValue - self.yourSensor
            derivative = error - previousError
            self.output = error * self.KP + derivative * self.KD + totalError * self.KI
            self.left.set_velocity(self.output, PERCENT)
            self.right.set_velocity(-self.output, PERCENT)
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
        

