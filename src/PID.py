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
"""import matplotlib.pyplot as plt"""
from main import left, right, controller_1, brain


# Brain should be defined by default
brain=Brain()

brain.screen.print("Hello V5")

yourSensor = 0

class PID:
    "a beautiful well made pid system for all vex uses"

    def __init__(self, KP = 1, KI = 0, KD = 0):
        self.KP = KP
        self.KI = KI
        self.KD = KD

    def run(self, desiredValue: int, tollerance: float):
        previousError = 0
        totalError = 0
        i = 0
        while abs(desiredValue - yourSensor) > tollerance:
            i += 1
            error = desiredValue - yourSensor
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

        while abs(desiredValue - yourSensor) > tollerance or close == False:
            i += 1
            error = desiredValue - yourSensor
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

        while abs(desiredValue - yourSensor) > tollerance == False:
            i += 1
            error = desiredValue - yourSensor
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
        brain.sdcard.savefile(sd_file_name, bytearray(data_buffer, 'utf-8'))

class turnPID(PID):
    
    def __init__(self, leftMotorGroup: MotorGroup, rightMotorGroup: MotorGroup, KP = 1, KI = 0, KD = 0):
        super().__init__()
        self.KP = KP
        self.KI = KI
        self.KD = KD
        self.left = leftMotorGroup
        self.right = rightMotorGroup

    def run (self, desiredValue: int, tollerance: float):
        previousError = 0
        totalError = 0
        i = 0
        while abs(desiredValue - yourSensor) > tollerance:
            i += 1
            error = desiredValue - yourSensor
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

        while abs(desiredValue - yourSensor) > tollerance == False:
            i += 1
            error = desiredValue - yourSensor
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
        brain.sdcard.savefile(sd_file_name, bytearray(data_buffer, 'utf-8'))
        

