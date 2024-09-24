from constants import GyroConstants as Gc
from commands2 import Subsystem
from wpilib import SmartDashboard
from navx import AHRS

class GyroSub(Subsystem):
    def __init__(self):
        self.__g = AHRS()
        self.reset()

    def periodic(self):
        SmartDashboard.putNumber("Gyro", self.getRot())

    def getRot(self): return self.__g.getRotation2d().degrees()
    
    def reset(self): self.__g.reset()