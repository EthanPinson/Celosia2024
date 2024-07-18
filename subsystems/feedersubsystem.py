from commands2 import Subsystem, Command, cmd
from constants import FeederConstants as FC
from wpilib import Spark

class FeederSubsystem(Subsystem):
    __motor: Spark

    def __init__(self) -> None:
        super().__init__()

        self.__motor = Spark(FC.ID)

    def setSpeed(self, speed: float):
        self.__motor.set(speed)

    def runFeeder(self) -> Command:
        return cmd.runOnce(
            lambda: self.setSpeed(FC.SHOOT_SPEED), self)
    
    def runFeederRev(self) -> Command:
        return cmd.runOnce(
            lambda: self.setSpeed(-1.*FC.INTAKE_SPEED), self)
    
    def stopFeeder(self) -> Command:
        return cmd.runOnce(
            lambda: self.setSpeed(0.), self)