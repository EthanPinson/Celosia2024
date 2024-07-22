from commands2 import Subsystem, Command, cmd
from constants import FeederConstants as FC
from wpilib import Spark

class FeederSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()
        self.__motor = Spark(FC.ID)

    def setSpeed(self, speed: float) -> Command:
        return cmd.runOnce(lambda: self.__motor.set(speed), self)