from commands2 import Subsystem, Command, cmd
from constants import FeederConstants as FC
from wpilib import Spark

class FeederSubsystem(Subsystem):
    __motor: Spark

    def __init__(self) -> None:
        super().__init__()

        self.__motor = Spark(FC.ID)

    def __setSpeed(self, mult: int):
        self.__motor.set(FC.NOMINAL_SPEED * mult)

    def runFeeder(self) -> Command:
        return cmd.runOnce(
            lambda: self.__setSpeed(1), self
        )
    
    def runFeederRev(self) -> Command:
        return cmd.runOnce(
            lambda: self.__setSpeed(-1), self
        )
    
    def stopFeeder(self) -> Command:
        return cmd.runOnce(
            lambda: self.__setSpeed(0), self
        )