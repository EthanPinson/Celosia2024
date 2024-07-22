from commands2 import Subsystem, Command, cmd
from constants import ShooterConstants as SC
from wpilib import Spark

class ShooterSubsystem(Subsystem):
    def __init__(self) -> None:
        super().__init__()

        self.__motorA = Spark(SC.INNER_ID)
        self.__motorB = Spark(SC.OUTER_ID)
        self.__motorA.addFollower(self.__motorB)

    def setSpeed(self, speed: float) -> Command:
        return cmd.runOnce(lambda: self.__motorA.set(speed), self)