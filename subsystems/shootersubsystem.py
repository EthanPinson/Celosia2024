from commands2 import Subsystem, Command, cmd
from constants import ShooterConstants as SC
from wpilib import Spark

class ShooterSubsystem(Subsystem):
    __innerMotor: Spark
    __outerMotor: Spark

    def __init__(self) -> None:
        super().__init__()

        self.__innerMotor = Spark(SC.INNER_ID)
        self.__outerMotor = Spark(SC.OUTER_ID)

    def __setSpeed(self, mult: int):
        self.__innerMotor.set(SC.INNER_SPEED * mult)
        self.__outerMotor.set(SC.OUTER_SPEED * mult)

    def runShooter(self) -> Command:
        return cmd.runOnce(
            lambda: self.__setSpeed(1), self
        )
    
    def runShooterRev(self) -> Command:
        return cmd.runOnce(
            lambda: self.__setSpeed(-1), self
        )
    
    def stopShooter(self) -> Command:
        return cmd.runOnce(
            lambda: self.__setSpeed(0), self
        )